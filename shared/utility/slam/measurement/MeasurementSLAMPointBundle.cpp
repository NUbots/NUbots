/*
* MIT License
*
* Copyright (c) 2025 NUbots
*
* This file is part of the NUbots codebase.
* See https://github.com/NUbots/NUbots for further info.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include "MeasurementSLAMPointBundle.hpp"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "../camera/Camera.hpp"
#include "../gaussian/GaussianInfo.hpp"
#include "../system/SystemBase.hpp"
#include "../system/SystemEstimator.hpp"
#include "../system/SystemSLAM.hpp"
#include "../association_util.hpp"
#include "../rotation.hpp"

namespace utility::slam::measurement {

    MeasurementPointBundle::MeasurementPointBundle(double time,
                                                const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                                                const Camera& camera)
        : MeasurementSLAM(time, camera), Y_(Y), sigma_(5.0)  // TODO: Assignment(s)
    {
        // updateMethod_ = UpdateMethod::BFGSLMSQRT;
        updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
        // updateMethod_ = UpdateMethod::SR1TRUSTEIG;
        // updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    MeasurementSLAM* MeasurementPointBundle::clone() const {
        return new MeasurementPointBundle(*this);
    }

    Eigen::VectorXd MeasurementPointBundle::simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        Eigen::VectorXd y(Y_.size());
        throw std::runtime_error("Not implemented");
        return y;
    }

    double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        const SystemSLAM& systemSLAM = dynamic_cast<const SystemSLAM&>(system);
        return logLikelihoodTemplated<double>(x, systemSLAM);
    }


    // gradient version using GaussianInfo log function with chain rule
    double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd& x,
                                                const SystemEstimator& system,
                                                Eigen::VectorXd& g) const {
        const SystemSLAM& systemSLAM = dynamic_cast<const SystemSLAM&>(system);

        // compute log-likelihood using the base function
        double logLik = logLikelihood(x, system);

        // initialize gradient
        g = Eigen::VectorXd::Zero(x.size());

        // create measurement noise model for a single point
        Eigen::MatrixXd S                           = sigma_ * Eigen::MatrixXd::Identity(2, 2);
        gaussian::GaussianInfo<double> measurementModel = gaussian::GaussianInfo<double>::fromSqrtMoment(S);

        // compute gradient for all associated feature/landmark pairs
        for (std::size_t j = 0; j < idxFeatures_.size(); ++j) {
            if (idxFeatures_[j] >= 0) {
                int detectionIdx   = idxFeatures_[j];
                size_t landmarkIdx = visibleLandmarks_[j];

                // predict single point with Jacobian
                Eigen::MatrixXd J_h;
                Eigen::Vector2d h_pred = predictFeature(x, J_h, systemSLAM, landmarkIdx);

                // get measured point position
                Eigen::Vector2d y_i = Y_.col(detectionIdx);

                // compute residual
                Eigen::Vector2d residual = y_i - h_pred;

                // use GaussianInfo log function to get gradient w.r.t. residual
                Eigen::VectorXd g_residual;
                measurementModel.log(residual, g_residual);

                // chain rule
                g += -J_h.transpose() * g_residual;
            }
        }

        // no gradient contribution from penalty term (constant w.r.t. x)

        return logLik;
    }

    // hessian version using forward-mode autodiff
    double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd& x,
                                                const SystemEstimator& system,
                                                Eigen::VectorXd& g,
                                                Eigen::MatrixXd& H) const {
        const SystemSLAM& systemSLAM = dynamic_cast<const SystemSLAM&>(system);

        // forward-mode autodifferentiation with dual2nd for Hessian
        autodiff::dual2nd logLik_dual;
        Eigen::VectorX<autodiff::dual2nd> x_dual = x.cast<autodiff::dual2nd>();

        H = hessian(
            [&](const Eigen::VectorX<autodiff::dual2nd>& x_ad) {
                return logLikelihoodTemplated<autodiff::dual2nd>(x_ad, systemSLAM);
            },
            wrt(x_dual),
            at(x_dual),
            logLik_dual,
            g);

        return val(logLik_dual);
    }

    void MeasurementPointBundle::update(SystemBase& system) {
        SystemSLAM& systemSLAM = dynamic_cast<SystemSLAM&>(system);

        // Get camera state for visibility checks
        Eigen::VectorXd x       = systemSLAM.density.mean();
        Eigen::Vector3d rCNn    = systemSLAM.cameraPositionDensity(camera_).mean();
        Eigen::Vector3d Thetanc = systemSLAM.cameraOrientationEulerDensity(camera_).mean();
        Eigen::Matrix3d Rnc     = rpy2rot(Thetanc);

        // Initialize consecutive failures vector if needed
        if (consecutiveFailures_.size() != systemSLAM.numberLandmarks()) {
            consecutiveFailures_.resize(systemSLAM.numberLandmarks(), 0);
        }

        // Identify landmarks with matching features (FOV check + association)
        visibleLandmarks_.clear();
        for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
            std::size_t idx         = systemSLAM.landmarkPositionIndex(i);
            Eigen::Vector3d rPNn    = x.segment<3>(idx);

            // Transform to camera frame
            Eigen::Vector3d rPCc = Rnc.transpose() * (rPNn - rCNn);

            cv::Vec3d rPCc_cv(rPCc(0), rPCc(1), rPCc(2));
            if (camera_.isVectorWithinFOV(rPCc_cv)) {
                visibleLandmarks_.push_back(i);
            }
        }

        idxFeatures_ = associate(systemSLAM, visibleLandmarks_);

        // Update consecutive failures: only for visible landmarks
        for (std::size_t j = 0; j < visibleLandmarks_.size(); ++j) {
            std::size_t landmarkIdx = visibleLandmarks_[j];
            if (idxFeatures_[j] >= 0) {
                // Successfully associated - reset failure count
                consecutiveFailures_[landmarkIdx] = 0;
            }
            else {
                // Failed to associate - increment failure count
                consecutiveFailures_[landmarkIdx]++;
            }
        }

        // Determine which landmarks to delete
        std::vector<std::size_t> landmarksToDelete;

        // delete landmarks with >= 10 consecutive failures
        for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
            if (consecutiveFailures_[i] >= 10) {
                landmarksToDelete.push_back(i);
            }
        }

        // if at capacity, prune worst performers to make room
        int maxTotalLandmarks = 30;
        int currentTotal      = systemSLAM.numberLandmarks();

        if (currentTotal >= maxTotalLandmarks) {
            // Find landmarks with highest failure counts (not already marked for deletion)
            std::vector<std::pair<int, size_t>> failureRanking;  // (failures, landmarkIdx)
            for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
                if (consecutiveFailures_[i] < 10) {
                    failureRanking.push_back({consecutiveFailures_[i], i});
                }
            }

            // Sort by failures (descending - worst first)
            std::sort(failureRanking.begin(), failureRanking.end(), [](const auto& a, const auto& b) {
                return a.first > b.first;
            });

            // Delete worst performers to make room
            int spotsNeeded  = 5;  // Make room for 5 new landmarks
            int needToDelete = currentTotal - maxTotalLandmarks + spotsNeeded;
            for (int i = 0; i < std::min(needToDelete, (int) failureRanking.size()); ++i) {
                landmarksToDelete.push_back(failureRanking[i].second);
            }
        }

        // Sort deletion list and remove duplicates
        std::sort(landmarksToDelete.begin(), landmarksToDelete.end());
        landmarksToDelete.erase(std::unique(landmarksToDelete.begin(), landmarksToDelete.end()),
                                landmarksToDelete.end());

        // Delete landmarks in reverse order to maintain correct indices
        if (!landmarksToDelete.empty()) {
            std::cout << "Deleting " << landmarksToDelete.size() << " landmarks: ";
            for (auto it = landmarksToDelete.rbegin(); it != landmarksToDelete.rend(); ++it) {
                std::size_t landmarkIdx = *it;
                std::cout << landmarkIdx << "(f=" << consecutiveFailures_[landmarkIdx] << ") ";

                // Get state indices to keep (all except this landmark's 3 DOF)
                std::size_t stateIdx = systemSLAM.landmarkPositionIndex(landmarkIdx);
                std::vector<int> indicesToKeep;
                indicesToKeep.reserve(systemSLAM.density.dim() - 3);

                for (std::size_t i = 0; i < systemSLAM.density.dim(); ++i) {
                    if (i < stateIdx || i >= stateIdx + 3) {
                        indicesToKeep.push_back(static_cast<int>(i));
                    }
                }

                // Marginalize out this landmark from density
                systemSLAM.density = systemSLAM.density.marginal(indicesToKeep);

                // Remove from consecutive failures tracking
                consecutiveFailures_.erase(consecutiveFailures_.begin() + landmarkIdx);

                // Remove from visible landmarks and associations if present
                auto it_vis = std::find(visibleLandmarks_.begin(), visibleLandmarks_.end(), landmarkIdx);
                if (it_vis != visibleLandmarks_.end()) {
                    size_t position = std::distance(visibleLandmarks_.begin(), it_vis);
                    visibleLandmarks_.erase(it_vis);
                    idxFeatures_.erase(idxFeatures_.begin() + position);
                }

                // Decrement all landmark indices > landmarkIdx in visibleLandmarks_
                for (size_t& idx : visibleLandmarks_) {
                    if (idx > landmarkIdx) {
                        idx--;
                    }
                }
            }
            std::cout << std::endl;
        }

        // Identify surplus features that do not correspond to landmarks in the map
        std::vector<bool> detectionUsed(Y_.cols(), false);
        for (int idx : idxFeatures_) {
            if (idx >= 0) {
                detectionUsed[idx] = true;
            }
        }

        // Initialize up to Nmax - N new landmarks from best surplus features
        int maxVisibleLandmarksPerFrame = 10;
        currentTotal                    = systemSLAM.numberLandmarks();  // Recalculate after deletions
        int currentVisible              = visibleLandmarks_.size();

        // Calculate how many we can initialize
        int spotsAvailableInFrame   = maxVisibleLandmarksPerFrame - currentVisible;
        int spotsAvailableTotal     = maxTotalLandmarks - currentTotal;
        int landmarksToInitialize   = std::min(spotsAvailableInFrame, spotsAvailableTotal);

        if (landmarksToInitialize <= 0) {
            Measurement::update(system);
            return;
        }

        int landmarksInitializedThisFrame = 0;

        // Collect candidate detections: (detectionIdx, pixel, minDistToOthers)
        std::vector<std::tuple<size_t, Eigen::Vector2d, double>> candidates;

        double borderMargin  = 100.0;  // pixels
        double minSeparation = 150.0;  // pixels - minimum distance to existing landmarks

        for (std::size_t detectionIdx = 0; detectionIdx < Y_.cols(); ++detectionIdx) {
            if (detectionUsed[detectionIdx])
                continue;

            Eigen::Vector2d pixel = Y_.col(detectionIdx);

            // is detection too close to image border?
            bool tooCloseToEdge = (pixel(0) < borderMargin || pixel(0) > camera_.imageSize.width - borderMargin
                                || pixel(1) < borderMargin || pixel(1) > camera_.imageSize.height - borderMargin);
            if (tooCloseToEdge)
                continue;

            // find minimum distance to existing landmarks
            double minDistToExisting = std::numeric_limits<double>::max();
            for (size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
                std::size_t idx                   = systemSLAM.landmarkPositionIndex(i);
                Eigen::Vector3d rPNn_existing     = x.segment<3>(idx);
                Eigen::Vector3d rPCc_existing     = Rnc.transpose() * (rPNn_existing - rCNn);

                if (rPCc_existing(2) > 0) {
                    Eigen::Vector2d projected_pixel = camera_.vectorToPixel(rPCc_existing);
                    double distance                 = (projected_pixel - pixel).norm();
                    minDistToExisting               = std::min(minDistToExisting, distance);
                }
            }

            if (minDistToExisting < minSeparation)
                continue;

            // compute minimum distance to all other detections
            double minDistToOtherDetections = std::numeric_limits<double>::max();
            for (std::size_t otherIdx = 0; otherIdx < Y_.cols(); ++otherIdx) {
                if (otherIdx == detectionIdx)
                    continue;
                Eigen::Vector2d otherPixel = Y_.col(otherIdx);
                double distance            = (pixel - otherPixel).norm();
                minDistToOtherDetections   = std::min(minDistToOtherDetections, distance);
            }

            candidates.push_back(std::make_tuple(detectionIdx, pixel, minDistToOtherDetections));
        }

        // Sort candidates by minimum distance to other detections (descending - furthest first)
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return std::get<2>(a) > std::get<2>(b);
        });

        // Initialize landmarks from sorted candidates
        for (const auto& candidate : candidates) {
            if (landmarksInitializedThisFrame >= landmarksToInitialize) {
                break;
            }

            size_t candidateIdx          = std::get<0>(candidate);
            Eigen::Vector2d candidatePixel = std::get<1>(candidate);
            double candidateDist         = std::get<2>(candidate);

            // Check distance to already-initialized landmarks this frame
            bool tooCloseToNewLandmark = false;
            for (int alreadyInit = 0; alreadyInit < landmarksInitializedThisFrame; ++alreadyInit) {
                size_t newLandmarkIdx     = systemSLAM.numberLandmarks() - 1 - alreadyInit;
                std::size_t idx           = systemSLAM.landmarkPositionIndex(newLandmarkIdx);
                Eigen::Vector3d rPNn_new  = systemSLAM.density.mean().segment<3>(idx);
                Eigen::Vector3d rPCc_new  = Rnc.transpose() * (rPNn_new - rCNn);

                if (rPCc_new(2) > 0) {
                    Eigen::Vector2d projected_pixel = camera_.vectorToPixel(rPCc_new);
                    double distance                 = (projected_pixel - candidatePixel).norm();
                    if (distance < minSeparation) {
                        tooCloseToNewLandmark = true;
                        break;
                    }
                }
            }

            if (tooCloseToNewLandmark)
                continue;

            // All checks passed - initialize this landmark at arbitrary depth
            double arbitrary_depth = 15.0;  // meters

            // Backproject pixel to 3D at arbitrary depth
            cv::Vec2d pixel_cv(candidatePixel(0), candidatePixel(1));
            cv::Vec3d rPCc_cv        = camera_.pixelToVector(pixel_cv);
            Eigen::Vector3d rPCc_unit(rPCc_cv[0], rPCc_cv[1], rPCc_cv[2]);
            Eigen::Vector3d rPCc = rPCc_unit.normalized() * arbitrary_depth;

            // Transform to world frame
            Eigen::Vector3d rPNn = Rnc * rPCc + rCNn;

            // Create new landmark with prior
            Eigen::VectorXd mu_new = rPNn;
            double epsilon         = 1.0;  // Precision (low confidence in initial position)
            Eigen::MatrixXd Xi_new = epsilon * Eigen::MatrixXd::Identity(3, 3);
            Eigen::VectorXd nu_new = Xi_new * mu_new;

            gaussian::GaussianInfo<double> newLandmarkDensity = gaussian::GaussianInfo<double>::fromSqrtInfo(nu_new, Xi_new);
            systemSLAM.density *= newLandmarkDensity;
            landmarksInitializedThisFrame++;

            size_t newLandmarkIdx = systemSLAM.numberLandmarks() - 1;
            std::cout << "  Initialized point landmark " << newLandmarkIdx << " (dist=" << candidateDist
                    << ", depth=" << arbitrary_depth << "m)" << std::endl;

            // Add to visible landmarks and associate with detection for immediate update
            visibleLandmarks_.push_back(newLandmarkIdx);
            idxFeatures_.push_back(static_cast<int>(candidateIdx));

            // Initialize failure tracking for new landmark
            consecutiveFailures_.push_back(0);
        }

        // Perform measurement update
        Measurement::update(system);
    }

    // Image feature location for a given landmark and Jacobian
    Eigen::Vector2d MeasurementPointBundle::predictFeature(const Eigen::VectorXd& x,
                                                        Eigen::MatrixXd& J,
                                                        const SystemSLAM& system,
                                                        std::size_t idxLandmark) const {
        // Get camera pose from state
        Pose<double> Tnc;
        Tnc.translationVector = system.cameraPosition(camera_, x);     // rCNn
        Tnc.rotationMatrix    = system.cameraOrientation(camera_, x);  // Rnc

        // Get landmark position from state
        std::size_t idx         = system.landmarkPositionIndex(idxLandmark);
        Eigen::Vector3d rPNn = x.segment<3>(idx);

        // Transform to camera coordinates
        Eigen::Vector3d rPCc = Tnc.rotationMatrix.transpose() * (rPNn - Tnc.translationVector);

        // Get pixel coordinates with Jacobian w.r.t. camera coordinates
        Eigen::Matrix23d J_camera;  // Fixed: Different name from parameter
        Eigen::Vector2d rQOi = camera_.vectorToPixel(rPCc, J_camera);

        // Compute full Jacobian ∂h/∂x using chain rule and Appendix B expressions
        J.resize(2, x.size());
        J.setZero();

        // Extract state components
        Eigen::Vector3d rBNn    = x.segment<3>(6);
        Eigen::Vector3d Thetanb = x.segment<3>(9);

        // Get body pose
        Pose<double> Tnb;
        Tnb.rotationMatrix    = rpy2rot(Thetanb);
        Tnb.translationVector = rBNn;

        // Get camera-to-body transformation
        Pose<double> Tbc    = camera_.Tbc;
        Eigen::Matrix3d Rnb = Tnb.rotationMatrix;
        Eigen::Matrix3d Rbc = Tbc.rotationMatrix;

        // Equation (27a):
        Eigen::Matrix<double, 2, 3> dhj_drPNn = J_camera * Rbc.transpose() * Rnb.transpose();
        J.block<2, 3>(0, idx)                 = dhj_drPNn;

        // Equation (27b):
        Eigen::Matrix<double, 2, 3> dhj_drBNn = -J_camera * Rbc.transpose() * Rnb.transpose();
        J.block<2, 3>(0, 6)                   = dhj_drBNn;

        Eigen::Vector3d rPNn_minus_rBNn = rPNn - rBNn;

        // roll
        Eigen::Matrix3d dRx_dphi;
        rotx(Thetanb(0), dRx_dphi);
        Eigen::Matrix3d dRnb_dphi = rotz(Thetanb(2)) * roty(Thetanb(1)) * dRx_dphi;

        // pitch
        Eigen::Matrix3d dRy_dtheta;
        roty(Thetanb(1), dRy_dtheta);
        Eigen::Matrix3d dRnb_dtheta = rotz(Thetanb(2)) * dRy_dtheta * rotx(Thetanb(0));

        // yaw
        Eigen::Matrix3d dRz_dpsi;
        rotz(Thetanb(2), dRz_dpsi);
        Eigen::Matrix3d dRnb_dpsi = dRz_dpsi * roty(Thetanb(1)) * rotx(Thetanb(0));

        // Apply equation (27c) for each Euler angle
        J.col(9)  = J_camera * Rbc.transpose() * dRnb_dphi.transpose() * rPNn_minus_rBNn;
        J.col(10) = J_camera * Rbc.transpose() * dRnb_dtheta.transpose() * rPNn_minus_rBNn;
        J.col(11) = J_camera * Rbc.transpose() * dRnb_dpsi.transpose() * rPNn_minus_rBNn;

        return rQOi;
    }

    // Density of image feature location for a given landmark
    gaussian::GaussianInfo<double> MeasurementPointBundle::predictFeatureDensity(const SystemSLAM& system,
                                                                                  std::size_t idxLandmark) const {
        const std::size_t& nx = system.density.dim();
        const std::size_t ny  = 2;

        //   y   =   h(x) + v
        // \___/   \__________/
        //   ya  =   ha(x, v)
        //
        // Helper function to evaluate ha(x, v) and its Jacobian Ja = [dha/dx, dha/dv]
        const auto func = [&](const Eigen::VectorXd& xv, Eigen::MatrixXd& Ja) {
            assert(xv.size() == nx + ny);
            Eigen::VectorXd x = xv.head(nx);
            Eigen::VectorXd v = xv.tail(ny);
            Eigen::MatrixXd J;
            Eigen::VectorXd ya = predictFeature(x, J, system, idxLandmark) + v;
            Ja.resize(ny, nx + ny);
            Ja << J, Eigen::MatrixXd::Identity(ny, ny);
            return ya;
        };

        auto pv  = gaussian::GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
        auto pxv = system.density * pv;  // p(x, v) = p(x)*p(v)
        return pxv.affineTransform(func);
    }

    // Image feature locations for a bundle of landmarks
    Eigen::VectorXd MeasurementPointBundle::predictFeatureBundle(const Eigen::VectorXd& x,
                                                                 Eigen::MatrixXd& J,
                                                                 const SystemSLAM& system,
                                                                 const std::vector<std::size_t>& idxLandmarks) const {
        const std::size_t& nL = idxLandmarks.size();
        const std::size_t& nx = system.density.dim();
        assert(x.size() == nx);

        Eigen::VectorXd h(2 * nL);
        J.resize(2 * nL, nx);
        for (std::size_t i = 0; i < nL; ++i) {
            Eigen::MatrixXd Jfeature;
            Eigen::Vector2d rQOi = predictFeature(x, Jfeature, system, idxLandmarks[i]);
            // Set pair of elements of h
            h.segment<2>(2 * i) = rQOi;
            // Set pair of rows of J
            J.block<2, Eigen::Dynamic>(2 * i, 0, 2, nx) = Jfeature;
        }
        return h;
    }

    // Density of image features for a set of landmarks
    gaussian::GaussianInfo<double> MeasurementPointBundle::predictFeatureBundleDensity(
        const SystemSLAM& system,
        const std::vector<std::size_t>& idxLandmarks) const {
        const std::size_t& nx = system.density.dim();
        const std::size_t ny  = 2 * idxLandmarks.size();

        //   y   =   h(x) + v
        // \___/   \__________/
        //   ya  =   ha(x, v)
        //
        // Helper function to evaluate ha(x, v) and its Jacobian Ja = [dha/dx, dha/dv]
        const auto func = [&](const Eigen::VectorXd& xv, Eigen::MatrixXd& Ja) {
            assert(xv.size() == nx + ny);
            Eigen::VectorXd x = xv.head(nx);
            Eigen::VectorXd v = xv.tail(ny);
            Eigen::MatrixXd J;
            Eigen::VectorXd ya = predictFeatureBundle(x, J, system, idxLandmarks) + v;
            Ja.resize(ny, nx + ny);
            Ja << J, Eigen::MatrixXd::Identity(ny, ny);
            return ya;
        };

        auto pv  = gaussian::GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
        auto pxv = system.density * pv;  // p(x, v) = p(x)*p(v)
        return pxv.affineTransform(func);
    }

    const std::vector<int>& MeasurementPointBundle::associate(const SystemSLAM& system,
                                                              const std::vector<std::size_t>& idxLandmarks) {
        // guard to prevent snn being called with an empty idxLandmarks
        if (idxLandmarks.empty()) {
            idxFeatures_.clear();
            return idxFeatures_;
        }
        gaussian::GaussianInfo<double> featureBundleDensity = predictFeatureBundleDensity(system, idxLandmarks);
        snn(system, featureBundleDensity, idxLandmarks, Y_, camera_, idxFeatures_);
        return idxFeatures_;
    }

}  // namespace utility::slam::measurement
