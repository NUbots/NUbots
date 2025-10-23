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

#include <Eigen/Core>
#include <algorithm>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "../association_util.hpp"
#include "../camera/Camera.hpp"
#include "../gaussian/GaussianInfo.hpp"
#include "../rotation.hpp"
#include "../system/SystemBase.hpp"
#include "../system/SystemEstimator.hpp"
#include "../system/SystemSLAM.hpp"
#include "../system/SystemSLAMPointLandmarks.hpp"

namespace utility::slam::measurement {

    MeasurementPointBundle::MeasurementPointBundle(double time,
                                                   const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                                                   const camera::Camera& camera)
        : MeasurementSLAM(time, camera)
        , Y_(Y)
        , sigma_(4.0)  // Measurement noise (pixels)
    {
        updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
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
        const system::SystemSLAM& systemSLAM = dynamic_cast<const system::SystemSLAM&>(system);
        return logLikelihoodTemplated<double>(x, systemSLAM);
    }

    double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd& x,
                                                 const SystemEstimator& system,
                                                 Eigen::VectorXd& g) const {
        const system::SystemSLAM& systemSLAM = dynamic_cast<const system::SystemSLAM&>(system);

        double logLik = logLikelihood(x, system);
        g             = Eigen::VectorXd::Zero(x.size());

        Eigen::MatrixXd S                               = sigma_ * Eigen::MatrixXd::Identity(2, 2);
        gaussian::GaussianInfo<double> measurementModel = gaussian::GaussianInfo<double>::fromSqrtMoment(S);

        for (std::size_t j = 0; j < idxFeatures_.size(); ++j) {
            if (idxFeatures_[j] >= 0) {
                int detectionIdx   = idxFeatures_[j];
                size_t landmarkIdx = visibleLandmarks_[j];

                Eigen::MatrixXd J_h;
                Eigen::Vector2d h_pred   = predictFeature(x, J_h, systemSLAM, landmarkIdx);
                Eigen::Vector2d y_i      = Y_.col(detectionIdx);
                Eigen::Vector2d residual = y_i - h_pred;

                Eigen::VectorXd g_residual;
                measurementModel.log(residual, g_residual);
                g += -J_h.transpose() * g_residual;
            }
        }

        return logLik;
    }

    double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd& x,
                                                 const SystemEstimator& system,
                                                 Eigen::VectorXd& g,
                                                 Eigen::MatrixXd& H) const {
        const system::SystemSLAM& systemSLAM = dynamic_cast<const system::SystemSLAM&>(system);

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
        system::SystemSLAM& systemSLAM = dynamic_cast<system::SystemSLAM&>(system);
        system::SystemSLAMPointLandmarks& systemPointLandmarks =
            dynamic_cast<system::SystemSLAMPointLandmarks&>(system);

        // Ensure consecutiveFailures_ is sized correctly
        while (systemPointLandmarks.consecutiveFailures_.size() < systemSLAM.numberLandmarks()) {
            systemPointLandmarks.consecutiveFailures_.push_back(0);
        }

        // Get camera state for visibility checks
        Eigen::VectorXd x       = systemSLAM.density.mean();
        Eigen::Vector3d rCNn    = systemSLAM.cameraPositionDensity(camera_).mean();
        Eigen::Vector3d Thetanc = systemSLAM.cameraOrientationEulerDensity(camera_).mean();
        Eigen::Matrix3d Rnc     = rpy2rot(Thetanc);

        // Identify visible landmarks (FOV check)
        visibleLandmarks_.clear();
        for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
            std::size_t idx      = systemSLAM.landmarkPositionIndex(i);
            Eigen::Vector3d rPNn = x.segment<3>(idx);
            Eigen::Vector3d rPCc = Rnc.transpose() * (rPNn - rCNn);

            cv::Vec3d rPCc_cv(rPCc(0), rPCc(1), rPCc(2));
            if (camera_.isVectorWithinFOV(rPCc_cv)) {
                visibleLandmarks_.push_back(i);
            }
        }

        // Associate visible landmarks with detected features
        idxFeatures_ = associate(systemSLAM, visibleLandmarks_);

        // Update consecutive failures tracking
        for (std::size_t j = 0; j < visibleLandmarks_.size(); ++j) {
            std::size_t landmarkIdx = visibleLandmarks_[j];
            if (idxFeatures_[j] >= 0) {
                if (systemPointLandmarks.consecutiveFailures_[landmarkIdx] > 0) {
                    std::cout << "  Landmark " << landmarkIdx << " SUCCESSFULLY associated after "
                              << systemPointLandmarks.consecutiveFailures_[landmarkIdx] << " failures (reset to 0)"
                              << std::endl;
                }
                systemPointLandmarks.consecutiveFailures_[landmarkIdx] = 0;
            }
            else {
                int oldValue = systemPointLandmarks.consecutiveFailures_[landmarkIdx];
                systemPointLandmarks.consecutiveFailures_[landmarkIdx]++;
                int newValue = systemPointLandmarks.consecutiveFailures_[landmarkIdx];
                std::cout << "  Landmark " << landmarkIdx << " failed association (was=" << oldValue
                          << ", now=" << newValue << ")" << std::endl;
            }
        }

        // Collect landmarks to delete
        std::vector<std::size_t> landmarksToDelete;
        int maxTotalLandmarks      = 60;
        int maxConsecutiveFailures = 10;

        // Criterion 1: Delete landmarks with too many consecutive failures
        // std::cout << "Checking for deletion (threshold=" << maxConsecutiveFailures << "):" << std::endl;
        for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
            std::cout << "  Landmark " << i << ": failures=" << systemPointLandmarks.consecutiveFailures_[i];
            if (systemPointLandmarks.consecutiveFailures_[i] >= maxConsecutiveFailures) {
                std::cout << " -> MARKED FOR DELETION";
                landmarksToDelete.push_back(i);
            }
            std::cout << std::endl;
        }

        // Criterion 2: If at capacity, delete worst performers to make room
        int currentTotal = systemSLAM.numberLandmarks();
        if (currentTotal >= maxTotalLandmarks) {
            int spotsNeeded  = 1;  // Room for new landmarks
            int needToDelete = (currentTotal + spotsNeeded) - maxTotalLandmarks;

            if (needToDelete > 0) {
                std::vector<std::pair<int, size_t>> failureRanking;
                for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
                    if (std::find(landmarksToDelete.begin(), landmarksToDelete.end(), i) == landmarksToDelete.end()) {
                        failureRanking.push_back({systemPointLandmarks.consecutiveFailures_[i], i});
                    }
                }

                std::sort(failureRanking.begin(), failureRanking.end(), [](const auto& a, const auto& b) {
                    return a.first > b.first;
                });

                for (int i = 0; i < std::min(needToDelete, (int) failureRanking.size()); ++i) {
                    landmarksToDelete.push_back(failureRanking[i].second);
                }
            }
        }

        // Sort and remove duplicates
        std::sort(landmarksToDelete.begin(), landmarksToDelete.end());
        landmarksToDelete.erase(std::unique(landmarksToDelete.begin(), landmarksToDelete.end()),
                                landmarksToDelete.end());

        // Delete in reverse order to maintain correct indices
        if (!landmarksToDelete.empty()) {
            std::cout << "Deleting " << landmarksToDelete.size() << " landmarks: ";
            for (auto it = landmarksToDelete.rbegin(); it != landmarksToDelete.rend(); ++it) {
                std::size_t landmarkIdx = *it;
                std::cout << landmarkIdx << "(f=" << systemPointLandmarks.consecutiveFailures_[landmarkIdx] << ") ";

                // Marginalize out landmark
                std::size_t stateIdx = systemSLAM.landmarkPositionIndex(landmarkIdx);
                std::vector<int> indicesToKeep;
                indicesToKeep.reserve(systemSLAM.density.dim() - 3);

                for (std::size_t i = 0; i < systemSLAM.density.dim(); ++i) {
                    if (i < stateIdx || i >= stateIdx + 3) {
                        indicesToKeep.push_back(static_cast<int>(i));
                    }
                }

                systemSLAM.density = systemSLAM.density.marginal(indicesToKeep);
                systemPointLandmarks.consecutiveFailures_.erase(systemPointLandmarks.consecutiveFailures_.begin()
                                                                + landmarkIdx);

                // Update visible landmarks list
                auto it_vis = std::find(visibleLandmarks_.begin(), visibleLandmarks_.end(), landmarkIdx);
                if (it_vis != visibleLandmarks_.end()) {
                    size_t position = std::distance(visibleLandmarks_.begin(), it_vis);
                    visibleLandmarks_.erase(it_vis);
                    idxFeatures_.erase(idxFeatures_.begin() + position);
                }

                // Decrement indices > landmarkIdx
                for (size_t& idx : visibleLandmarks_) {
                    if (idx > landmarkIdx)
                        idx--;
                }
            }
            std::cout << std::endl;
        }

        // Identify surplus features that do not correspond to landmarks
        std::vector<bool> detectionUsed(Y_.cols(), false);
        for (int idx : idxFeatures_) {
            if (idx >= 0) {
                detectionUsed[idx] = true;
            }
        }

        // Initialize new landmarks from best surplus features
        int maxVisibleLandmarksPerFrame = 20;
        currentTotal                    = systemSLAM.numberLandmarks();
        int currentVisible              = visibleLandmarks_.size();

        int spotsAvailableInFrame = maxVisibleLandmarksPerFrame - currentVisible;
        int spotsAvailableTotal   = maxTotalLandmarks - currentTotal;
        int landmarksToInitialize = std::min(spotsAvailableInFrame, spotsAvailableTotal);

        if (landmarksToInitialize <= 0) {
            Measurement::update(system);
            return;
        }

        int landmarksInitializedThisFrame = 0;

        // Collect candidate detections
        std::vector<std::tuple<size_t, Eigen::Vector2d, double>> candidates;
        double borderMargin  = 0.0;  // pixels
        double minSeparation = 0.0;  // pixels

        for (std::size_t detectionIdx = 0; detectionIdx < Y_.cols(); ++detectionIdx) {
            if (detectionUsed[detectionIdx])
                continue;

            Eigen::Vector2d pixel = Y_.col(detectionIdx);

            // Check if too close to image border
            bool tooCloseToEdge = (pixel(0) < borderMargin || pixel(0) > camera_.imageSize.width - borderMargin
                                   || pixel(1) < borderMargin || pixel(1) > camera_.imageSize.height - borderMargin);
            if (tooCloseToEdge)
                continue;

            // Find minimum distance to existing landmarks
            double minDistToExisting = std::numeric_limits<double>::max();
            for (size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
                std::size_t idx               = systemSLAM.landmarkPositionIndex(i);
                Eigen::Vector3d rPNn_existing = x.segment<3>(idx);
                Eigen::Vector3d rPCc_existing = Rnc.transpose() * (rPNn_existing - rCNn);

                if (rPCc_existing(2) > 0) {
                    Eigen::Vector2d projected_pixel = camera_.vectorToPixel(rPCc_existing);
                    double distance                 = (projected_pixel - pixel).norm();
                    minDistToExisting               = std::min(minDistToExisting, distance);
                }
            }

            if (minDistToExisting < minSeparation)
                continue;

            // Compute minimum distance to all other detections
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

        // Sort candidates by minimum distance (descending - furthest first)
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return std::get<2>(a) > std::get<2>(b);
        });

        // Initialize landmarks from sorted candidates
        for (const auto& candidate : candidates) {
            if (landmarksInitializedThisFrame >= landmarksToInitialize) {
                break;
            }

            size_t candidateIdx            = std::get<0>(candidate);
            Eigen::Vector2d candidatePixel = std::get<1>(candidate);
            double candidateDist           = std::get<2>(candidate);

            // Check distance to already-initialized landmarks this frame
            bool tooCloseToNewLandmark = false;
            for (int alreadyInit = 0; alreadyInit < landmarksInitializedThisFrame; ++alreadyInit) {
                size_t newLandmarkIdx    = systemSLAM.numberLandmarks() - 1 - alreadyInit;
                std::size_t idx          = systemSLAM.landmarkPositionIndex(newLandmarkIdx);
                Eigen::Vector3d rPNn_new = systemSLAM.density.mean().segment<3>(idx);
                Eigen::Vector3d rPCc_new = Rnc.transpose() * (rPNn_new - rCNn);

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

            // Initialize landmark at arbitrary depth
            double arbitrary_depth = 2.0;  // meters

            cv::Vec2d pixel_cv(candidatePixel(0), candidatePixel(1));
            cv::Vec3d rPCc_cv = camera_.pixelToVector(pixel_cv);
            Eigen::Vector3d rPCc_unit(rPCc_cv[0], rPCc_cv[1], rPCc_cv[2]);
            Eigen::Vector3d rPCc = rPCc_unit.normalized() * arbitrary_depth;

            // Transform to world frame
            Eigen::Vector3d rPNn = Rnc * rPCc + rCNn;

            // Create new landmark with prior
            Eigen::VectorXd mu_new = rPNn;
            double epsilon         = 5.0;  // Low confidence in initial position
            Eigen::MatrixXd Xi_new = epsilon * Eigen::MatrixXd::Identity(3, 3);
            Eigen::VectorXd nu_new = Xi_new * mu_new;

            gaussian::GaussianInfo<double> newLandmarkDensity =
                gaussian::GaussianInfo<double>::fromSqrtInfo(nu_new, Xi_new);
            systemSLAM.density *= newLandmarkDensity;
            landmarksInitializedThisFrame++;

            size_t newLandmarkIdx = systemSLAM.numberLandmarks() - 1;
            std::cout << "  Initialized point landmark " << newLandmarkIdx << " (dist=" << candidateDist
                      << ", depth=" << arbitrary_depth << "m)" << std::endl;

            // Add to visible landmarks and associate with detection
            visibleLandmarks_.push_back(newLandmarkIdx);
            idxFeatures_.push_back(static_cast<int>(candidateIdx));
            systemPointLandmarks.consecutiveFailures_.push_back(0);
        }

        // Perform measurement update
        Measurement::update(system);
    }

    Eigen::Vector2d MeasurementPointBundle::predictFeature(const Eigen::VectorXd& x,
                                                           Eigen::MatrixXd& J,
                                                           const system::SystemSLAM& system,
                                                           std::size_t idxLandmark) const {
        // Get camera pose from state
        Pose<double> Tnc;
        Tnc.translationVector = system.cameraPosition(camera_, x);
        Tnc.rotationMatrix    = system.cameraOrientation(camera_, x);

        // Get landmark position from state
        std::size_t idx      = system.landmarkPositionIndex(idxLandmark);
        Eigen::Vector3d rPNn = x.segment<3>(idx);

        // Transform to camera coordinates
        Eigen::Vector3d rPCc = Tnc.rotationMatrix.transpose() * (rPNn - Tnc.translationVector);

        // Get pixel coordinates with Jacobian
        Eigen::Matrix23d J_camera;
        Eigen::Vector2d rQOi = camera_.vectorToPixel(rPCc, J_camera);

        // Compute full Jacobian using chain rule
        J.resize(2, x.size());
        J.setZero();

        Eigen::Vector3d rBNn    = x.segment<3>(6);
        Eigen::Vector3d Thetanb = x.segment<3>(9);

        Pose<double> Tnb;
        Tnb.rotationMatrix    = rpy2rot(Thetanb);
        Tnb.translationVector = rBNn;

        Pose<double> Tbc    = camera_.Tbc;
        Eigen::Matrix3d Rnb = Tnb.rotationMatrix;
        Eigen::Matrix3d Rbc = Tbc.rotationMatrix;

        // Equation (27a): ∂h/∂rPNn
        Eigen::Matrix<double, 2, 3> dhj_drPNn = J_camera * Rbc.transpose() * Rnb.transpose();
        J.block<2, 3>(0, idx)                 = dhj_drPNn;

        // Equation (27b): ∂h/∂rBNn
        Eigen::Matrix<double, 2, 3> dhj_drBNn = -J_camera * Rbc.transpose() * Rnb.transpose();
        J.block<2, 3>(0, 6)                   = dhj_drBNn;

        Eigen::Vector3d rPNn_minus_rBNn = rPNn - rBNn;

        // Roll
        Eigen::Matrix3d dRx_dphi;
        rotx(Thetanb(0), dRx_dphi);
        Eigen::Matrix3d dRnb_dphi = rotz(Thetanb(2)) * roty(Thetanb(1)) * dRx_dphi;

        // Pitch
        Eigen::Matrix3d dRy_dtheta;
        roty(Thetanb(1), dRy_dtheta);
        Eigen::Matrix3d dRnb_dtheta = rotz(Thetanb(2)) * dRy_dtheta * rotx(Thetanb(0));

        // Yaw
        Eigen::Matrix3d dRz_dpsi;
        rotz(Thetanb(2), dRz_dpsi);
        Eigen::Matrix3d dRnb_dpsi = dRz_dpsi * roty(Thetanb(1)) * rotx(Thetanb(0));

        // Apply equation (27c)
        J.col(9)  = J_camera * Rbc.transpose() * dRnb_dphi.transpose() * rPNn_minus_rBNn;
        J.col(10) = J_camera * Rbc.transpose() * dRnb_dtheta.transpose() * rPNn_minus_rBNn;
        J.col(11) = J_camera * Rbc.transpose() * dRnb_dpsi.transpose() * rPNn_minus_rBNn;

        return rQOi;
    }

    gaussian::GaussianInfo<double> MeasurementPointBundle::predictFeatureDensity(const system::SystemSLAM& system,
                                                                                 std::size_t idxLandmark) const {
        const std::size_t& nx = system.density.dim();
        const std::size_t ny  = 2;

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
        auto pxv = system.density * pv;
        return pxv.affineTransform(func);
    }

    Eigen::VectorXd MeasurementPointBundle::predictFeatureBundle(const Eigen::VectorXd& x,
                                                                 Eigen::MatrixXd& J,
                                                                 const system::SystemSLAM& system,
                                                                 const std::vector<std::size_t>& idxLandmarks) const {
        const std::size_t& nL = idxLandmarks.size();
        const std::size_t& nx = system.density.dim();
        assert(x.size() == nx);

        Eigen::VectorXd h(2 * nL);
        J.resize(2 * nL, nx);
        for (std::size_t i = 0; i < nL; ++i) {
            Eigen::MatrixXd Jfeature;
            Eigen::Vector2d rQOi                        = predictFeature(x, Jfeature, system, idxLandmarks[i]);
            h.segment<2>(2 * i)                         = rQOi;
            J.block<2, Eigen::Dynamic>(2 * i, 0, 2, nx) = Jfeature;
        }
        return h;
    }

    gaussian::GaussianInfo<double> MeasurementPointBundle::predictFeatureBundleDensity(
        const system::SystemSLAM& system,
        const std::vector<std::size_t>& idxLandmarks) const {
        const std::size_t& nx = system.density.dim();
        const std::size_t ny  = 2 * idxLandmarks.size();

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
        auto pxv = system.density * pv;
        return pxv.affineTransform(func);
    }

    const std::vector<int>& MeasurementPointBundle::associate(const system::SystemSLAM& system,
                                                              const std::vector<std::size_t>& idxLandmarks) {
        if (idxLandmarks.empty()) {
            idxFeatures_.clear();
            return idxFeatures_;
        }
        gaussian::GaussianInfo<double> featureBundleDensity = predictFeatureBundleDensity(system, idxLandmarks);
        snn(system, featureBundleDensity, idxLandmarks, Y_, camera_, idxFeatures_);
        return idxFeatures_;
    }

}  // namespace utility::slam::measurement
