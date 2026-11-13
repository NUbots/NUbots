/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "MeasurementFieldLandmarks.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

namespace module::localisation::measurement {

    MeasurementFieldLandmarks::MeasurementFieldLandmarks(double time,
                                                         const VisionSample& sample,
                                                         const Pose<double>& Tbc,
                                                         const FieldMap& map,
                                                         const SystemLocalisation& system,
                                                         const Options& options)
        : Measurement(time), map_(map), Tbc_(Tbc), options_(options) {
        // The exact Hessian gives the Laplace-approximation posterior sqrt information
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;

        // Gather usable detections
        for (std::size_t i = 0; i < sample.detections.size(); ++i) {
            const Detection& det = sample.detections[i];
            if (det.confidence < options_.min_confidence) {
                continue;
            }
            Eigen::Vector3d ray;
            LandmarkType type;
            if (detection_ray(det, ray, type)) {
                // Confidence sets the inlier weight of this detection's likelihood (see Options)
                const double w =
                    std::clamp(options_.inlier_probability * det.confidence / options_.confidence_reference,
                               1e-3,
                               options_.max_inlier_probability);
                candidates_.push_back({ray, type, i, w});
            }
        }

        assoc_keys_ = associate(system.density.mean(), system.density.cov());
    }

    MeasurementFieldLandmarks::MeasurementFieldLandmarks(double time,
                                                         const VisionSample& sample,
                                                         const Pose<double>& Tbc,
                                                         const FieldMap& map,
                                                         const SystemLocalisation& system)
        : MeasurementFieldLandmarks(time, sample, Tbc, map, system, Options{}) {}

    bool MeasurementFieldLandmarks::detection_ray(const Detection& det, Eigen::Vector3d& ray, LandmarkType& type) {
        // Class names as the K1 YOLO model emits them
        if (det.name == "LCross") {
            type = LandmarkType::L_INTERSECTION;
            ray  = det.corners.rowwise().sum();  // Bounding box centre
        }
        else if (det.name == "TCross") {
            type = LandmarkType::T_INTERSECTION;
            ray  = det.corners.rowwise().sum();
        }
        else if (det.name == "XCross") {
            type = LandmarkType::X_INTERSECTION;
            ray  = det.corners.rowwise().sum();
        }
        else if (det.name == "Goalpost") {
            type = LandmarkType::GOAL_POST;
            ray  = det.corners.col(2) + det.corners.col(3);  // Bottom-centre: BR + BL
        }
        else {
            return false;  // Not a mapped landmark class
        }

        if (!ray.allFinite() || ray.norm() < 1e-12) {
            return false;
        }
        ray.normalize();
        return true;
    }

    std::vector<std::pair<std::size_t, std::size_t>> MeasurementFieldLandmarks::associate(const Eigen::VectorXd& x,
                                                                                          const Eigen::MatrixXd& P) {
        std::vector<std::pair<std::size_t, std::size_t>> keys;
        inlier_weight_.clear();
        if (candidates_.empty()) {
            u_meas_.resize(3, 0);
            rLFf_.resize(3, 0);
            return keys;
        }

        // Predicted ray for every mapped landmark at the given state
        Pose<double> Tbias(SystemLocalisation::camera_bias_rotation<double>(x), Eigen::Vector3d::Zero());
        Pose<double> Tfc           = SystemLocalisation::field_pose<double>(x) * Tbc_ * Tbias;
        const Eigen::Matrix3d Rfc  = Tfc.rotation_matrix;
        const Eigen::Vector3d rCFf = Tfc.translation_vector;

        // Pose uncertainty inflating each predicted bearing, compared in {f}
        const Eigen::Matrix3d Ppos = P.topLeftCorner<3, 3>();
        const double yaw_var       = SystemLocalisation::yaw_variance(x, P);
        const double sigma2        = options_.sigma_angular * options_.sigma_angular;
        // Pre-gate widened by the yaw uncertainty (see Options::gate_yaw_scale)
        const double gate_angle =
            std::min(std::max(options_.gate_angle, options_.gate_yaw_scale * std::sqrt(std::max(yaw_var, 0.0))),
                     options_.gate_angle_max);
        const double cos_gate = std::cos(gate_angle);

        // Score every (detection, landmark) pair of matching type inside the pre-gate by its
        // surprisal relative to the clutter crossover, then assign the best pairs first, each
        // detection and landmark at most once
        struct CandidatePair {
            double score;
            std::size_t det;
            LandmarkType type;
            std::size_t lm;
        };
        std::vector<CandidatePair> pairs;
        for (std::size_t i = 0; i < candidates_.size(); ++i) {
            const Eigen::Vector3d u_meas_f = Rfc * candidates_[i].ray;
            // Crossover surprisal: w N(e; S) > (1 - w)/(4 pi)
            const double w         = candidates_[i].inlier_weight;
            const double crossover = std::log(w) - std::log(1.0 - w) + std::log(4.0 * M_PI);

            const std::vector<Eigen::Vector3d>& lms = map_.landmarks(candidates_[i].type);
            for (std::size_t j = 0; j < lms.size(); ++j) {
                const Eigen::Vector3d rel = lms[j] - rCFf;
                const double range        = rel.norm();
                if (range < 1e-9) {
                    continue;
                }
                const Eigen::Vector3d u_pred_f = rel / range;
                if (u_meas_f.dot(u_pred_f) < cos_gate) {
                    continue;  // Geometric pre-gate
                }

                // Innovation covariance in the tangent plane: bearing noise, camera position
                // uncertainty across the range, and yaw uncertainty
                const Eigen::Matrix<double, 3, 2> T = tangent_basis(u_pred_f);
                const Eigen::Vector2d a             = T.transpose() * Eigen::Vector3d::UnitZ().cross(u_pred_f);
                const Eigen::Matrix2d S             = Eigen::Matrix2d::Identity() * sigma2
                                          + T.transpose() * Ppos * T / (range * range) + yaw_var * a * a.transpose();
                const Eigen::Vector2d e = T.transpose() * (u_meas_f - u_pred_f);
                const double surprisal =
                    0.5 * e.dot(S.inverse() * e) + 0.5 * std::log(S.determinant()) + std::log(2.0 * M_PI);
                const double score = surprisal - crossover;
                if (score < 0.0) {
                    pairs.push_back({score, i, candidates_[i].type, j});
                }
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const CandidatePair& a, const CandidatePair& b) {
            return a.score < b.score;
        });

        std::vector<bool> det_used(candidates_.size(), false);
        // Landmark usage is tracked by a flat type-major key
        auto lm_key = [](LandmarkType type, std::size_t j) { return static_cast<std::size_t>(type) * 1000 + j; };
        std::vector<std::size_t> used_landmarks;
        std::vector<const CandidatePair*> chosen;
        for (const CandidatePair& p : pairs) {
            if (det_used[p.det]) {
                continue;
            }
            std::size_t key = lm_key(p.type, p.lm);
            if (std::find(used_landmarks.begin(), used_landmarks.end(), key) != used_landmarks.end()) {
                continue;
            }
            det_used[p.det] = true;
            used_landmarks.push_back(key);
            chosen.push_back(&p);
        }

        u_meas_.resize(3, static_cast<Eigen::Index>(chosen.size()));
        rLFf_.resize(3, static_cast<Eigen::Index>(chosen.size()));
        inlier_weight_.reserve(chosen.size());
        for (std::size_t k = 0; k < chosen.size(); ++k) {
            const CandidatePair& p                    = *chosen[k];
            u_meas_.col(static_cast<Eigen::Index>(k)) = candidates_[p.det].ray;
            rLFf_.col(static_cast<Eigen::Index>(k))   = map_.landmarks(p.type)[p.lm];
            inlier_weight_.push_back(candidates_[p.det].inlier_weight);
            keys.emplace_back(p.det, lm_key(p.type, p.lm));
        }
        std::sort(keys.begin(), keys.end());
        return keys;
    }

    Eigen::VectorXd MeasurementFieldLandmarks::simulate(const Eigen::VectorXd& x,
                                                        const SystemEstimator& /*system*/) const {
        // Stack predicted rays
        Eigen::Matrix<double, 3, Eigen::Dynamic> u_pred = predict_rays<double>(x);
        return Eigen::Map<Eigen::VectorXd>(u_pred.data(), u_pred.size());
    }

    double MeasurementFieldLandmarks::log_likelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& /*system*/) const {
        return log_likelihood_impl<double>(x);
    }

    double MeasurementFieldLandmarks::log_likelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& /*system*/,
                                                     Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template log_likelihood_impl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementFieldLandmarks::log_likelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& /*system*/,
                                                     Eigen::VectorXd& g,
                                                     Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template log_likelihood_impl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

    void MeasurementFieldLandmarks::update(SystemBase& system_) {
        SystemEstimator& system          = dynamic_cast<SystemEstimator&>(system_);
        const GaussianInfo<double> prior = system.density;

        // Optimise with the current association, re-associate at the posterior mean, and if
        // the association set changed restore the prior and re-run
        for (int iteration = 0; iteration < max_association_iterations_; ++iteration) {
            if (u_meas_.cols() == 0) {
                system.density = prior;  // Nothing associated
                return;
            }

            Measurement::update(system);

            if (iteration == max_association_iterations_ - 1) {
                break;  // Iteration budget exhausted
            }

            std::vector<std::pair<std::size_t, std::size_t>> new_keys =
                associate(system.density.mean(), system.density.cov());
            if (new_keys == assoc_keys_) {
                break;  // Association converged
            }
            assoc_keys_    = new_keys;
            system.density = prior;
        }
    }
}  // namespace module::localisation::measurement
