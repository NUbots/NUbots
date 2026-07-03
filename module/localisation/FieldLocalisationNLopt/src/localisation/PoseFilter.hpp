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
#ifndef MODULE_LOCALISATION_POSEFILTER_HPP
#define MODULE_LOCALISATION_POSEFILTER_HPP

#include <Eigen/Core>
#include <Eigen/LU>

#include "utility/math/angle.hpp"

namespace module::localisation {

    /// @brief Minimal 3-state (x, y, theta) Kalman filter over the Hfw pose estimate.
    ///
    /// The state (x, y, theta) parameterises Hfw, which is constant under perfect odometry, so the process
    /// model is identity transition: `predict` grows the covariance in proportion to observed odometry motion
    /// rather than applying any motion model. The optimisation result from `run_field_line_optimisation` is
    /// treated as a measurement of the same state, with covariance derived from the cost function's curvature
    /// (see `covariance_from_hessian` in filter.cpp). The innovation in `update` is angle-wrapped so heading
    /// estimation doesn't fight the +-pi seam.
    struct PoseFilter {
        /// @brief Current state estimate (x, y, theta)
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        /// @brief Current state covariance
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();

        /// @brief Hard reset to a pose with independent per-axis 1-sigma uncertainty.
        /// @param pose The new mean (x, y, theta)
        /// @param sigma The new 1-sigma uncertainty per axis (x, y, theta)
        void reset(const Eigen::Vector3d& pose, const Eigen::Vector3d& sigma) {
            mean = pose;
            cov  = sigma.cwiseProduct(sigma).asDiagonal();
        }

        /// @brief Grow the covariance by odometry-scaled process noise.
        /// @param motion A scalar summarising how much the robot moved since the last predict, i.e.
        /// ||delta translation|| + |delta yaw|, from consecutive Sensors::Hrw
        /// @param process_noise_rate The per-unit-motion variance rate for each state (x^2, y^2, theta^2 per
        /// metre-or-radian of motion)
        void predict(double motion, const Eigen::Vector3d& process_noise_rate) {
            cov.diagonal() += motion * process_noise_rate;
        }

        /// @brief Fuse a measurement z (x, y, theta) with covariance R into the current estimate.
        /// @param z The measurement (x, y, theta)
        /// @param R The measurement covariance
        void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) {
            Eigen::Vector3d innovation = z - mean;
            innovation.z()             = utility::math::angle::signedDifference(z.z(), mean.z());

            Eigen::Matrix3d S = cov + R;
            Eigen::Matrix3d K = cov * S.inverse();

            mean += K * innovation;
            mean.z() = utility::math::angle::normalise_angle(mean.z());

            cov = (Eigen::Matrix3d::Identity() - K) * cov;
        }
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_POSEFILTER_HPP
