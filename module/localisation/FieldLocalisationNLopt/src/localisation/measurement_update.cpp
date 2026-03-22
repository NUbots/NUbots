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

#include "FieldLocalisationNLopt.hpp"

#include "utility/math/angle.hpp"

namespace module::localisation {

    using utility::math::angle::normalise_angle;

    bool FieldLocalisationNLopt::measurement_update(const Eigen::Vector3d& nlopt_result, double cost) {
        // Reject if cost exceeds threshold — filtered_state remains at the prediction
        if (cost >= cfg.cost_threshold) {
            return false;
        }

        if (first_measurement) {
            // First accepted update: seed EKF state directly without blending
            filtered_state    = nlopt_result;
            P                 = cfg.initial_covariance * Eigen::Matrix3d::Identity();
            first_measurement = false;
            has_prev_Hrw      = false;
            return true;
        }

        // EKF measurement update — NLopt result is a direct observation of state (H = I₃)
        // Measurement noise scales with cost: lower cost → smaller R → more trust in vision
        const Eigen::Matrix3d R_meas =
            (cfg.measurement_noise_scale * std::max(cost, 1e-6)) * Eigen::Matrix3d::Identity();

        // Innovation with angle wrapping on the heading component
        Eigen::Vector3d inn = nlopt_result - filtered_state;
        inn.z()             = normalise_angle(inn.z());

        // Innovation covariance S = P + R  (simplifies since H = I₃)
        const Eigen::Matrix3d S = P + R_meas;

        // Kalman gain
        const Eigen::Matrix3d K = P * S.inverse();

        // State update
        filtered_state += K * inn;
        filtered_state.z() = normalise_angle(filtered_state.z());

        // Covariance update — Joseph form for numerical stability: P = (I-K)·P·(I-K)ᵀ + K·R·Kᵀ
        const Eigen::Matrix3d IK = Eigen::Matrix3d::Identity() - K;
        P                        = IK * P * IK.transpose() + K * R_meas * K.transpose();

        return true;
    }

}  // namespace module::localisation
