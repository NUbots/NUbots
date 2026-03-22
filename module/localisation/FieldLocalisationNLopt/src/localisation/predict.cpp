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
#include "utility/math/euler.hpp"

namespace module::localisation {

    using message::behaviour::state::Stability;
    using message::input::Sensors;

    using utility::math::angle::normalise_angle;
    using utility::math::euler::mat_to_rpy_intrinsic;

    void FieldLocalisationNLopt::predict(const Sensors& sensors, const Stability& stability) {
        // Don't predict before we have an initial state
        if (startup || first_measurement) {
            return;
        }

        // Odometry is unreliable while falling — invalidate reference so no stale delta accumulates on recovery
        if (stability <= Stability::FALLING) {
            has_prev_Hrw = false;
            return;
        }

        Eigen::Isometry3d Hrw_current(sensors.Hrw);

        // On the first valid Sensors after startup or recovery, just store the reference and skip
        if (!has_prev_Hrw) {
            Hrw_prev     = Hrw_current;
            has_prev_Hrw = true;
            return;
        }

        // Incremental motion expressed in robot body frame
        const Eigen::Isometry3d dH = Hrw_prev.inverse() * Hrw_current;
        const double dx_r          = dH.translation().x();
        const double dy_r          = dH.translation().y();
        const double dtheta        = mat_to_rpy_intrinsic(dH.rotation()).z();

        // Capture the current heading BEFORE updating state — Jacobian is linearised here
        const double theta = filtered_state.z();
        const double ct    = std::cos(theta);
        const double st    = std::sin(theta);

        // Propagate state with nonlinear planar motion model
        filtered_state.x() += dx_r * ct - dy_r * st;
        filtered_state.y() += dx_r * st + dy_r * ct;
        filtered_state.z()  = normalise_angle(filtered_state.z() + dtheta);

        // Motion Jacobian F = ∂f/∂[x,y,θ] evaluated at the old heading
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2)           = -dx_r * st - dy_r * ct;
        F(1, 2)           = dx_r * ct - dy_r * st;

        // Covariance prediction: P = F·P·Fᵀ + Q
        const Eigen::Matrix3d Q = cfg.process_noise.asDiagonal();
        P                       = F * P * F.transpose() + Q;

        Hrw_prev = Hrw_current;
    }

}  // namespace module::localisation
