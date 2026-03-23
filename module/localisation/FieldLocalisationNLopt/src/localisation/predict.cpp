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

#include "utility/math/GaussianInfo.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"

namespace module::localisation {

    using utility::math::GaussianInfo;
    using utility::math::angle::normalise_angle;
    using utility::math::euler::mat_to_rpy_intrinsic;

    using message::behaviour::state::Stability;
    using message::input::Sensors;

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
        const Eigen::Vector3d mu = density.mean();
        const double theta       = mu.z();
        const double ct          = std::cos(theta);
        const double st          = std::sin(theta);

        // Motion Jacobian F = ∂f/∂[x,y,θ] evaluated at old heading
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2)           = -dx_r * st - dy_r * ct;
        F(1, 2)           = dx_r * ct - dy_r * st;

        // Process noise Q = diag(q_x, q_y, q_theta)
        const Eigen::Matrix3d Q = cfg.process_noise.asDiagonal();

        // SRIF predict via affineTransform on augmented state [x; w]:
        //   Phi([x; w]) = f(x) + w,   Jacobian J = [F, I₃]
        // Nonlinear motion model:
        //   x_new = x + cos(θ)*dx_r - sin(θ)*dy_r
        //   y_new = y + sin(θ)*dx_r + cos(θ)*dy_r
        //   θ_new = θ + dtheta
        // Build process-noise density: w ~ N(0, Q)
        const GaussianInfo<double> pdw = GaussianInfo<double>::fromMoment(Eigen::Vector3d::Zero(), Q);

        // Form joint density p(x, w) — block-diagonal (independent)
        const GaussianInfo<double> joint = density.join(pdw);

        // Propagate: x_new = f(x) + w  (nonlinear mean update, linearised Jacobian)
        auto Phi = [&](const Eigen::VectorXd& xw, Eigen::MatrixXd& J_out) -> Eigen::VectorXd {
            const double x_f = xw(0), y_f = xw(1), theta_f = xw(2);
            const double ct_f = std::cos(theta_f), st_f = std::sin(theta_f);
            Eigen::Vector3d f;
            f(0) = x_f + ct_f * dx_r - st_f * dy_r;
            f(1) = y_f + st_f * dx_r + ct_f * dy_r;
            f(2) = theta_f + dtheta;
            // Jacobian linearised at prior mean heading (theta from mu.z())
            J_out.resize(3, 6);
            J_out.leftCols<3>()  = F;
            J_out.rightCols<3>() = Eigen::Matrix3d::Identity();
            return f + xw.tail<3>();
        };

        density = joint.affineTransform(Phi);

        // Angle-wrap theta in the mean while keeping Xi unchanged
        Eigen::Vector3d mu_new = density.mean();
        mu_new.z()             = normalise_angle(mu_new.z());
        density = GaussianInfo<double>::fromSqrtInfo(density.sqrtInfoMat() * mu_new, density.sqrtInfoMat());

        Hrw_prev = Hrw_current;
    }

}  // namespace module::localisation
