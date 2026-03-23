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

namespace module::localisation {

    using utility::math::GaussianInfo;
    using utility::math::angle::normalise_angle;

    bool FieldLocalisationNLopt::measurement_update(const Eigen::Vector3d& nlopt_result, double cost) {
        // Reject if cost exceeds threshold — density remains at the prediction
        if (cost >= cfg.cost_threshold) {
            return false;
        }

        if (first_measurement) {
            // First accepted update: seed EKF state directly without blending
            density = GaussianInfo<double>::fromMoment(nlopt_result,
                                                       cfg.initial_covariance * Eigen::Matrix3d::Identity());
            first_measurement = false;
            has_prev_Hrw      = false;
            return true;
        }

        // SRIF measurement update — NLopt result is a direct observation of state (H = I₃)
        // Measurement noise R = scale * max(cost, 1e-6) * I₃
        // Xi_meas = 1/sqrt(r) * I₃  (sqrt of information matrix of measurement noise)
        const double r            = cfg.measurement_noise_scale * std::max(cost, 1e-6);
        const double xi_meas      = 1.0 / std::sqrt(r);

        // Angle-wrap the innovation term: z = nlopt_result with theta wrapped relative to current mean
        Eigen::Vector3d z    = nlopt_result;
        z.z()                = normalise_angle(nlopt_result.z() - density.mean().z()) + density.mean().z();

        // Stack [Xi_prior;         xi_meas * I₃  ] and [nu_prior;        xi_meas * z]
        // QR of the stacked 6×3 matrix → upper 3×3 block gives new Xi
        // QR of the stacked 6×4 matrix (appended with rhs) → upper 3×1 gives new nu
        Eigen::Matrix<double, 6, 4> Ab;
        Ab.topLeftCorner<3, 3>()    = density.sqrtInfoMat();
        Ab.bottomLeftCorner<3, 3>() = xi_meas * Eigen::Matrix3d::Identity();
        Ab.topRightCorner<3, 1>()   = density.sqrtInfoVec();
        Ab.bottomRightCorner<3, 1>() = xi_meas * z;

        Eigen::HouseholderQR<Eigen::Matrix<double, 6, 4>> qr(Ab);
        // matrixQR gives the upper-triangular R from QR in the top rows
        const Eigen::Matrix<double, 6, 4> R =
            qr.matrixQR().template triangularView<Eigen::Upper>();

        const Eigen::Matrix3d  Xi_new = R.topLeftCorner<3, 3>();
        const Eigen::Vector3d  nu_new = R.topRightCorner<3, 1>();

        density = GaussianInfo<double>::fromSqrtInfo(nu_new, Xi_new);

        // Angle-wrap theta in the mean while keeping Xi unchanged
        Eigen::Vector3d mu = density.mean();
        mu.z()             = normalise_angle(mu.z());
        density            = GaussianInfo<double>::fromSqrtInfo(Xi_new * mu, Xi_new);

        return true;
    }

}  // namespace module::localisation
