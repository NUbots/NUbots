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
#include "ImuPreFilter.hpp"

#include <cmath>
#include <limits>

namespace module::localisation::srif {

    Eigen::Vector3d Butterworth3::update(const Eigen::Vector3d& x, double dt, double tau) {
        // A non-positive time constant means no filtering
        if (!(tau > 0.0)) {
            reset();
            y = x;
            return y;
        }
        // The coefficients need a time step; hold the first sample until there is one
        if (!(dt > 0.0)) {
            if (!initialised && !pending) {
                y       = x;
                pending = true;
            }
            return y;
        }

        // Bilinear-transform Butterworth coefficients, as in VQF's filterCoeffs
        const double fc = M_SQRT2 / (2.0 * M_PI * tau);
        const double C  = std::tan(M_PI * fc * dt);
        const double D  = C * C + M_SQRT2 * C + 1.0;
        const double b0 = C * C / D;
        const double b1 = 2.0 * b0;
        const double b2 = b0;
        const double a1 = 2.0 * (C * C - 1.0) / D;
        const double a2 = (1.0 - M_SQRT2 * C + C * C) / D;

        // Start in steady state at the first sample, so the output does not ramp up from zero
        if (!initialised) {
            const Eigen::Vector3d x0 = pending ? y : x;
            s2                       = (b2 - a2) * x0;
            s1                       = (b1 - a1) * x0 + s2;
            initialised              = true;
            pending                  = false;
        }

        // Direct form II transposed
        y  = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        return y;
    }

    void ImuPreFilter::reset() {
        initialised = false;
        qib         = Eigen::Quaterniond::Identity();
        acc_i_lp.reset();
        gyro_lp.reset();
        acc_lp.reset();
        rest_time = 0.0;
    }

    void ImuPreFilter::update(double t,
                              const Eigen::Vector3d& gyroscope,
                              const Eigen::Vector3d& accelerometer,
                              const Eigen::Vector3d& gyro_bias) {
        if (!gyroscope.allFinite() || !accelerometer.allFinite()) {
            return;
        }

        const double dt = t - last_t;
        if (initialised && (dt > params.max_gap || dt < 0.0)) {
            reset();
        }
        last_t = t;

        if (!initialised) {
            qib = Eigen::Quaterniond::Identity();
            acc_i_lp.update(accelerometer, 0.0, params.tau_acc);
            gyro_lp.update(gyroscope, 0.0, params.rest_filter_tau);
            acc_lp.update(accelerometer, 0.0, params.rest_filter_tau);
            initialised = true;
            return;
        }
        if (dt == 0.0) {
            return;
        }

        // Strapdown integration of the bias-corrected gyroscope: qib <- qib * exp(omega dt / 2)
        const Eigen::Vector3d omega = gyro_bias.allFinite() ? Eigen::Vector3d(gyroscope - gyro_bias) : gyroscope;
        const double angle          = omega.norm() * dt;
        if (angle > 0.0) {
            qib = (qib * Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized()))).normalized();
        }

        // Gravity is constant in {i}, so filter there
        acc_i_lp.update(qib * accelerometer, dt, params.tau_acc);

        // Rest detection: both signals stay close to their own low-pass filtered values
        const Eigen::Vector3d gyro_mean = gyro_lp.update(gyroscope, dt, params.rest_filter_tau);
        const Eigen::Vector3d acc_mean  = acc_lp.update(accelerometer, dt, params.rest_filter_tau);
        const bool still =
            (gyroscope - gyro_mean).squaredNorm() < params.rest_threshold_gyro * params.rest_threshold_gyro
            && (accelerometer - acc_mean).squaredNorm() < params.rest_threshold_acc * params.rest_threshold_acc;
        rest_time = still ? rest_time + dt : 0.0;
    }

    Eigen::Vector3d ImuPreFilter::gravity() const {
        if (!initialised) {
            return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        }
        return qib.conjugate() * acc_i_lp.value();
    }

}  // namespace module::localisation::srif
