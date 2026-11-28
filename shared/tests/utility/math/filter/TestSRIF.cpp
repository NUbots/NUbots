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

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <random>

#include "utility/math/filter/SRIF.hpp"

using utility::math::filter::SRIF;

namespace {
    using Vec4 = Eigen::Matrix<double, 4, 1>;
    using Mat4 = Eigen::Matrix<double, 4, 4>;

    /// Constant velocity model with velocity damping, the same as the cooperative tracker's
    Mat4 transition(const double dt, const double kappa) {
        const double b = std::exp(-kappa * dt);
        const double a = kappa > 0.0 ? (1.0 - b) / kappa : dt;
        Mat4 F         = Mat4::Identity();
        F(0, 2)        = a;
        F(1, 3)        = a;
        F(2, 2)        = b;
        F(3, 3)        = b;
        return F;
    }
}  // namespace

TEST_CASE("SRIF stores and recovers a Gaussian", "[utility][math][filter][SRIF]") {
    Vec4 mean(1.0, -2.0, 0.5, 0.25);
    Eigen::Matrix<double, 4, 4> A;
    A << 2.0, 0.3, 0.1, 0.0, 0.3, 1.0, 0.0, 0.2, 0.1, 0.0, 0.5, 0.05, 0.0, 0.2, 0.05, 0.4;
    const Mat4 P = A * A.transpose();

    SRIF<double, 4> filter(mean, P);
    REQUIRE((filter.mean() - mean).norm() < 1e-9);
    REQUIRE((filter.covariance() - P).norm() < 1e-9);
    // The square-root information matrix is upper triangular with a positive diagonal
    const Mat4 Xi = filter.sqrt_info_matrix();
    REQUIRE(Xi.diagonal().minCoeff() > 0.0);
    REQUIRE(Xi.triangularView<Eigen::StrictlyLower>().toDenseMatrix().norm() == 0.0);
}

TEST_CASE("SRIF matches the moment-form Kalman filter", "[utility][math][filter][SRIF]") {
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    Vec4 mu = Vec4(0.0, 0.0, 0.0, 0.0);
    Mat4 P  = Vec4(1.0, 1.0, 4.0, 4.0).asDiagonal();
    SRIF<double, 4> filter(mu, P);

    const double kappa            = 0.2;
    const double q                = 0.6;
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0)                       = 1.0;
    H(1, 1)                       = 1.0;
    Eigen::Matrix2d R;
    R << 0.09, 0.03, 0.03, 0.02;

    for (int k = 0; k < 200; ++k) {
        // Time update
        const double dt = 0.02 + 0.1 * std::abs(noise(rng));
        const Mat4 F    = transition(dt, kappa);
        Eigen::Matrix<double, 4, 2> G;
        G << dt / 2.0, 0.0, 0.0, dt / 2.0, 1.0, 0.0, 0.0, 1.0;
        const Eigen::Matrix2d Q = q * q * dt * Eigen::Matrix2d::Identity();
        filter.time(F, G, Q);
        mu = F * mu;
        P  = F * P * F.transpose() + G * Q * G.transpose();

        REQUIRE((filter.mean() - mu).norm() < 1e-6);
        REQUIRE((filter.covariance() - P).norm() < 1e-6 * (1.0 + P.norm()));

        // Measurement update, on every other step
        if (k % 2 == 0) {
            const Eigen::Vector2d z(std::sin(0.05 * k) + 0.3 * noise(rng), std::cos(0.05 * k) + 0.3 * noise(rng));
            REQUIRE(filter.measure(Eigen::Vector2d(z), H, R));
            const Eigen::Matrix2d S             = H * P * H.transpose() + R;
            const Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
            mu                                  = mu + K * (z - H * mu);
            P                                   = (Mat4::Identity() - K * H) * P;

            REQUIRE((filter.mean() - mu).norm() < 1e-6);
            REQUIRE((filter.covariance() - P).norm() < 1e-6 * (1.0 + P.norm()));
        }
    }
}

TEST_CASE("SRIF fuses a rank-deficient bearing-like measurement", "[utility][math][filter][SRIF]") {
    // A measurement that only constrains the position across the line of sight (the y axis) must leave the x
    // variance untouched and shrink the y variance
    Vec4 mu = Vec4(2.0, 0.0, 0.0, 0.0);
    Mat4 P  = Vec4(1.0, 1.0, 1.0, 1.0).asDiagonal();
    SRIF<double, 4> filter(mu, P);

    Eigen::Matrix<double, 1, 4> H = Eigen::Matrix<double, 1, 4>::Zero();
    H(0, 1)                       = 1.0;
    const Eigen::Matrix<double, 1, 1> R(0.01);
    const Eigen::Matrix<double, 1, 1> z(0.5);
    REQUIRE(filter.measure(z, H, R));

    const Mat4 Pn = filter.covariance();
    REQUIRE(std::abs(Pn(0, 0) - 1.0) < 1e-9);
    REQUIRE(Pn(1, 1) < 0.0100);
    REQUIRE(std::abs(filter.mean().y() - 0.5 / 1.01) < 1e-9);
}
