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

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <vector>

#include "FieldLocalisationNLopt.hpp"

using module::localisation::FieldLocalisationNLopt;

// covariance_from_hessian is a static, pure member function (no reactor/PowerPlant required), so it can be
// exercised directly with hand-built Hessians. finite_difference_hessian itself requires a constructed
// FieldLocalisationNLopt (it calls evaluate_cost, position_in_map, data_association, all of which need the
// module's runtime state such as the field-line distance map and landmark list), so it is not unit tested
// here; it is exercised indirectly via the main loop / Webots verification instead.

TEST_CASE("covariance_from_hessian clamps large and small eigenvalues into [r_min, r_max]", "[Hessian]") {
    Eigen::Matrix3d H  = Eigen::Matrix3d::Zero();
    H(0, 0)            = 100.0;
    H(1, 1)            = 1.0;
    H(2, 2)            = -5.0;  // Non-positive curvature: should clamp to r_max regardless of scale.
    const double scale = 1.0;
    const double r_min = 0.01;
    const double r_max = 10.0;

    Eigen::Matrix3d R = FieldLocalisationNLopt::covariance_from_hessian(H, scale, r_min, r_max);

    REQUIRE(std::abs(R(0, 0) - std::clamp(scale / 100.0, r_min, r_max)) < 1e-9);
    REQUIRE(std::abs(R(1, 1) - std::clamp(scale / 1.0, r_min, r_max)) < 1e-9);
    REQUIRE(std::abs(R(2, 2) - r_max) < 1e-9);
}

TEST_CASE("covariance_from_hessian on an identity Hessian applies the scale uniformly", "[Hessian]") {
    Eigen::Matrix3d H  = Eigen::Matrix3d::Identity();
    const double scale = 2.0;
    const double r_min = 0.001;
    const double r_max = 100.0;

    Eigen::Matrix3d R = FieldLocalisationNLopt::covariance_from_hessian(H, scale, r_min, r_max);

    Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() * std::clamp(scale, r_min, r_max);
    REQUIRE((R - expected).norm() < 1e-9);
}

TEST_CASE("covariance_from_hessian result is symmetric and matches clamped eigenvalues on a coupled Hessian",
          "[Hessian]") {
    // A symmetric Hessian with x/y coupling.
    Eigen::Matrix3d H;
    // clang-format off
    H << 4.0, 1.0, 0.0,
         1.0, 2.0, 0.0,
         0.0, 0.0, 0.5;
    // clang-format on

    const double scale = 1.0;
    const double r_min = 0.01;
    const double r_max = 5.0;

    Eigen::Matrix3d R = FieldLocalisationNLopt::covariance_from_hessian(H, scale, r_min, r_max);

    // Symmetric output.
    REQUIRE((R - R.transpose()).norm() < 1e-9);

    // Round-trip: the eigenvalues of R should be the clamped inverse-scaled eigenvalues of H.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> h_solver(H);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> r_solver(R);

    std::vector<double> expected_variances;
    for (int i = 0; i < 3; ++i) {
        double lambda = h_solver.eigenvalues()(i);
        expected_variances.push_back(lambda <= 1e-9 ? r_max : std::clamp(scale / lambda, r_min, r_max));
    }
    std::sort(expected_variances.begin(), expected_variances.end());

    std::vector<double> actual_variances;
    for (int i = 0; i < 3; ++i) {
        actual_variances.push_back(r_solver.eigenvalues()(i));
    }
    std::sort(actual_variances.begin(), actual_variances.end());

    for (int i = 0; i < 3; ++i) {
        REQUIRE(std::abs(actual_variances[i] - expected_variances[i]) < 1e-9);
    }
}
