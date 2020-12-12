/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <array>
#include <catch.hpp>
#include <cmath>
#include <utility>

#include "utility/math/coordinates.h"
#include "utility/math/quaternion.h"

static constexpr double ERROR_THRESHOLD    = 1e-6;
static constexpr double DBL_LWR_INVLD_EDGE = -1e-8;
static constexpr double DBL_LWR_VALID_EDGE = 1e-8;


// vec3 caretesian test coords
static const std::array<Eigen::Vector3d, 1> cart_coords = {
    Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0)
    // TODO: input edge cases first
    // TODO: Add around 200 test values in between edge cases
};

// vec3 spherical test coords
// distance, theta, phi
static const std::array<Eigen::MatrixBase<double>, 17> spher_coords = {
    // NOTE: should the valid/invalid radial boundaries be tested with each angle value?
    // Edge cases
    //
    Eigen::MatrixBase<double>(-5, 0, 0),        // invalid radial dist
    Eigen::MatrixBase<double>(-DBL_MIN, 0, 0),  // invalid edge radial distance....
    Eigen::MatrixBase<double>(0, 0, 0),         // valid boundary radial dist
    Eigen::MatrixBase<double>(DBL_MIN, 0, 0),   // valid open boundary
    Eigen::MatrixBase<double>(1, 0, 0),         // valid
    //(pi/2)
    Eigen::MatrixBase<double>(1, (cmath::M_PI / 2), 0),
    Eigen::MatrixBase<double>(1, 0, (cmath::M_PI / 2)),
    Eigen::MatrixBase<double>(1, (cmath::M_PI / 2), (cmath::M_PI / 2))
    //(pi)
    Eigen::MatrixBase<double>(1, cmath::M_PI, 0),
    Eigen::MatrixBase<double>(1, 0, cmath::M_PI),
    Eigen::MatrixBase<double>(1, cmath::M_PI, cmath::M_PI),
    //(3pi/2)
    Eigen::MatrixBase<double>(1, ((3 * cmath::M_PI) / 2), 0),
    Eigen::MatrixBase<double>(1, 0, ((3 * cmath::M_PI) / 2)),
    Eigen::MatrixBase<double>(1, ((3 * cmath::M_PI) / 2), ((3 * cmath::M_PI) / 2))
    //(2pi)
    Eigen::MatrixBase<double>(1, 2 * cmath::M_PI, 0),
    Eigen::MatrixBase<double>(1, 0, 2 * cmath::M_PI),
    Eigen::MatrixBase<double>(1, 2 * cmath::M_PI, 2 * cmath::M_PI)

    // TODO: add random values between boundaries - Add angle boundaries


};

static const Eigen::Vector3d spher_true(0, 0, 0);

// Test cartesianToSpherical conversion (Vector3d)
TEST_CASE("Test coordinate conversion - Cartesian to spherical (Vector3d).", "[utility][math][coordinates]") {
    INFO("Calculating Spherical coordinates for the origin");
    // Loop through test values and compare output

    for (auto coord : cart_coords) {
        Eigen::Vector3d spher_result = utility::math::coordinates::cartesianToSpherical(coord);
        // NOTE: Don't compare norms.

        double result_to_truth = (spher_result - spher_true).norm();

        INFO("The distance between the externally calculated result and out utilities result: "
             << result_to_truth << ". This should be small.");

        REQUIRE(result_to_truth <= ERROR_THRESHOLD);
    }
}

// sphericalToCartesian
