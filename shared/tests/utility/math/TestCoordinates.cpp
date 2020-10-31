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
#include <utility>

#include "utility/math/quaternion.h"

static const std::array<Eigen::Vector3d, 1> cart_coords = {Eigen::Vector3d(0, 0, 0)};

static const Eigen::Vector3d spher_true(0, 0, 0);

TEST_CASE("Test Coordinates", "[utility][math][coordinates]") {

    INFO("Calculating Spherical coordinates for the origin");
    Eigen::Vector3d spher_result = utility::math::coordinates::cartesianToSpherical(cart_coords[0]);

    double result_to_truth = (spher_result - spher_true).norm;
    INFO("The distance between the externally calculated result and out utilities result: "
         << result_to_truth << ". This should be small.");

    REQUIRE(result_to_truth <= 1e-6);
}
