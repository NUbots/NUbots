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
#include <cfloat>
#include <cmath>
#include <fmt/format.h>
#include <iomanip>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"

using utility::math::coordinates::cartesianToSpherical;
using utility::math::coordinates::sphericalToCartesian;
using utility::support::Expression;
using utility::support::resolve_expression;


/**
 * Test data and functions to validate coordinates.cpp functions.
 *
 * Test value domain to stay between the values of TEST_DBL_MAX = 5.9667e+153, and TEST_DBL_MIN = 1.4853e-307
 * This means the usual domain edge cases - DBL_MAX and DBL_MIN - have been left out or replaced. It has been determined
 * that inputs approaching these edge cases will not be possible in normal operation. See coordinates.h comments for
 * more details.
 * @author Liam Craft
 */

static constexpr double ERROR_THRESHOLD = 1e-6;
// static constexpr double TEST_DBL_MAX    = 5.9667e+153;
// static constexpr double TEST_DBL_MIN    = 1.4853e-307;


/** Cartesian test coords */
YAML::Node test_values = YAML::LoadFile("tests/CoordinateTests.yaml");

// BDD cartesian to spherical
SCENARIO("Cartesian coordinates can be converted to spherical coordinates", "[utility][math][coordinates]") {
    GIVEN("A set of cartesian coordinates and a set of expected values") {
        // cart input
        static const std::array<Eigen::Vector3d, 200> cart_coords =
            resolve_expression<Eigen::Vector3d, 200>(test_values["cartesianInput"]);
        // cart to spher results
        static const std::array<Eigen::Vector3d, 200> cart_to_spher_expected =
            resolve_expression<Eigen::Vector3d, 200>(test_values["cartToSpher_results"]);
        for (size_t i = 0; i < cart_coords.size(); i++) {
            WHEN("Cartesian coordinates are converted to spherical coordinates") {
                static const Eigen::Vector3d result = cartesianToSpherical(cart_coords.at(i));
                THEN("The results of the calculation should be approximately equal to the expected values") {
                    // NOTE:We use approx because floating points dont like being compared
                    // at the epsilon value of the expected results
                    Approx epsilon_x = Approx(cart_to_spher_expected.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(cart_to_spher_expected.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_z = Approx(cart_to_spher_expected.at(i).z()).epsilon(ERROR_THRESHOLD);
                    // This passes!?!?!?
                    REQUIRE((result.x() + 100) == epsilon_x);
                    REQUIRE(result.y() == epsilon_y);
                    REQUIRE(result.z() == epsilon_z);
                    // REQUIRE(1 == 2);
                }
            }
        }

        // TEST
        // WHEN("TEST") {
        //     THEN("Pass") {
        //         REQUIRE(1 == 1);
        //     }
        // }
    }
}

// BDD spherical to cartesian
SCENARIO("Spherical coordinates can be converted to cartesian coordinates", "[utility][math][coordinates]") {
    GIVEN("A set of spherical coordinates and a set of expected values") {
        // spherical input
        static const std::array<Eigen::Vector3d, 210> spher_coords =
            resolve_expression<Eigen::Vector3d, 210>(test_values["sphericalInput"]);
        // spher to cart results
        static const std::array<Eigen::Vector3d, 210> spher_to_cart_expected =
            resolve_expression<Eigen::Vector3d, 210>(test_values["spherToCart_results"]);
        for (size_t i = 0; i < spher_coords.size(); i++) {
            WHEN("Spherical coordinates are converted to cartesian coordinates") {
                static const Eigen::Vector3d result = sphericalToCartesian(spher_coords.at(i));
                THEN("The results of the calculation should be approximately equal to the expected values") {
                    Approx epsilon_x = Approx(spher_to_cart_expected.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(spher_to_cart_expected.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_z = Approx(spher_to_cart_expected.at(i).z()).epsilon(ERROR_THRESHOLD);
                    REQUIRE(result.x() == epsilon_x);
                    REQUIRE(result.y() == epsilon_y);
                    REQUIRE(result.z() == epsilon_z);
                }
            }
        }

        // TEST
        // WHEN("TEST") {
        //     THEN("Pass") {
        //         REQUIRE(1 == 1);
        //     }
        // }
    }
}
