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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cfloat>
#include <cmath>
#include <fmt/format.h>
#include <iomanip>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"

using Catch::Matchers::WithinRel;
using utility::math::coordinates::cartesianToSpherical;
using utility::math::coordinates::sphericalToCartesian;
using utility::support::Expression;
using utility::support::resolve_expression;


/**
 * @author Liam Craft
 */

static constexpr double ERROR_THRESHOLD = 1e-6;

/** Test Data */
static const YAML::Node test_values = YAML::LoadFile("tests/CoordinateTests.yaml");

// cartesian to spherical
SCENARIO("Cartesian coordinates can be converted to spherical coordinates", "[utility][math][coordinates]") {
    GIVEN("A set of cartesian coordinates and a set of expected values") {
        // cart input
        static const std::vector<Eigen::Vector3d> cart_coords =
            resolve_expression<Eigen::Vector3d>(test_values["cartesian_input"]);
        // cart to spher results
        static const std::vector<Eigen::Vector3d> cart_to_spher_expected =
            resolve_expression<Eigen::Vector3d>(test_values["cart_to_spherical_results"]);
        for (size_t i = 0; i < cart_coords.size(); i++) {
            WHEN("Cartesian coordinates are converted to spherical coordinates") {
                static const Eigen::Vector3d result = cartesianToSpherical(cart_coords.at(i));
                THEN("The results of the calculation should be approximately equal to the expected values") {
                    REQUIRE_THAT(result.x(), WithinRel(cart_to_spher_expected.at(i).x(), 1e-6));
                    REQUIRE_THAT(result.y(), WithinRel(cart_to_spher_expected.at(i).y(), 1e-6));
                    REQUIRE_THAT(result.z(), WithinRel(cart_to_spher_expected.at(i).z(), 1e-6));
                }
            }
        }
    }
}

// spherical to cartesian
SCENARIO("Spherical coordinates can be converted to cartesian coordinates", "[utility][math][coordinates]") {
    GIVEN("A set of spherical coordinates and a set of expected values") {
        // spherical input
        static const std::vector<Eigen::Vector3d> spher_coords =
            resolve_expression<Eigen::Vector3d>(test_values["spherical_input"]);
        // spher to cart results
        static const std::vector<Eigen::Vector3d> spher_to_cart_expected =
            resolve_expression<Eigen::Vector3d>(test_values["spherical_to_cartesian_results"]);
        for (size_t i = 0; i < spher_coords.size(); i++) {
            WHEN("Spherical coordinates are converted to cartesian coordinates") {
                static const Eigen::Vector3d result = sphericalToCartesian(spher_coords.at(i));
                THEN("The results of the calculation should be approximately equal to the expected values") {
                    REQUIRE_THAT(result.x(), WithinRel(spher_to_cart_expected.at(i).x(), 1e-6));
                    REQUIRE_THAT(result.y(), WithinRel(spher_to_cart_expected.at(i).y(), 1e-6));
                    REQUIRE_THAT(result.z(), WithinRel(spher_to_cart_expected.at(i).z(), 1e-6));
                }
            }
        }
    }
}
