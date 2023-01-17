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
                    // Approx is a Catch2 wrapper class used to allow more tolerant comparisons
                    // when comparing floating point values.
                    // Epsilon sets the cooeffcient by which the result can differ from Approx's value before
                    // it's rejected.
                    Approx approx_x = Approx(cart_to_spher_expected.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx approx_y = Approx(cart_to_spher_expected.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx approx_z = Approx(cart_to_spher_expected.at(i).z()).epsilon(ERROR_THRESHOLD);

                    INFO("Test value at index: " << i);

                    INFO("Calculated x value: " << result.x());
                    INFO("Expected x value: " << cart_to_spher_expected.at(i).x());
                    INFO("Expected approximate x value: " << approx_x.toString());

                    INFO("Calculated y value: " << result.y());
                    INFO("Expected y value: " << cart_to_spher_expected.at(i).y());
                    INFO("Expected approximate y value: " << approx_y.toString());

                    INFO("Calculated z value: " << result.z());
                    INFO("Expected z value: " << cart_to_spher_expected.at(i).z());
                    INFO("Expected approximate z value: " << approx_z.toString());

                    REQUIRE(result.x() == approx_x);
                    REQUIRE(result.y() == approx_y);
                    REQUIRE(result.z() == approx_z);
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
                    // Approx is a Catch2 wrapper class used to allow more tolerant comparisons
                    // when comparing floating point values.
                    // Epsilon sets the cooeffcient by which the result can differ from Approx's value before
                    // it's rejected.
                    Approx approx_x = Approx(spher_to_cart_expected.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx approx_y = Approx(spher_to_cart_expected.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx approx_z = Approx(spher_to_cart_expected.at(i).z()).epsilon(ERROR_THRESHOLD);

                    INFO("Test value at index: " << i);

                    INFO("Calculated x value: " << result.x());
                    INFO("Expected x value: " << spher_to_cart_expected.at(i).x());
                    INFO("Expected approximate x value: " << approx_x.toString());

                    INFO("Calculated y value: " << result.y());
                    INFO("Expected y value: " << spher_to_cart_expected.at(i).y());
                    INFO("Expected approximate y value: " << approx_y.toString());

                    INFO("Calculated z value: " << result.z());
                    INFO("Expected z value: " << spher_to_cart_expected.at(i).z());
                    INFO("Expected approximate z value: " << approx_z.toString());

                    REQUIRE(result.x() == approx_x);
                    REQUIRE(result.y() == approx_y);
                    REQUIRE(result.z() == approx_z);
                }
            }
        }
    }
}
