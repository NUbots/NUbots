/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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
