/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <numeric>
#include <yaml-cpp/yaml.h>

#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/support/yaml_expression.hpp"

using utility::math::geometry::chans_convex_hull;
using utility::math::geometry::point_in_convex_hull;
using utility::support::resolve_expression;

static const YAML::Node test_values = YAML::LoadFile("tests/TestConvexHull.yaml");

SCENARIO("Convex hull of a square can be calculated using Chan's algorithm", "[utility][math][geometry]") {
    GIVEN("A set of points and a set of expected convex hull values") {
        // Get the points from the yaml file
        static const std::vector<Eigen::Vector2d> points =
            resolve_expression<Eigen::Vector2d>(test_values["square"]["input"]);

        // Set up the indices vector
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);

        // Make a matrix from the points
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(2, points.size());
        for (size_t i = 0; i < points.size(); i++) {
            points_matrix.col(i) << points[i][0], points[i][1];
        }

        // Get the expected hull points
        static const std::vector<int> expected_hull = test_values["square"]["hull"].as<std::vector<int>>();

        WHEN("A convex hull is calculated using Chan's algorithm") {
            static const std::vector<int> result = utility::math::geometry::chans_convex_hull(indices, points_matrix);

            THEN("The results of the calculation should be equal to the expected values") {
                REQUIRE(result.size() == expected_hull.size());
                REQUIRE(result == expected_hull);
            }
        }
    }
}


SCENARIO("Convex hull of a triangle can be calculated using Chan's algorithm", "[utility][math][geometry]") {
    GIVEN("A set of points and a set of expected convex hull values") {
        // Get the points from the yaml file
        static const std::vector<Eigen::Vector2d> points =
            resolve_expression<Eigen::Vector2d>(test_values["triangle"]["input"]);

        // Set up the indices vector
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);

        // Make a matrix from the points
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(2, points.size());
        for (size_t i = 0; i < points.size(); i++) {
            points_matrix.col(i) << points[i][0], points[i][1];
        }

        // Get the expected hull points
        static const std::vector<int> expected_hull = test_values["triangle"]["hull"].as<std::vector<int>>();

        WHEN("A convex hull is calculated using Chan's algorithm") {
            static const std::vector<int> result = utility::math::geometry::chans_convex_hull(indices, points_matrix);

            THEN("The results of the calculation should be equal to the expected values") {
                REQUIRE(result.size() == expected_hull.size());
                REQUIRE(result == expected_hull);
            }
        }
    }
}


SCENARIO("Convex hull of a polygon can be calculated using Chan's algorithm", "[utility][math][geometry]") {
    GIVEN("A set of points and a set of expected convex hull values") {
        // Get the points from the yaml file
        static const std::vector<Eigen::Vector2d> points =
            resolve_expression<Eigen::Vector2d>(test_values["polygon"]["input"]);

        // Set up the indices vector
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);

        // Make a matrix from the points
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(2, points.size());
        for (size_t i = 0; i < points.size(); i++) {
            points_matrix.col(i) << points[i][0], points[i][1];
        }

        // Get the expected hull points
        static const std::vector<int> expected_hull = test_values["polygon"]["hull"].as<std::vector<int>>();

        WHEN("A convex hull is calculated using Chan's algorithm") {
            static const std::vector<int> result = utility::math::geometry::chans_convex_hull(indices, points_matrix);

            THEN("The results of the calculation should be equal to the expected values") {
                REQUIRE(result.size() == expected_hull.size());
                REQUIRE(result == expected_hull);

                // Check that we know if a point is in or outside of the hull
                std::vector<Eigen::Vector2d> hull_points;
                for (const auto& idx : result) {
                    hull_points.push_back(points[idx]);
                }

                // The points to check for in/out-ness
                std::vector<Eigen::Vector2d> check_points =
                    resolve_expression<Eigen::Vector2d>(test_values["polygon"]["check_points"]);

                // The expected results of the check. True if inside, false if outside
                std::vector<bool> check_results = resolve_expression<bool>(test_values["polygon"]["check_results"]);

                for (size_t i = 0; i < check_points.size(); i++) {
                    REQUIRE(point_in_convex_hull(hull_points, check_points[i]) == check_results[i]);
                }
            }
        }
    }
}
