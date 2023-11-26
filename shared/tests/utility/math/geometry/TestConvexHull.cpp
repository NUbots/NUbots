#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <numeric>
#include <yaml-cpp/yaml.h>

#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/support/yaml_expression.hpp"

using utility::math::geometry::chans_convex_hull;
using utility::support::Expression;
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
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(points.size(), 2);
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
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(points.size(), 2);
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
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_matrix(points.size(), 2);
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
            }
        }
    }
}
