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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */


#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "utility/support/yaml_expression.hpp"


using Catch::Matchers::WithinRel;
using utility::support::Expression;
using utility::support::resolve_expression;

/**
 * Unit tests for yaml_expression.cpp
 *
 * @author Liam Craft
 */
SCENARIO("yaml nodes can be converted to a given container type", "[utility][support][yaml_expression]") {
    GIVEN("A yaml node containing a list of data") {
        YAML::Node test_values = YAML::LoadFile("tests/YamlConversionTestVals.yaml");
        // error threshold when comparing doubles
        static constexpr double ERROR_THRESHOLD = 1e-6;
        // ints
        WHEN("A vector is declared using integers from the YAML node") {
            std::vector<int> int_vector = resolve_expression<int>(test_values["ints"]);
            // Check that the values are good(container size, data type....)
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["ints"].size() == int_vector.size());
            }
            THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < test_values["ints"].size(); i++) {
                    REQUIRE(test_values["ints"][i].as<int>() == int_vector.at(i));
                }
            }
        }

        // float
        WHEN("A vector is declared using float values from the YAML node") {
            std::vector<float> float_vector = resolve_expression<float>(test_values["floats"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["floats"].size() == float_vector.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["floats"].size(); i++) {
                    REQUIRE_THAT(test_values["floats"][i].as<float>(),
                                 WithinRel(float_vector.at(i), float(ERROR_THRESHOLD)));
                }
            }
        }

        // double
        WHEN("A vector is declared using doubles from the YAML node") {
            std::vector<double> double_vector = resolve_expression<double>(test_values["doubles"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["doubles"].size() == double_vector.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["doubles"].size(); i++) {
                    REQUIRE_THAT(test_values["doubles"][i].as<double>(),
                                 WithinRel(double_vector.at(i), ERROR_THRESHOLD));
                }
            }
        }

        // Vector2d
        WHEN("A vector is declared using Vector2d values from the YAML node") {
            std::vector<Eigen::Vector2d> v2d_vec = resolve_expression<Eigen::Vector2d>(test_values["vector2"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["vector2"].size() == v2d_vec.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["vector2"].size(); i++) {
                    // NOTE: Must test individual xy values
                    REQUIRE_THAT(test_values["vector2"][i][0].as<double>(),
                                 WithinRel(v2d_vec.at(i).x(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector2"][i][1].as<double>(),
                                 WithinRel(v2d_vec.at(i).y(), ERROR_THRESHOLD));
                }
            }
        }

        // Vector3d
        WHEN("A vector is declared using Vector3d values from the YAML node") {
            std::vector<Eigen::Vector3d> v3d_vec = resolve_expression<Eigen::Vector3d>(test_values["vector3"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["vector3"].size() == v3d_vec.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["vector3"].size(); i++) {
                    // NOTE: Must test individual xyz values
                    REQUIRE_THAT(test_values["vector3"][i][0].as<double>(),
                                 WithinRel(v3d_vec.at(i).x(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector3"][i][1].as<double>(),
                                 WithinRel(v3d_vec.at(i).y(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector3"][i][2].as<double>(),
                                 WithinRel(v3d_vec.at(i).z(), ERROR_THRESHOLD));
                }
            }
        }

        // Array conversions
        // ints
        WHEN("An array is declared using integers from the YAML node") {
            std::array<int, 20> int_array = resolve_expression<int, 20>(test_values["ints"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["ints"].size() == int_array.size());
            }
            THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < test_values["ints"].size(); i++) {
                    REQUIRE(test_values["ints"][i].as<int>() == int_array.at(i));
                }
            }
        }
        // floats
        WHEN("An array is declared using floats from the YAML node") {
            std::array<float, 20> float_array = resolve_expression<float, 20>(test_values["floats"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["floats"].size() == float_array.size());
            }
            THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < test_values["floats"].size(); i++) {
                    REQUIRE_THAT(test_values["floats"][i].as<float>(),
                                 WithinRel(float_array.at(i), float(ERROR_THRESHOLD)));
                }
            }
        }
        // doubles
        WHEN("An array is declared using doubles from the YAML node") {
            std::array<double, 20> double_array = resolve_expression<double, 20>(test_values["doubles"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["doubles"].size() == double_array.size());
            }
            THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < test_values["doubles"].size(); i++) {
                    REQUIRE_THAT(test_values["doubles"][i].as<double>(),
                                 WithinRel(double_array.at(i), ERROR_THRESHOLD));
                }
            }
        }
        // vector3d
        WHEN("An array is declared using Vector3d values from the YAML node") {
            std::array<Eigen::Vector3d, 20> v3d_array = resolve_expression<Eigen::Vector3d, 20>(test_values["vector3"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["vector3"].size() == v3d_array.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["vector3"].size(); i++) {
                    // NOTE: Must test individual xyz values
                    REQUIRE_THAT(test_values["vector3"][i][0].as<double>(),
                                 WithinRel(v3d_array.at(i).x(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector3"][i][1].as<double>(),
                                 WithinRel(v3d_array.at(i).y(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector3"][i][2].as<double>(),
                                 WithinRel(v3d_array.at(i).z(), ERROR_THRESHOLD));
                }
            }
        }
        // vector2d
        WHEN("An array is declared using Vector2d values from the YAML node") {
            std::array<Eigen::Vector2d, 20> v2d_array = resolve_expression<Eigen::Vector2d, 20>(test_values["vector2"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(test_values["vector2"].size() == v2d_array.size());
            }
            THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < test_values["vector2"].size(); i++) {
                    // NOTE: Must test individual xy values
                    REQUIRE_THAT(test_values["vector2"][i][0].as<double>(),
                                 WithinRel(v2d_array.at(i).x(), ERROR_THRESHOLD));
                    REQUIRE_THAT(test_values["vector2"][i][1].as<double>(),
                                 WithinRel(v2d_array.at(i).y(), ERROR_THRESHOLD));
                }
            }
        }
    }
}
