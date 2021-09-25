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
#include <catch.hpp>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "utility/support/yaml_expression.hpp"


using utility::support::Expression;
using utility::support::resolve_expression;

/**
 * Unit tests for [utility][support][yaml_expression]
 *
 * @author Liam Craft
 */


SCENARIO("yaml nodes can be converted to a given container type", "[utility][support][yaml_expression]") {
    GIVEN("A yaml node containing a list of data") {
        YAML::Node testValues = YAML::LoadFile("/home/nubots/NUbots/shared/tests/data/yamlConversionTestVals.yaml");
        // error threshold when comparing doubles
        static constexpr double ERROR_THRESHOLD = 1e-6;
        // ints
        WHEN("A vector is declared using integers from the YAML node") {
            std::vector<int> intVector = resolve_expression<int>(testValues["ints"]);
            // Check that the values are good(container size, data type....)
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["ints"].size() == intVector.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < testValues["ints"].size(); i++) {
                    REQUIRE(testValues["ints"][i].as<int>() == intVector.at(i));
                }
            }
        }

        // float
        AND_WHEN("A vector is declared using float values from the YAML node") {
            std::vector<float> floatVector = resolve_expression<float>(testValues["floats"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["floats"].size() == floatVector.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["floats"].size(); i++) {
                    REQUIRE(testValues["floats"][i].as<float>() == floatVector.at(i));
                }
            }
        }

        // double
        AND_WHEN("A vector is declared using doubles from the YAML node") {
            std::vector<double> doubleVector = resolve_expression<double>(testValues["doubles"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["doubles"].size() == doubleVector.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["doubles"].size(); i++) {
                    Approx epsilon_result = Approx(doubleVector.at(i)).epsilon(ERROR_THRESHOLD);
                    REQUIRE(testValues["doubles"][i].as<double>() == epsilon_result);
                }
            }
        }

        // Vector2d
        AND_WHEN("A vector is declared using Vector2d values from the YAML node") {
            std::vector<Eigen::Vector2d> eigenV2dVector = resolve_expression<Eigen::Vector2d>(testValues["vector2"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["vector2"].size() == eigenV2dVector.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["vector2"].size(); i++) {
                    Approx epsilon_x = Approx(eigenV2dVector.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(eigenV2dVector.at(i).y()).epsilon(ERROR_THRESHOLD);
                    // NOTE: Must test individual xy values
                    REQUIRE(testValues["vector2"][i][0].as<double>() == epsilon_x);
                    REQUIRE(testValues["vector2"][i][1].as<double>() == epsilon_y);
                }
            }
        }

        // Vector3d
        AND_WHEN("A vector is declared using Vector3d values from the YAML node") {
            std::vector<Eigen::Vector3d> eigenV3dVector = resolve_expression<Eigen::Vector3d>(testValues["vector3"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["vector3"].size() == eigenV3dVector.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["vector3"].size(); i++) {
                    Approx epsilon_x = Approx(eigenV3dVector.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(eigenV3dVector.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_z = Approx(eigenV3dVector.at(i).z()).epsilon(ERROR_THRESHOLD);
                    // NOTE: Must test individual xyz values
                    REQUIRE(testValues["vector3"][i][0].as<double>() == epsilon_x);
                    REQUIRE(testValues["vector3"][i][1].as<double>() == epsilon_y);
                    REQUIRE(testValues["vector3"][i][2].as<double>() == epsilon_z);
                }
            }
        }

        // Array conversions
        // ints
        AND_WHEN("An array is declared using integers from the YAML node") {
            std::array<int, 20> intArray = resolve_expression<int, 20>(testValues["ints"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["ints"].size() == intArray.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < testValues["ints"].size(); i++) {
                    REQUIRE(testValues["ints"][i].as<int>() == intArray.at(i));
                }
            }
        }
        // floats
        AND_WHEN("An array is declared using floats from the YAML node") {
            std::array<float, 20> floatArray = resolve_expression<float, 20>(testValues["floats"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["floats"].size() == floatArray.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < testValues["floats"].size(); i++) {
                    REQUIRE(testValues["floats"][i].as<float>() == floatArray.at(i));
                }
            }
        }
        // doubles
        AND_WHEN("An array is declared using doubles from the YAML node") {
            std::array<double, 20> doubleArray = resolve_expression<double, 20>(testValues["doubles"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["doubles"].size() == doubleArray.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                // Check values at each index
                for (size_t i = 0; i < testValues["doubles"].size(); i++) {
                    Approx epsilon_result = Approx(doubleArray.at(i)).epsilon(ERROR_THRESHOLD);
                    REQUIRE(testValues["doubles"][i].as<double>() == epsilon_result);
                }
            }
        }
        // vector3d
        AND_WHEN("An array is declared using Vector3d values from the YAML node") {
            std::array<Eigen::Vector3d, 20> eigenV3dArray =
                resolve_expression<Eigen::Vector3d, 20>(testValues["vector3"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["vector3"].size() == eigenV3dArray.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["vector3"].size(); i++) {
                    Approx epsilon_x = Approx(eigenV3dArray.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(eigenV3dArray.at(i).y()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_z = Approx(eigenV3dArray.at(i).z()).epsilon(ERROR_THRESHOLD);
                    // NOTE: Must test individual xyz values
                    REQUIRE(testValues["vector3"][i][0].as<double>() == epsilon_x);
                    REQUIRE(testValues["vector3"][i][1].as<double>() == epsilon_y);
                    REQUIRE(testValues["vector3"][i][2].as<double>() == epsilon_z);
                }
            }
        }
        // vector2d
        AND_WHEN("An array is declared using Vector2d values from the YAML node") {
            std::array<Eigen::Vector2d, 20> eigenV2dArray =
                resolve_expression<Eigen::Vector2d, 20>(testValues["vector2"]);
            THEN("Container sizes should be equivalent") {
                REQUIRE(testValues["vector2"].size() == eigenV2dArray.size());
            }
            AND_THEN("Values at each index should be equivalent") {
                for (size_t i = 0; i < testValues["vector2"].size(); i++) {
                    Approx epsilon_x = Approx(eigenV2dArray.at(i).x()).epsilon(ERROR_THRESHOLD);
                    Approx epsilon_y = Approx(eigenV2dArray.at(i).y()).epsilon(ERROR_THRESHOLD);
                    // NOTE: Must test individual xy values
                    REQUIRE(testValues["vector2"][i][0].as<double>() == epsilon_x);
                    REQUIRE(testValues["vector2"][i][1].as<double>() == epsilon_y);
                }
            }
        }
    }
}
