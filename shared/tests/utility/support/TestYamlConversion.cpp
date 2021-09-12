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


#include <catch.hpp>
#include <utility>
#include <string>

#include <yaml-cpp/yaml.h>
#include "utility/support/yaml_expression.hpp"


using utility::support::Expression;
using utility::support::resolve_expression;

/**
 * @author Liam Craft
 */

// MY NOTES: test for primitives (int, double), Eigens(vectors, matrices), strings
// Testing for return types and that data is good
// Use the BDD macros

//
SCENARIO("yaml nodes can be converted to a given container type", "[utility][support][yaml_expression]"){
    GIVEN("A yaml node containing a list of data"){
        YAML::Node testValues = YAML::LoadFile("/home/nubots/NUbots/shared/tests/data/yamlConversionTestVals.yaml");
        //ints
        WHEN("A vector is declared using integers from the YAML node"){
            std::vector<int> intVector = resolve_expression<int>(testValues["ints"]);
            //Check that the values are good(container size, data type....)
            THEN("Container sizes should be equivalent"){
                REQUIRE(testValues["ints"].size() == intVector.size());
            }
            AND_THEN("Values at each index should be equivalent"){
                //Check values at each index
                for(size_t i = 0; i < testValues["ints"].size(); i++){
                    REQUIRE(testValues["ints"][i].as<int>() == intVector.at(i));
                }
            }
        }
        //float
        AND_WHEN("A vector is declared using float values from the YAML node"){
            std::vector<float> floatVector = resolve_expression<float>(testValues["floats"]);
            THEN("Container sizes should be equivalent"){
                REQUIRE(testValues["floats"].size() == floatVector.size());
            }
            AND_THEN("Values at each index should be equivalent"){
                for(size_t i = 0; i < testValues["floats"].size(); i++){
                    REQUIRE(testValues["floats"][i].as<float>() == floatVector.at(i));
                }
            }
        }
        //double
        AND_WHEN("A vector is declared using doubles from the YAML node"){
            std::vector<double> doubleVector = resolve_expression<double>(testValues["doubles"]);
            THEN("Container sizes should be equivalent"){
                REQUIRE(testValues["doubles"].size() == doubleVector.size());
            }
            AND_THEN("Values at each index should be equivalent"){
                for(size_t i = 0; i < testValues["doubles"].size(); i++){
                    REQUIRE(testValues["doubles"][i].as<double>() == doubleVector.at(i));
                }
            }
        }
        //strings
        // AND_WHEN("A Vector is declared using strings from the YAML node"){
        //     std::vector<std::string> stringVector = resolve_expression<std::string>(testValues["strings"]);
        //     THEN("Container sizes should be equivalent"){
        //         REQUIRE(testValues["strings"].size() == stringVector.size());
        //     }
        //     AND_THEN("Values at each index should be equivalent"){
        //         for(size_t i = 0; i < testValues["strings"].size(); i++){
        //             REQUIRE(testValues["strings"][i].as<std::string>() == stringVector.at(i));
        //         }
        //     }
        // }
        //Eigen things: Affine2d, Affine3d, AngleAxisd, Vector3d, Vector2d, Rotation2Dd,

    }
}
