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
        WHEN("An int vector is declared using the return value of utility::support::resolve_expression(yamlNode[ints])"){
            std::vector<int> intVector = resolve_expression<int>(testValues["ints"]);
            //Check that the values are good(container size, data type....)
            THEN("Container sizes should be equivalent and values in the vector should be the same as the yaml node"){
                //Check container sizes are equivalent
                REQUIRE(testValues["ints"].size() == intVector.size());
                //Check values at each index
                for(size_t i = 0; i < testValues["ints"].size(); i++){
                    REQUIRE(testValues["ints"][i].as<int>() == intVector.at(i));
                }
            }
        }
        //float
        WHEN("A float vector is declared using the return value of utility::support::resolve_expression(yamlNode[floats])"){
            std::vector<float> floatVector = resolve_expression<float>(testValues["floats"]);
            THEN("Container sizes should be equivalent and values in the vector should be the same as the yaml node"){
                //Check container sizes are equivalent
                REQUIRE(testValues["floats"].size() == floatVector.size());
                //check values at each index
                for(size_t i = 0; i < testValues["floats"].size(); i++){
                    REQUIRE(testValues["floats"][i].as<float>() == floatVector.at(i));
                }
            }
        }
        //double
        WHEN("A double vector is declared using the return value of utility::support::resolve_expression(yamlNode[doubles])"){
            std::vector<double> doubleVector = resolve_expression<double>(testValues["doubles"]);
            THEN("Container sizes should be equivalent and values in the vector should be the same as the yaml node"){
                //Check container sizes are equivalent
                REQUIRE(testValues["doubles"].size() == doubleVector.size());
                //check values at each index
                for(size_t i = 0; i < testValues["doubles"].size(); i++){
                    REQUIRE(testValues["doubles"][i].as<double>() == doubleVector.at(i));
                }
            }
        }
    }
}
