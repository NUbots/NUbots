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
// Testing for return types and that data is not fucky
// Use the BDD macros

//
SCENARIO("yaml nodes can be converted to a given container type.", "[utility][support][yaml_expression]"){
    GIVEN("A yaml node containing a list of data"){
        YAML::Node testInts = YAML::LoadFile("/home/nubots/NUbots/shared/tests/data/yamlConversionTestVals.yaml");
        WHEN("A vector is declared using the return value of utility::support::resolve_expression(yamlNode)"){
            const std::vector<int> intVector = resolve_expression<int>(testInts["ints"]);
        }
        //Check that the values aren't fucked (container size, data type....)
        for(size_t i = 0; i < testInts["ints"].size(); i++){
            REQUIRE(1 == 2);
        }
    }
}
