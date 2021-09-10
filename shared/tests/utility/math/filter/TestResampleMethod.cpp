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

#include <catch.hpp>

#include "utility/math/filter/ResampleMethod.hpp"

namespace {
    void set_all_methods_false(utility::math::filter::ResampleMethod& method) {
        method.multinomial_enabled = false;
        method.residual_enabled    = false;
        method.stratified_enabled  = false;
        method.systematic_enabled  = false;
    }
}  // namespace

TEST_CASE("Residual method enabled alone is invalid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with only residual enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.residual_enabled = true;

    INFO("The state should be invalid");
    REQUIRE(!method_instance.is_valid());
}

TEST_CASE("Residual + multinomial method is valid") {
    INFO("Given a ResampleMethod with the residual and multinomial methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.residual_enabled    = true;
    method_instance.multinomial_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Residual + systematic method is valid") {
    INFO("Given a ResampleMethod with the residual and systematic methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.residual_enabled   = true;
    method_instance.systematic_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Residual + stratified method is valid") {
    INFO("Given a ResampleMethod with the residual and stratified methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.residual_enabled   = true;
    method_instance.stratified_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Multinomial method alone is valid") {
    INFO("Given a ResampleMethod with the Multinomial methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.multinomial_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Systematic method alone is valid") {
    INFO("Given a ResampleMethod with the Systematic methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.systematic_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Stratified method alone is valid") {
    INFO("Given a ResampleMethod with the Stratified methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();
    set_all_methods_false(method_instance);
    method_instance.stratified_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

// TODO: Add tests for multiple disjoint states
