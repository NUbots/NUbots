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

TEST_CASE("Enabling no resampling method is invalid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod without any methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    INFO("The state should be invalid");
    REQUIRE(!method_instance.is_valid());
}

TEST_CASE("Residual method enabled alone is invalid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with only residual enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.residual_enabled = true;

    INFO("The state should be invalid");
    REQUIRE(!method_instance.is_valid());
}

TEST_CASE("Residual + multinomial method is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the residual and multinomial methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.residual_enabled    = true;
    method_instance.multinomial_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Residual + systematic method is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the residual and systematic methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.residual_enabled   = true;
    method_instance.systematic_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Residual + stratified method is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the residual and stratified methods enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.residual_enabled   = true;
    method_instance.stratified_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Multinomial method alone is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the Multinomial method enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.multinomial_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Systematic method alone is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the Systematic method enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.systematic_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

TEST_CASE("Stratified method alone is valid", "[utility][math][filter][resample_method]") {
    INFO("Given a ResampleMethod with the Stratified method enabled");
    auto method_instance = utility::math::filter::ResampleMethod();

    method_instance.stratified_enabled = true;

    INFO("The state should be valid");
    REQUIRE(method_instance.is_valid());
}

// TODO: Add tests for multiple disjoint states
