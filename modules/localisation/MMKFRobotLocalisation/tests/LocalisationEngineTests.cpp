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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <catch.hpp>

#include "utility/math/angle.h"
#include "utility/localisation/transform.h"

using utility::localisation::transform::WorldToRobotTransform;
using utility::localisation::transform::RobotToWorldTransform;
using utility::math::angle::normalizeAngle;

TEST_CASE("Angle convinience functions should handle corner cases", "[math][angle]") {

    // INFO("Testing the FSR centre conversions");

    for(int i = 0; i < 10000; ++i) {
        float a = i * M_PI / 10000.0;

        REQUIRE(normalizeAngle(a + 2 * M_PI) ==
            normalizeAngle(a));

        REQUIRE(normalizeAngle(a - 2 * M_PI) ==
            normalizeAngle(a));
    }

    REQUIRE(normalizeAngle(0.0) == Approx(0.0));
    REQUIRE(normalizeAngle( M_PI) == Approx(M_PI));
    REQUIRE(normalizeAngle(-M_PI) == Approx(M_PI));
    REQUIRE(normalizeAngle( M_PI * 2) == Approx(0.0));
    REQUIRE(normalizeAngle(-M_PI * 2) == Approx(0.0));
    REQUIRE(normalizeAngle( M_PI * 77 - 0.1) == Approx(M_PI - 0.1));
    REQUIRE(normalizeAngle(-M_PI * 77 + 0.1) == Approx(-M_PI + 0.1));
}

TEST_CASE("RobotToWorldTransform should be inverse of WorldToRobotTransform") {

    // INFO("Testing the FSR centre conversions");

    arma::vec2 robot_pos = { 3, 2 };
    arma::vec2 robot_heading = arma::normalise(arma::vec({ 1, 5 }));
    arma::vec2 field_ball = { -3, -1 };

    auto robot_ball = WorldToRobotTransform(robot_pos, robot_heading, field_ball);
    auto result_ball = RobotToWorldTransform(robot_pos, robot_heading, robot_ball);

    REQUIRE(field_ball(0) == Approx(result_ball(0)));
    REQUIRE(field_ball(1) == Approx(result_ball(1)));
}
