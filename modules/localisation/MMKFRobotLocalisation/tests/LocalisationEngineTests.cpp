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

TEST_CASE("Angle convinience functions should handle corner cases", "[math][angle]") {

    // INFO("Testing the FSR centre conversions");

    for(int i = 0; i < 10000; ++i) {
        float a = i * M_PI / 10000.0;

        REQUIRE(utility::math::angle::normalizeAngle(a + 2 * M_PI) ==
            utility::math::angle::normalizeAngle(a));

        REQUIRE(utility::math::angle::normalizeAngle(a - 2 * M_PI) ==
            utility::math::angle::normalizeAngle(a));
    }

    REQUIRE(utility::math::angle::normalizeAngle(0.0) == Approx(0.0));
    REQUIRE(utility::math::angle::normalizeAngle( M_PI) == Approx(M_PI));
    REQUIRE(utility::math::angle::normalizeAngle(-M_PI) == Approx(M_PI));
    REQUIRE(utility::math::angle::normalizeAngle( M_PI * 2) == Approx(0.0));
    REQUIRE(utility::math::angle::normalizeAngle(-M_PI * 2) == Approx(0.0));
    REQUIRE(utility::math::angle::normalizeAngle( M_PI * 77 - 0.1) == Approx(M_PI - 0.1));
    REQUIRE(utility::math::angle::normalizeAngle(-M_PI * 77 + 0.1) == Approx(-M_PI + 0.1));
}


