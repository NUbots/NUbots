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


