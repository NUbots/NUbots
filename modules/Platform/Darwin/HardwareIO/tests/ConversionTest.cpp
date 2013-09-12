/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinPlatform.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include <catch.hpp>
#include "Convert.h"

using namespace modules::Platform::Darwin;

TEST_CASE("Testing the hardware accelerometer conversions to SI units", "[hardware][conversion][accelerometer]") {

    REQUIRE(Convert::accelerometer(0)    == Approx(-4.0 * 9.80665));  // Should be -4g
    REQUIRE(Convert::accelerometer(512)  == Approx(0.0));             // Should be 0
    REQUIRE(Convert::accelerometer(640)  == Approx(9.80665));       // Should be +1g
    REQUIRE(Convert::accelerometer(1024) == Approx(4.0 * 9.80665));   // Should be +4g
}

TEST_CASE("Testing the hardware gyroscope conversions to SI units", "[hardware][conversion][gyroscope]") {

    REQUIRE(Convert::gyroscope(0)    == Approx((-1600.0 * M_PI) / 180.0)); // Should be -1600 degrees/second in radians/second
    REQUIRE(Convert::gyroscope(512)  == Approx(0));                        // Should be 0
    REQUIRE(Convert::gyroscope(1024) == Approx((1600.0 * M_PI) / 180.0));  // Should be 1600 degrees/second in radians/second
}

TEST_CASE("Testing the hardware voltage conversions to SI units", "[hardware][conversion][voltage]") {

    INFO("Testing the voltage conversion");
    REQUIRE(Convert::voltage(0)    == Approx(0));   // Should be 0 volts
    REQUIRE(Convert::voltage(120)  == Approx(12)); // Should be 12 volts
    REQUIRE(Convert::voltage(96)   == Approx(9.6)); // Should be 9.6 volts
}

TEST_CASE("Testing the hardware FSR conversions to SI units", "[hardware][conversion][fsr]") {

    INFO("Testing the FSR force conversion");
    REQUIRE(Convert::fsrForce(670)   == Approx(0.67));   // Should be 0.67 newtons
    REQUIRE(Convert::fsrForce(1100)  == Approx(1.1));    // Should be 1.1 newtons
    REQUIRE(Convert::fsrForce(12345) == Approx(12.345)); // Should be 12.345 newtons

    INFO("Testing the FSR centre conversions");
    // Test Left Foot
    // Test X
    REQUIRE(Convert::fsrCentre(true, true, 0)     == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(true, true, 254)   == Approx(-1)); // Should be -1
    REQUIRE(Convert::fsrCentre(true, true, 127)   == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, true, 0xFF)));    // Should be NaN
    // Test Y
    REQUIRE(Convert::fsrCentre(true, false, 0)    == Approx(-1)); // Should be -1
    REQUIRE(Convert::fsrCentre(true, false, 254)  == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(true, false, 127)  == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, false, 0xFF)));   // Should be NaN
    // Test Right Foot
    // Test X
    REQUIRE(Convert::fsrCentre(false, true, 0)    == Approx(-1)); // Should be -1
    REQUIRE(Convert::fsrCentre(false, true, 254)  == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(false, true, 127)  == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, true, 0xFF)));    // Should be NaN
    // Test Y
    REQUIRE(Convert::fsrCentre(false, false, 0)   == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(false, false, 254) == Approx(-1)); // Should be -1
    REQUIRE(Convert::fsrCentre(false, false, 127) == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, false, 0xFF)));   // Should be NaN
}

TEST_CASE("Testing the hardware coloured LED conversions to 24bit rgb", "[hardware][conversion][led]") {

    INFO("Testing the forward coloured LED conversions");
    REQUIRE(Convert::colourLED(0)      == std::make_tuple(0, 0, 0));            // Should be black
    REQUIRE(Convert::colourLED(0x1F)   == std::make_tuple(0xF8, 0, 0));         // Should be red
    REQUIRE(Convert::colourLED(0x3E0)  == std::make_tuple(0, 0xF8, 0));         // Should be green
    REQUIRE(Convert::colourLED(0x7C00) == std::make_tuple(0, 0, 0xF8));         // Should be blue
    REQUIRE(Convert::colourLED(0x7FFF) == std::make_tuple(0xF8, 0xF8, 0xF8));   // Should be white
    REQUIRE(Convert::colourLED(0x7C1F) == std::make_tuple(0xF8, 0, 0xF8));      // Should be red and blue

    INFO("Testing the inverse coloured LED conversions");
    REQUIRE(Convert::colourLEDInverse(0, 0, 0)          == 0);      // Should be black
    REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0)       == 0x1F);   // Should be red
    REQUIRE(Convert::colourLEDInverse(0, 0xFF, 0)       == 0x3E0);  // Should be green
    REQUIRE(Convert::colourLEDInverse(0, 0, 0xFF)       == 0x7C00); // Should be blue
    REQUIRE(Convert::colourLEDInverse(0xFF, 0xFF, 0xFF) == 0x7FFF); // Should be white
    REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0xFF)    == 0x7C1F); // Should be red and blue
}

TEST_CASE("Testing the hardware gain conversions to SI units", "[hardware][conversion][gain]") {

    INFO("Testing the forward gain conversions");
    REQUIRE(Convert::gain(254) == Approx(100)); // Should be 100
    REQUIRE(Convert::gain(127) == Approx(50));  // Should be 50
    REQUIRE(Convert::gain(0)   == Approx(0));   // Should be 0

    INFO("Testing the inverse gain conversions");
    REQUIRE(Convert::gainInverse(100)  == 254);     // Should be max
    REQUIRE(Convert::gainInverse(0)    == 0);       // Should be min
    REQUIRE(Convert::gainInverse(50)   == 127);     // Should be the middle
    REQUIRE(Convert::gainInverse(1000) == 254);     // Should cap to max
    REQUIRE(Convert::gainInverse(-10)  == 0);       // Should cap to min
}

TEST_CASE("Testing the hardware position conversions to radians", "[hardware][conversion][position]") {

    FAIL("Write the test");

    INFO("Testing the forward position conversions");
    for(size_t i = 0; i < 20; ++i) {
        REQUIRE(Convert::servoPosition(i, 0) == Approx(0));
        REQUIRE(Convert::servoPosition(i, 1024) == Approx(0));
        REQUIRE(Convert::servoPosition(i, 2048) == Approx(0));
        REQUIRE(Convert::servoPosition(i, 4096) == Approx(0));
    }

    INFO("Testing the inverse position conversions");
    for(size_t i = 0; i < 20; ++i) {
        REQUIRE(Convert::servoPositionInverse(i, 0) == Approx(0));
        REQUIRE(Convert::servoPositionInverse(i, 1024) == Approx(0));
        REQUIRE(Convert::servoPositionInverse(i, 2048) == Approx(0));
        REQUIRE(Convert::servoPositionInverse(i, 4096) == Approx(0));
        // TODO test values that are too large
    }
}

TEST_CASE("Testing the hardware speed conversions to radians/second", "[hardware][conversion][speed]") {

    // TODO test changing the motor sensor speed conversion values

    FAIL("Write the test");
    INFO("Testing the forward speed conversions");
    for(size_t i = 0; i < 20; ++i) {
        // TODO test directions
    }

    INFO("Testing the inverse speed conversions");
    for(size_t i = 0; i < 20; ++i) {
        // TODO test directions
        // TODO test
    }
}

TEST_CASE("Testing the hardware torque limit conversions to between 0 and 1", "[hardware][conversion][torquelimit]") {

    REQUIRE(Convert::torqueLimit(0)  == Approx(0));
    REQUIRE(Convert::torqueLimit(1023) == Approx(100));
}

TEST_CASE("Testing the hardware load conversions to between -1 and 1", "[hardware][conversion][load]") {

    FAIL("Write the test");
}

TEST_CASE("Testing the hardware temperature conversions to SI units", "[hardware][conversion][temperature]") {
    REQUIRE(Convert::temperature(0)  == Approx(0));
    REQUIRE(Convert::temperature(10) == Approx(10));
    REQUIRE(Convert::temperature(20) == Approx(20));
    REQUIRE(Convert::temperature(50) == Approx(50));
}