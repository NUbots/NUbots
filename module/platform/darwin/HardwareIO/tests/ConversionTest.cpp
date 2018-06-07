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
#include "Convert.h"
#include "utility/math/angle.h"

using namespace module::platform::darwin;

TEST_CASE("Testing the hardware accelerometer conversions to SI units", "[hardware][conversion][accelerometer]") {

    REQUIRE(Convert::accelerometer(0) == Approx(-4.0 * 9.80665));    // Should be -4g
    REQUIRE(Convert::accelerometer(512) == Approx(0.0));             // Should be 0
    REQUIRE(Convert::accelerometer(640) == Approx(9.80665));         // Should be +1g
    REQUIRE(Convert::accelerometer(1024) == Approx(4.0 * 9.80665));  // Should be +4g
}

TEST_CASE("Testing the hardware gyroscope conversions to SI units", "[hardware][conversion][gyroscope]") {

    REQUIRE(Convert::gyroscope(0)
            == Approx((-1800.0 * M_PI) / 180.0));   // Should be -1800 degrees/second in radians/second
    REQUIRE(Convert::gyroscope(512) == Approx(0));  // Should be 0
    REQUIRE(Convert::gyroscope(1024)
            == Approx((1800.0 * M_PI) / 180.0));  // Should be 1800 degrees/second in radians/second
}

TEST_CASE("Testing the hardware voltage conversions to SI units", "[hardware][conversion][voltage]") {

    REQUIRE(Convert::voltage(0) == Approx(0));     // Should be 0 volts
    REQUIRE(Convert::voltage(120) == Approx(12));  // Should be 12 volts
    REQUIRE(Convert::voltage(96) == Approx(9.6));  // Should be 9.6 volts
}

TEST_CASE("Testing the hardware FSR conversions to SI units", "[hardware][conversion][fsr]") {

    REQUIRE(Convert::fsrForce(670) == Approx(0.67));      // Should be 0.67 newtons
    REQUIRE(Convert::fsrForce(1100) == Approx(1.1));      // Should be 1.1 newtons
    REQUIRE(Convert::fsrForce(12345) == Approx(12.345));  // Should be 12.345 newtons

    INFO("Testing the FSR centre conversions");
    // Test Left Foot
    // Test X
    REQUIRE(Convert::fsrCentre(true, true, 0) == Approx(1));     // Should be 1
    REQUIRE(Convert::fsrCentre(true, true, 254) == Approx(-1));  // Should be -1
    REQUIRE(Convert::fsrCentre(true, true, 127) == Approx(0));   // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, true, 0xFF)));   // Should be NaN
    // Test Y
    REQUIRE(Convert::fsrCentre(true, false, 0) == Approx(-1));   // Should be -1
    REQUIRE(Convert::fsrCentre(true, false, 254) == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(true, false, 127) == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, false, 0xFF)));  // Should be NaN
    // Test Right Foot
    // Test X
    REQUIRE(Convert::fsrCentre(false, true, 0) == Approx(-1));   // Should be -1
    REQUIRE(Convert::fsrCentre(false, true, 254) == Approx(1));  // Should be 1
    REQUIRE(Convert::fsrCentre(false, true, 127) == Approx(0));  // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, true, 0xFF)));   // Should be NaN
    // Test Y
    REQUIRE(Convert::fsrCentre(false, false, 0) == Approx(1));     // Should be 1
    REQUIRE(Convert::fsrCentre(false, false, 254) == Approx(-1));  // Should be -1
    REQUIRE(Convert::fsrCentre(false, false, 127) == Approx(0));   // Should be 0
    REQUIRE(std::isnan(Convert::fsrCentre(true, false, 0xFF)));    // Should be NaN
}

TEST_CASE("Testing the hardware coloured LED conversions to 24bit rgb", "[hardware][conversion][led]") {

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the forward coloured LED conversions");
        REQUIRE(Convert::colourLED(0) == std::make_tuple(0, 0, 0));                // Should be black
        REQUIRE(Convert::colourLED(0x1F) == std::make_tuple(0xF8, 0, 0));          // Should be red
        REQUIRE(Convert::colourLED(0x3E0) == std::make_tuple(0, 0xF8, 0));         // Should be green
        REQUIRE(Convert::colourLED(0x7C00) == std::make_tuple(0, 0, 0xF8));        // Should be blue
        REQUIRE(Convert::colourLED(0x7FFF) == std::make_tuple(0xF8, 0xF8, 0xF8));  // Should be white
        REQUIRE(Convert::colourLED(0x7C1F) == std::make_tuple(0xF8, 0, 0xF8));     // Should be red and blue
    }

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the inverse coloured LED conversions");
        REQUIRE(Convert::colourLEDInverse(0, 0, 0) == 0);                // Should be black
        REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0) == 0x1F);          // Should be red
        REQUIRE(Convert::colourLEDInverse(0, 0xFF, 0) == 0x3E0);         // Should be green
        REQUIRE(Convert::colourLEDInverse(0, 0, 0xFF) == 0x7C00);        // Should be blue
        REQUIRE(Convert::colourLEDInverse(0xFF, 0xFF, 0xFF) == 0x7FFF);  // Should be white
        REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0xFF) == 0x7C1F);     // Should be red and blue
    }
}

TEST_CASE("Testing the hardware gain conversions to SI units", "[hardware][conversion][gain]") {

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the forward gain conversions");
        REQUIRE(Convert::gain(254) == Approx(100));  // Should be 100
        REQUIRE(Convert::gain(127) == Approx(50));   // Should be 50
        REQUIRE(Convert::gain(0) == Approx(0));      // Should be 0
    }

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the inverse gain conversions");
        REQUIRE(Convert::gainInverse(100) == 254);   // Should be max
        REQUIRE(Convert::gainInverse(0) == 0);       // Should be min
        REQUIRE(Convert::gainInverse(50) == 127);    // Should be the middle
        REQUIRE(Convert::gainInverse(1000) == 254);  // Should cap to max
        REQUIRE(Convert::gainInverse(-10) == 0);     // Should cap to min
    }
}

TEST_CASE("Testing the hardware position conversions to radians", "[hardware][conversion][position]") {

    // We allow it to be within 2 steps (as it could step the opposite directions on conversion)
    const double maxForwardError = ((2 * M_PI) / 4095.0) * 2;
    // On converting back it should never stray more then 1 value
    const double maxInverseError = 1;

    std::vector<std::vector<std::pair<float, uint16_t>>> inverseTests;

    // This scope gets rid of the old INFO messages once we pass this section
    {
        INFO("Testing the forward position conversions");

        const std::pair<uint16_t, float> forwardTests[] = {
            {0, -M_PI}, {1023, -M_PI_2}, {2048, 0}, {3073, M_PI_2}, {4095, M_PI}};

        for (size_t i = 0; i < 20; ++i) {
            INFO("Testing forward motor " << i);

            std::vector<std::pair<float, uint16_t>> inverseTest;

            for (auto& test : forwardTests) {
                float expected = utility::math::angle::normalizeAngle((test.second + Convert::SERVO_OFFSET[i])
                                                                      * Convert::SERVO_DIRECTION[i]);
                float actual   = Convert::servoPosition(i, test.first);

                INFO("Expected: " << expected << " Actual: " << actual);

                inverseTest.push_back({actual, test.first});

                // Test that the error is within 1 radian unit
                REQUIRE(utility::math::angle::difference(expected, actual) <= maxForwardError);
            }

            inverseTests.push_back(std::move(inverseTest));
        }
    }

    // This scope gets rid of the old INFO messages once we pass this section
    {
        INFO("Testing the inverse position conversions");

        for (size_t i = 0; i < 20; ++i) {

            INFO("Testing inverse motor " << i);

            for (auto& test : inverseTests[i]) {
                int distance, expected, actual;

                actual = Convert::servoPositionInverse(i, test.first);
                INFO("Testing Input:" << test.first << " Expected: " << test.second << " Actual: " << actual);
                expected = test.second;
                distance = (((actual - expected) + 2048) % 4095) - 2048;
                REQUIRE(distance <= maxInverseError);

                actual = Convert::servoPositionInverse(i, test.first + 2 * M_PI);
                INFO("Testing Input:" << test.first - 2 * M_PI << " Expected: " << test.second
                                      << " Actual: " << actual);
                expected = test.second;
                distance = (((actual - expected) + 2048) % 4095) - 2048;
                REQUIRE(distance <= maxInverseError);

                actual = Convert::servoPositionInverse(i, test.first - 2 * M_PI);
                INFO("Testing Input:" << test.first + 2 * M_PI << " Expected: " << test.second
                                      << " Actual: " << actual);
                expected = test.second;
                distance = (((actual - expected) + 2048) % 4095) - 2048;
                REQUIRE(distance <= maxInverseError);

                actual = Convert::servoPositionInverse(i, test.first + M_PI);
                INFO("Testing Input:" << test.first + M_PI << " Expected: " << test.second - 2048
                                      << " Actual: " << actual);
                expected = test.second - 2048;
                distance = (((actual - expected) + 2048) % 4095) - 2048;
                REQUIRE(distance <= maxInverseError);

                actual = Convert::servoPositionInverse(i, test.first - M_PI);
                INFO("Testing Input:" << test.first - M_PI << " Expected: " << test.second - 2048
                                      << " Actual: " << actual);
                expected = test.second - 2048;
                distance = (((actual - expected) + 2048) % 4095) - 2048;
                REQUIRE(distance <= maxInverseError);
            }
        }
    }
}

TEST_CASE("Testing the hardware speed conversions to radians/second", "[hardware][conversion][speed]") {

    // This scope gets rid of the old INFO messages once we pass this section
    {
        INFO("Testing the forward position conversions");

        const std::pair<uint16_t, float> tests[] = {{0, 0}, {1023, 1.0}, {1024, 0}, {2047, -1.0}};

        for (size_t i = 0; i < 20; ++i) {
            INFO("Testing forward motor " << i);

            // Test with MX28s
            INFO("Testing with MX28s");
            for (auto& test : tests) {
                float expected =
                    test.second * (Convert::MX28_SPEED_CONVERSION_FACTOR * 1023) * Convert::SERVO_DIRECTION[i];
                float actual = Convert::servoSpeed(i, test.first);

                INFO("Input: " << test.first << " Expected: " << test.second << " Actual: " << actual);

                REQUIRE(expected == Approx(actual));
            }

            // Test with RX28s it fails
            INFO("Testing with RX28s");
            for (auto& test : tests) {
                float expected =
                    test.second * (Convert::RX28_SPEED_CONVERSION_FACTOR * 1023) * Convert::SERVO_DIRECTION[i];
                float actual = Convert::servoSpeed(i, test.first);

                INFO("Input: " << test.first << " Expected: " << test.second << " Actual: " << actual);

                // Test that if they're not both equal to 0, they are unequal
                if ((expected + actual) != 0) {
                    REQUIRE(expected != Approx(actual));
                }
            }

            // Change this motor to be an RX28
            Convert::SPEED_CONVERSION_FACTOR[i] = Convert::RX28_SPEED_CONVERSION_FACTOR;

            // Check with an RX28 it now succeeds
            INFO("Testing with converted RX28s");
            for (auto& test : tests) {
                float expected =
                    test.second * (Convert::RX28_SPEED_CONVERSION_FACTOR * 1023) * Convert::SERVO_DIRECTION[i];
                float actual = Convert::servoSpeed(i, test.first);

                INFO("Input: " << test.first << " Expected: " << test.second << " Actual: " << actual);

                // It should now succeed
                REQUIRE(expected == Approx(actual));
            }

            // Test our inverse case
            INFO("Testing inverse operations");
            for (auto& test : tests) {
                uint16_t expected = test.first % 1024;
                uint16_t actual =
                    Convert::servoSpeedInverse(i, fabs(test.second * (Convert::RX28_SPEED_CONVERSION_FACTOR * 1023)));

                INFO("Input: " << fabs(test.second * (Convert::RX28_SPEED_CONVERSION_FACTOR * 1023))
                               << " Expected: " << test.first << " Actual: " << actual);

                // These should be equal
                REQUIRE(expected == actual);

                // Test that going over the max speed makes it go to 0
                REQUIRE(Convert::servoSpeedInverse(i, Convert::RX28_SPEED_CONVERSION_FACTOR * 1024) == 0);
            }
        }
    }
}

TEST_CASE("Testing the hardware torque limit conversions to between 0 and 100", "[hardware][conversion][torquelimit]") {

    REQUIRE(Convert::torqueLimit(0) == Approx(0));
    REQUIRE(Convert::torqueLimit(1023) == Approx(100));
}

TEST_CASE("Testing the hardware load conversions to between -100 and 100", "[hardware][conversion][load]") {

    for (int i = 0; i < 20; ++i) {
        REQUIRE(Convert::servoLoad(i, 0) == Approx(0 * Convert::SERVO_DIRECTION[i]));
        REQUIRE(Convert::servoLoad(i, 1024) == Approx(0 * Convert::SERVO_DIRECTION[i]));
        REQUIRE(Convert::servoLoad(i, 2047) == Approx(-100 * Convert::SERVO_DIRECTION[i]));
        REQUIRE(Convert::servoLoad(i, 1023) == Approx(100 * Convert::SERVO_DIRECTION[i]));
    }
}

TEST_CASE("Testing the hardware temperature conversions to SI units", "[hardware][conversion][temperature]") {
    REQUIRE(Convert::temperature(0) == Approx(0));
    REQUIRE(Convert::temperature(10) == Approx(10));
    REQUIRE(Convert::temperature(20) == Approx(20));
    REQUIRE(Convert::temperature(50) == Approx(50));
}
