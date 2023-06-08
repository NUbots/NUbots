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

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Convert.hpp"

#include "utility/math/angle.hpp"

using namespace module::platform::cm740;
using Catch::Matchers::IsNaN;
using Catch::Matchers::WithinRel;

TEST_CASE("Testing the hardware accelerometer conversions to SI units", "[hardware][conversion][accelerometer]") {

    REQUIRE_THAT(Convert::accelerometer(0), WithinRel(-4.0f * 9.80665f));
    REQUIRE_THAT(Convert::accelerometer(512), WithinRel(0.0f * 9.80665f));
    REQUIRE_THAT(Convert::accelerometer(640), WithinRel(1.0f * 9.80665f));
    REQUIRE_THAT(Convert::accelerometer(1024), WithinRel(4.0f * 9.80665f));
}

TEST_CASE("Testing the hardware gyroscope conversions to SI units", "[hardware][conversion][gyroscope]") {

    REQUIRE_THAT(Convert::gyroscope(0), WithinRel((-1880.0f * float(M_PI)) / 180.0f));
    REQUIRE_THAT(Convert::gyroscope(512), WithinRel((0.0f * float(M_PI)) / 180.0f));
    REQUIRE_THAT(Convert::gyroscope(1024), WithinRel((1880.0f * float(M_PI)) / 180.0f));
}

TEST_CASE("Testing the hardware voltage conversions to SI units", "[hardware][conversion][voltage]") {

    REQUIRE_THAT(Convert::voltage(0), WithinRel(0.0f));
    REQUIRE_THAT(Convert::voltage(120), WithinRel(12.0f));
    REQUIRE_THAT(Convert::voltage(96), WithinRel(9.6f));
}

TEST_CASE("Testing the hardware FSR conversions to SI units", "[hardware][conversion][fsr]") {

    REQUIRE_THAT(Convert::fsrForce(670), WithinRel(0.67f));
    REQUIRE_THAT(Convert::fsrForce(1100), WithinRel(1.1f));
    REQUIRE_THAT(Convert::fsrForce(12345), WithinRel(12.345f));

    INFO("Testing the FSR centre conversions");
    // Test Left Foot
    REQUIRE_THAT(Convert::fsrCentre(true, 0), WithinRel(-1.0f));
    REQUIRE_THAT(Convert::fsrCentre(true, 254), WithinRel(1.0f));
    REQUIRE_THAT(Convert::fsrCentre(true, 127), WithinRel(0.0f));
    REQUIRE_THAT(Convert::fsrCentre(true, 0xFF), IsNaN());
    // Test Right Foot
    REQUIRE_THAT(Convert::fsrCentre(false, 0), WithinRel(1.0f));
    REQUIRE_THAT(Convert::fsrCentre(false, 254), WithinRel(-1.0f));
    REQUIRE_THAT(Convert::fsrCentre(false, 127), WithinRel(0.0f));
    REQUIRE_THAT(Convert::fsrCentre(false, 0xFF), IsNaN());
}

TEST_CASE("Testing the hardware coloured LED conversions to 24bit rgb", "[hardware][conversion][led]") {

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the forward coloured LED conversions");
        REQUIRE(Convert::colourLED(0) == std::make_tuple(0, 0, 0));
        REQUIRE(Convert::colourLED(0x1F) == std::make_tuple(0xF8, 0, 0));
        REQUIRE(Convert::colourLED(0x3E0) == std::make_tuple(0, 0xF8, 0));
        REQUIRE(Convert::colourLED(0x7C00) == std::make_tuple(0, 0, 0xF8));
        REQUIRE(Convert::colourLED(0x7FFF) == std::make_tuple(0xF8, 0xF8, 0xF8));
        REQUIRE(Convert::colourLED(0x7C1F) == std::make_tuple(0xF8, 0, 0xF8));
    }

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the inverse coloured LED conversions");
        REQUIRE(Convert::colourLEDInverse(0, 0, 0) == 0);
        REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0) == 0x1F);
        REQUIRE(Convert::colourLEDInverse(0, 0xFF, 0) == 0x3E0);
        REQUIRE(Convert::colourLEDInverse(0, 0, 0xFF) == 0x7C00);
        REQUIRE(Convert::colourLEDInverse(0xFF, 0xFF, 0xFF) == 0x7FFF);
        REQUIRE(Convert::colourLEDInverse(0xFF, 0, 0xFF) == 0x7C1F);
    }
}

TEST_CASE("Testing the hardware gain conversions to SI units", "[hardware][conversion][gain]") {

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the forward gain conversions");
        REQUIRE_THAT(Convert::gain(254), WithinRel(100.0f));
        REQUIRE_THAT(Convert::gain(127), WithinRel(50.0f));
        REQUIRE_THAT(Convert::gain(0), WithinRel(0.0f));
    }

    // This scope gets rid of the INFO messages once we pass this section
    {
        INFO("Testing the inverse gain conversions");
        REQUIRE(Convert::gainInverse(100) == 254);
        REQUIRE(Convert::gainInverse(0) == 0);
        REQUIRE(Convert::gainInverse(50) == 127);
        REQUIRE(Convert::gainInverse(1000) == 254);
        REQUIRE(Convert::gainInverse(-10) == 0);
    }
}

TEST_CASE("Testing the hardware position conversions to radians", "[hardware][conversion][position]") {

    // We allow it to be within 2 steps (as it could step the opposite directions on conversion)
    const double maxForwardError = ((2 * M_PI) / 4095.0) * 2;
    // On converting back it should never stray more then 1 value
    const double maxInverseError = 1;

    std::vector<std::vector<std::pair<float, uint16_t>>> inverseTests;

    // Load in servo offsets and directions
    constexpr std::array<int8_t, 20> direction = {-1, 1,  -1, -1, -1, 1, -1, -1, -1, -1,
                                                  1,  -1, 1,  -1, -1, 1, 1,  1,  1,  1};
    constexpr std::array<double, 20> offset    = {M_PI / 2, M_PI / 2, -M_PI / 4, M_PI / 4, -M_PI / 2, -M_PI / 2, 0.0,
                                               0.0,      0.0,      0.0,       0.0,      0.0,       0.0,       0.0,
                                               0.0,      0.0,      0.0,       0.0,      0.0,       0.0};

    for (size_t i = 0; i < 20; ++i) {
        Convert::SERVO_DIRECTION[i] = direction[i];
        Convert::SERVO_OFFSET[i]    = offset[i];
    }

    // This scope gets rid of the old INFO messages once we pass this section
    {
        INFO("Testing the forward position conversions");

        const std::pair<uint16_t, float> forwardTests[] = {{0, M_PI},
                                                           {1023, -M_PI_2},
                                                           {2048, 0.0},
                                                           {3073, M_PI_2},
                                                           {4095, M_PI}};

        for (size_t i = 0; i < 20; ++i) {
            INFO("Testing forward motor " << i);

            std::vector<std::pair<float, uint16_t>> inverseTest;

            for (auto& test : forwardTests) {
                INFO("Input: " << test.first << " Expected output: " << test.second);
                float expected = utility::math::angle::normalizeAngle(test.second * Convert::SERVO_DIRECTION[i]
                                                                      + Convert::SERVO_OFFSET[i]);
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

        const std::pair<uint16_t, float> tests[] = {{0, 0.0}, {1023, 1.0}, {1024, 0.0}, {2047, -1.0}};

        for (size_t i = 0; i < 20; ++i) {
            INFO("Testing forward motor " << i);

            // Test with Servos
            INFO("Testing with Servos");
            for (auto& test : tests) {
                float expected = test.second * (Convert::SPEED_CONVERSION_FACTOR * 1023) * Convert::SERVO_DIRECTION[i];
                float actual   = Convert::servoSpeed(i, test.first);

                INFO("Input: " << test.first << " Expected: " << test.second << " Actual: " << actual);

                REQUIRE_THAT(expected, WithinRel(actual));
            }

            // Test our inverse case
            INFO("Testing inverse operations");
            for (auto& test : tests) {
                uint16_t expected = test.first % 1024;
                uint16_t actual =
                    Convert::servoSpeedInverse(fabs(test.second * (Convert::SPEED_CONVERSION_FACTOR * 1023)));

                INFO("Input: " << fabs(test.second * (Convert::SPEED_CONVERSION_FACTOR * 1023))
                               << " Expected: " << test.first << " Actual: " << actual);

                // These should be equal
                REQUIRE(expected == actual);
            }
        }
    }
}

TEST_CASE("Testing the hardware torque limit conversions to between 0 and 100", "[hardware][conversion][torquelimit]") {

    REQUIRE_THAT(Convert::torqueLimit(0), WithinRel(0.0f));
    REQUIRE_THAT(Convert::torqueLimit(1023), WithinRel(100.0f));
}

TEST_CASE("Testing the hardware load conversions to between -100 and 100", "[hardware][conversion][load]") {

    for (int i = 0; i < 20; ++i) {
        REQUIRE_THAT(Convert::servoLoad(i, 0), WithinRel(0.0f * Convert::SERVO_DIRECTION[i]));
        REQUIRE_THAT(Convert::servoLoad(i, 1024), WithinRel(0.0f * Convert::SERVO_DIRECTION[i]));
        REQUIRE_THAT(Convert::servoLoad(i, 2047), WithinRel(-1.0f * Convert::SERVO_DIRECTION[i]));
        REQUIRE_THAT(Convert::servoLoad(i, 1023), WithinRel(1.0f * Convert::SERVO_DIRECTION[i]));
    }
}

TEST_CASE("Testing the hardware temperature conversions to SI units", "[hardware][conversion][temperature]") {
    REQUIRE_THAT(Convert::temperature(0), WithinRel(0.0f));
    REQUIRE_THAT(Convert::temperature(10), WithinRel(10.0f));
    REQUIRE_THAT(Convert::temperature(20), WithinRel(20.0f));
    REQUIRE_THAT(Convert::temperature(50), WithinRel(50.0f));
}
