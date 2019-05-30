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

#ifndef MODULES_PLATFORM_DARWIN_CONVERT_H
#define MODULES_PLATFORM_DARWIN_CONVERT_H

#include <nuclear>

namespace module {
namespace platform {
    namespace darwin {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct Convert {

            /// The value from the Darwin is between 0 and 1023, representing a value between -4g and 4g.
            /// This means 512 = 0
            static constexpr double ACCELEROMETER_CONVERSION_FACTOR = (4 * 9.80665) / 512.0;

            /// The Gyrosocope value from the Darwin is between 0 and 1023, representing a value between -500 and 500
            /// degrees per second. This means 512 = 0
            static constexpr double GYROSCOPE_CONVERSION_FACTOR = (500.0 * (M_PI / 180.0)) / 512.0;

            /// The value that comes from the darwin is measured in decivolts (0.1 of a volt)
            static constexpr double VOLTAGE_CONVERSION_FACTOR = 0.1;

            /// The FSR values that are measured by the darwins feet are measured in millinewtons
            static constexpr double FSR_FORCE_CONVERSION_FACTOR = 0.001;

            /// The gain is a value between 0 and 254, we want a value between 0 and 100
            static constexpr double GAIN_CONVERSION_FACTOR = 100.0 / 254.0;

            /// The angle is given as a value between 0 and 4095
            static constexpr double POSITION_CONVERSION_FACTOR = (2.0 * M_PI) / 4095.0;

            /// The load is measured as a value between 0 and 2047 where the 10th bit specifies direction and 1024 = 0
            /// We convert it to a value between -1 and 1 (percentage)
            static constexpr double LOAD_CONVERSION_FACTOR = 1.0 / 1023.0;

            /// The torque limit is measured as a value between 0 and 1023
            static constexpr double TORQUE_LIMIT_CONVERSION_FACTOR = 100.0 / 1023.0;

            /// The temperatures are given in degrees anyway
            static constexpr double TEMPERATURE_CONVERSION_FACTOR = 1.0;

            /// The MX28 measures its speed between 0 and 1023 where 1023 means a speed of 117.07rpm
            static constexpr double SPEED_CONVERSION_FACTOR = (117.07 * 2.0 * M_PI) / (1023.0 * 60);

            /// Picks which direction a motor should be measured in (forward or reverse) -- configurable based on the
            /// specific humanoid being used.
            static int8_t SERVO_DIRECTION[20];

            /// Offsets the radian angles of motors to change their 0 position -- configurable based on the specific
            /// humanoid being used.
            static double SERVO_OFFSET[20];

            template <int bit>
            static bool getBit(const uint16_t value) {
                return (value & (1 << bit)) == (1 << bit);
            }

            static float accelerometer(const uint16_t value);
            static float gyroscope(const uint16_t value);
            static float voltage(const uint8_t value);
            static float fsrForce(const uint16_t value);
            static float fsrCentre(const bool left, const uint8_t value);

            static std::tuple<uint8_t, uint8_t, uint8_t> colourLED(uint16_t value);
            static uint16_t colourLEDInverse(uint8_t r, uint8_t g, uint8_t b);

            static float gain(const uint8_t value);
            static uint8_t gainInverse(const float value);

            static float servoPosition(const uint8_t id, const uint16_t value);
            static uint16_t servoPositionInverse(const uint8_t id, const float value);

            static float servoSpeed(const uint8_t id, const uint16_t value);
            static uint16_t servoSpeedInverse(const float value);

            static float torqueLimit(const uint16_t value);
            static uint16_t torqueLimitInverse(const float value);

            static float servoLoad(const uint8_t id, const uint16_t value);

            static float temperature(const uint8_t value);
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module
#endif
