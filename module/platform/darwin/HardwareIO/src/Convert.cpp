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

#include "Convert.h"

#include "utility/math/angle.h"

namespace module {
namespace platform {
    namespace darwin {

        int8_t Convert::SERVO_DIRECTION[20] = {0};
        double Convert::SERVO_OFFSET[20]    = {0};

        float Convert::accelerometer(uint16_t value) {
            return (value - 512) * ACCELEROMETER_CONVERSION_FACTOR;
        }

        float Convert::gyroscope(uint16_t value) {
            return (value - 512) * GYROSCOPE_CONVERSION_FACTOR;
        }

        float Convert::voltage(const uint8_t value) {
            return value * VOLTAGE_CONVERSION_FACTOR;
        }

        float Convert::fsrForce(const uint16_t value) {
            return value * FSR_FORCE_CONVERSION_FACTOR;
        }

        float Convert::fsrCentre(const bool left, const uint8_t value) {
            if (value == 0xFF) {
                // Return NaN if there is no centre
                return std::numeric_limits<float>::quiet_NaN();
            }
            // Flips right foot coordinates to match robot coords
            // See:
            // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
            else if (left) {
                // This normalises the value between -1 and 1
                return double(value - 127) / 127.0;
            }
            else {
                // This normalises the value between -1 and 1
                return double(127 - value) / 127.0;
            }
        }

        std::tuple<uint8_t, uint8_t, uint8_t> Convert::colourLED(uint16_t value) {
            return std::make_tuple(static_cast<uint8_t>((value & 0x001F) << 3),
                                   static_cast<uint8_t>((value & 0x03E0) >> 2),
                                   static_cast<uint8_t>((value & 0x7C00) >> 7));
        }

        uint16_t Convert::colourLEDInverse(uint8_t r, uint8_t g, uint8_t b) {
            return ((r >> 3)) | ((g >> 3) << 5) | ((b >> 3) << 10);
        }

        float Convert::gain(const uint8_t value) {
            return value * GAIN_CONVERSION_FACTOR;
        }

        uint8_t Convert::gainInverse(const float value) {
            return value >= 100
                       ? 254            // If we are greater then 100, then set to 100
                       : value < 0 ? 0  // If we are less then 0, then set to 0
                                   : std::round(value / GAIN_CONVERSION_FACTOR);  // Otherwise do our conversion
        }

        float Convert::servoPosition(const uint8_t id, const uint16_t value) {
            // offset and normalize the angle
            return utility::math::angle::normalizeAngle(
                (((value - 2048) * POSITION_CONVERSION_FACTOR) * SERVO_DIRECTION[id]) + SERVO_OFFSET[id]);
        }

        uint16_t Convert::servoPositionInverse(const uint8_t id, const float value) {
            float angle = value;

            // Undo our conversion operations
            angle -= SERVO_OFFSET[id];
            angle *= SERVO_DIRECTION[id];

            // Normalize the angle
            angle = utility::math::angle::normalizeAngle(angle);

            // Convert it back
            return (angle / POSITION_CONVERSION_FACTOR) + 2048;
        }

        float Convert::servoSpeed(const uint8_t id, const uint16_t value) {

            // We only care about the lower bits
            float raw = (value & 0x3FF) * SPEED_CONVERSION_FACTOR;

            // If bit 10 is set we are moving Clockwise
            raw *= value & 0x400 ? -1 : 1;

            // Go the correct direction
            raw *= SERVO_DIRECTION[id];

            return raw;
        }
        uint16_t Convert::servoSpeedInverse(const float value) {
            // If the value is greater then 1023, then set to max speed (0)
            return value > 100 ? 0 : uint16_t(round(value / SPEED_CONVERSION_FACTOR));
        }

        float Convert::torqueLimit(const uint16_t value) {
            return value * TORQUE_LIMIT_CONVERSION_FACTOR;
        }

        uint16_t Convert::torqueLimitInverse(const float value) {
            return value >= 100 ? 1023.0 : std::round(value / TORQUE_LIMIT_CONVERSION_FACTOR);
        }

        float Convert::servoLoad(const uint8_t id, const uint16_t value) {
            // We only care about the lower bits if bit 10 is set then we are moving clockwise
            float raw = (value & 0x3FF) * (value & 0x400 ? -1 : 1);
            raw *= LOAD_CONVERSION_FACTOR;

            // Go the correct direction
            return raw * SERVO_DIRECTION[id];
        }

        float Convert::temperature(const uint8_t value) {
            return value * TEMPERATURE_CONVERSION_FACTOR;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
