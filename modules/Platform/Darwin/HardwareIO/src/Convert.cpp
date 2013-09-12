/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "Convert.h"
#include "utility/math/angle.h"

namespace modules {
namespace Platform {
namespace Darwin {

    float Convert::accelerometer(uint16_t value) {
        return (value - 512) * ACCELERONOMETER_CONVERSION_FACTOR;
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

    float Convert::fsrCentre(const bool left, const bool x, const uint8_t value) {
        if(value == 0xFF) {
            // Return NaN if there is no centre
            return std::numeric_limits<float>::quiet_NaN();
        }
        // On the right foot, the X is the correct way around while the Y is flipped
        // On the left foot, the X is flipped and the Y is the correct way around
        // Because of this, we can implement the resulting logic using an xor
        else if(left ^ x) {
            return double(value - 127) / 127.0;
        }
        else {
            return double(127 - value) / 127.0;
        }
    }

    std::tuple<uint8_t, uint8_t, uint8_t> Convert::colourLED(uint16_t value) {
        return std::make_tuple(static_cast<uint8_t>((value & 0x001F) << 3),
                               static_cast<uint8_t>((value & 0x03E0) >> 2),
                               static_cast<uint8_t>((value & 0x7C00) >> 7));
    }

    uint16_t Convert::colourLEDInverse(uint8_t r, uint8_t g, uint8_t b) {
        return ((r >> 3))
             | ((g >> 3) << 5)
             | ((b >> 3) << 10);
    }

    float Convert::gain(const uint8_t value) {
        return value * GAIN_CONVERSION_FACTOR;
    }

    uint8_t Convert::gainInverse(const float value) {
        return value >= 100 ? 254 // If we are greater then 100, then set to 100
                : value < 0 ? 0  // If we are less then 0, then set to 0
                : std::round(value / GAIN_CONVERSION_FACTOR); // Otherwise do our conversion
    }

    float Convert::servoPosition(const uint8_t id, const uint16_t value) {
        float raw = (value * POSITION_CONVERSION_FACTOR);
        raw += SERVO_OFFSET[id];
        raw *= SERVO_DIRECTION[id];

        // Normalize the angle
        return utility::math::angle::normalizeAngle(raw);
    }

    uint16_t Convert::servoPositionInverse(const uint8_t id, const float value) {
        // Normalize the angle
        float angle = utility::math::angle::normalizeAngle(value);

        // Undo our conversion operations
        angle *= SERVO_DIRECTION[id];
        angle -= SERVO_OFFSET[id];
        return angle / POSITION_CONVERSION_FACTOR;
    }

    float Convert::speed(const uint8_t id, const uint16_t value) {

        // We only care about the lower bits
        float raw = (value & 0x3FF) * SPEED_CONVERSION_FACTOR[id];

        // If bit 10 is set we are moving Clockwise
        raw *= value & 0x400 ? -1 : 1;

        // Go the correct direction
        raw *= SERVO_DIRECTION[id];

        return raw;
    }
    uint16_t Convert::speedInverse(const uint8_t id, const float value) {
        uint16_t raw = (value * SERVO_DIRECTION[id]) / SPEED_CONVERSION_FACTOR[id];

        // If the value is greater then 1023, then set to max speed (0)
        return raw > 1023 ? 0 : raw;
    }

    float Convert::torqueLimit(const uint16_t value) {
        return value * TORQUE_LIMIT_CONVERSION_FACTOR;
    }

    float Convert::load(const uint8_t id, const uint8_t value) {
         // We only care about the lower bits if bit 10 is set then we are moving clockwise
        float raw = (value & 0x3FF) * (value & 0x400 ? -1 : 1);
        raw *= LOAD_CONVERSION_FACTOR;

        // Go the correct direction
        return raw * SERVO_DIRECTION[id];;
    }

    float Convert::temperature(const uint8_t value) {
        return value * TEMPERATURE_CONVERSION_FACTOR;
    }
}
}
}