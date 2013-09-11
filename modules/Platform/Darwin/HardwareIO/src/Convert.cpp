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

    /// The value from the Darwin is between 0 and 1023, representing a value between -4g and 4g.
    /// This means 512 = 0
    constexpr double ACCELERONOMETER_CONVERSION_FACTOR = (4 * 9.80665) / 512.0;

    /// The Gyrosocope value from the Darwin is between 0 and 1023, representing a value between -1600 and 1600
    /// degrees per second. This means 512 = 0
    constexpr double GYROSCOPE_CONVERSION_FACTOR = (1600 * (M_PI / 180.0)) / 512;

    /// The value that comes from the darwin is measured in decivolts (0.1 of a volt)
    constexpr double VOLTAGE_CONVERSION_FACTOR = 0.1;

    /// The FSR values that are measured by the darwins feet are measured in millinewtons
    constexpr double FSR_FORCE_CONVERSION_FACTOR = 0.001;

    /// The gain is a value betwen 0 and 254, we want a value between 0 and 100
    constexpr double GAIN_CONVERSION_FACTOR = 100 / 254;

    /// The angle is given as a value between 0 and 4095
    constexpr double POSITION_CONVERSION_FACTOR = (2 * M_PI) / 4095;

    /// The load is measured as a value between 0 and 2047 where the 10th bit specifies direction and 1024 = 0
    /// We convert it to a value between -100 and 100 (percentage)
    constexpr double LOAD_CONVERSION_FACTOR = 100 / 1023;

    /// The torque limit is measured as a value between 0 and 1023
    constexpr double TORQUE_LIMIT_CONVERSION_FACTOR = 100 / 1023;

    /// The temperatures are given in degrees anyway
    constexpr double TEMPERATURE_CONVERSION_FACTOR = 1;

    /// The MX28 measures its speed between 0 and 1023 where 1023 means a speed of 117.07rpm
    constexpr double MX28_SPEED_CONVERSION_FACTOR = (117.07 * 2 * M_PI) / 1023;
    /// The RX28 measures its speed between 0 and 1023 where 1023 means a speed of 54rpm
    constexpr double RX28_SPEED_CONVERSION_FACTOR = (54 * 2 * M_PI) / 1023;

    /// Picks which direction a motor should be measured in (forward or reverse)
    constexpr const int8_t SERVO_DIRECTION[20] = {
        -1,             // [0]  R_SHOULDER_PITCH
        1,              // [1]  L_SHOULDER_PITCH
        -1,             // [2]  R_SHOULDER_ROLL
        -1,             // [3]  L_SHOULDER_ROLL
        -1,             // [4]  R_ELBOW
        1,              // [5]  L_ELBOW
        -1,             // [6]  R_HIP_YAW
        -1,             // [7]  L_HIP_YAW
        -1,             // [8]  R_HIP_ROLL
        -1,             // [9]  L_HIP_ROLL
        1,              // [10] R_HIP_PITCH
        -1,             // [11] L_HIP_PITCH
        1,              // [12] R_KNEE
        -1,             // [13] L_KNEE
        -1,             // [14] R_ANKLE_PITCH
        1,              // [15] L_ANKLE_PITCH
        1,              // [16] R_ANKLE_ROLL
        1,              // [17] L_ANKLE_ROLL
        1,              // [18]  HEAD_YAW
        -1,             // [19]  HEAD_PITCH
    };

    /// Offsets the radian angles of motors to change their 0 position
    constexpr const float SERVO_OFFSET[20] = {
        -1.5707963f,    // [0]  R_SHOULDER_PITCH
        1.5707963f,     // [1]  L_SHOULDER_PITCH
        0.7853981f,     // [2]  R_SHOULDER_ROLL
        -0.7853981f,    // [3]  L_SHOULDER_ROLL
        1.5707963,      // [4]  R_ELBOW
        -1.5707963f,    // [5]  L_ELBOW
        0.0f,           // [6]  R_HIP_YAW
        0.0f,           // [7]  L_HIP_YAW
        0.0f,           // [8]  R_HIP_ROLL
        0.0f,           // [9]  L_HIP_ROLL
        0.0f,           // [10] R_HIP_PITCH
        0.0f,           // [11] L_HIP_PITCH
        0.0f,           // [12] R_KNEE
        0.0f,           // [13] L_KNEE
        0.0f,           // [14] R_ANKLE_PITCH
        0.0f,           // [15] L_ANKLE_PITCH
        0.0f,           // [16] R_ANKLE_ROLL
        0.0f,           // [17] L_ANKLE_ROLL
        0.0f,           // [18] HEAD_YAW
        -0.631,         // [19] HEAD_PITCH
    };

    /**
     * The speed conversion factor cannot be const, as at startup we have to work out if each motor is an
     * RX28 or an MX28. Both motors have different speed conversion factors, although we default them all
     * to the MX28 as that is the most common motor (and what old motors will be replaced with)
     */
    double SPEED_CONVERSION_FACTOR[20] = {
        MX28_SPEED_CONVERSION_FACTOR,   // [0]  R_SHOULDER_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [1]  L_SHOULDER_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [2]  R_SHOULDER_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [3]  L_SHOULDER_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [4]  R_ELBOW
        MX28_SPEED_CONVERSION_FACTOR,   // [5]  L_ELBOW
        MX28_SPEED_CONVERSION_FACTOR,   // [6]  R_HIP_YAW
        MX28_SPEED_CONVERSION_FACTOR,   // [7]  L_HIP_YAW
        MX28_SPEED_CONVERSION_FACTOR,   // [8]  R_HIP_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [9]  L_HIP_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [10] R_HIP_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [11] L_HIP_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [12] R_KNEE
        MX28_SPEED_CONVERSION_FACTOR,   // [13] L_KNEE
        MX28_SPEED_CONVERSION_FACTOR,   // [14] R_ANKLE_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [15] L_ANKLE_PITCH
        MX28_SPEED_CONVERSION_FACTOR,   // [16] R_ANKLE_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [17] L_ANKLE_ROLL
        MX28_SPEED_CONVERSION_FACTOR,   // [18] HEAD_YAW
        MX28_SPEED_CONVERSION_FACTOR,   // [19] HEAD_PITCH
    };

    float Convert::acceleronometer(uint16_t value) {
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
        // This implements an XOR (since those are the cases where the value is flipped)
        else if(left ^ x) {
            return (126 - value) / 127;
        }
        else {
            return (value - 128) / 127;
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
        return value > 100 ? 254 // If we are greater then 100, then set to 100
                : value < 0 ? 0  // If we are less then 0, then set to 0
                : value / GAIN_CONVERSION_FACTOR; // Otherwise do our conversion
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