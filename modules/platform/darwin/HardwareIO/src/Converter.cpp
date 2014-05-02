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

#include "Converter.h"

#include "utility/math/angle.h"

namespace modules {
    namespace platform {
        namespace darwin {

            using Darwin::BulkReadResults;
            using messages::platform::darwin::DarwinSensors;
            using messages::motion::ServoTarget;
            using Darwin::Types::ServoValues;

            DarwinSensors Converter::convert(const BulkReadResults& data) {

                DarwinSensors sensors;

                // Timestamp when our data was taken
                sensors.timestamp = NUClear::clock::now();

                /*
                 CM730 Data
                 */

                // Read our Error code
                sensors.cm730ErrorFlags = data.cm730ErrorCode == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.cm730ErrorCode);

                // LED Panel - Unpack the bits
                sensors.ledPanel.led2 = data.cm730.ledPanel & (1 << 1) == (1 << 1);
                sensors.ledPanel.led3 = data.cm730.ledPanel & (1 << 2) == (1 << 2);
                sensors.ledPanel.led4 = data.cm730.ledPanel & (1 << 3) == (1 << 3);

                // Head LED - Extract the colours
                sensors.headLED.r = (data.cm730.headLED & 0x001F) << 3;
                sensors.headLED.g = (data.cm730.headLED & 0x03E0) >> 2;
                sensors.headLED.b = (data.cm730.headLED & 0x7C00) >> 7;

                // Eye LED - Extract the colours
                sensors.eyeLED.r = (data.cm730.eyeLED & 0x001F) << 3;
                sensors.eyeLED.g = (data.cm730.eyeLED & 0x03E0) >> 2;
                sensors.eyeLED.b = (data.cm730.eyeLED & 0x7C00) >> 7;

                // Buttons - Extract the bits
                sensors.buttons.left = data.cm730.buttons & (1 << 1) == (1 << 1);
                sensors.buttons.middle = data.cm730.buttons & (1 << 2) == (1 << 2);

                // Voltage (in volts) - Times by the conversion factor
                sensors.voltage = data.cm730 * VOLTAGE_CONVERSION_FACTOR;

                // Accelerometer (in m/s^2) - Times by the conversion factors
                sensors.accelerometer.x = data.cm730.accelerometer.x * ACCELEROMETER_CONVERSION_FACTOR_X;
                sensors.accelerometer.y = data.cm730.accelerometer.y * ACCELEROMETER_CONVERSION_FACTOR_Y;
                sensors.accelerometer.z = data.cm730.accelerometer.z * ACCELEROMETER_CONVERSION_FACTOR_Z;

                // Gyroscope (in radians/second) - Times by the conversion factors
                sensors.gyroscope.x = data.cm730.gyroscope.x * GYROSCOPE_CONVERSION_FACTOR_X;
                sensors.gyroscope.y = data.cm730.gyroscope.y * GYROSCOPE_CONVERSION_FACTOR_Y;
                sensors.gyroscope.z = data.cm730.gyroscope.z * GYROSCOPE_CONVERSION_FACTOR_Z;

                /*
                 Force Sensitive Resistor Data
                 */

                // Right Sensor
                // Error
                sensors.fsr.right.errorFlags = data.fsrErrorCodes[0] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.fsrErrorCodes[0]);

                // Sensors
                sensors.fsr.right.fsr1 = data.fsr[0].fsr1 * FSR_FORCE_CONVERSION_FACTOR_R1;
                sensors.fsr.right.fsr2 = data.fsr[0].fsr2 * FSR_FORCE_CONVERSION_FACTOR_R2;
                sensors.fsr.right.fsr3 = data.fsr[0].fsr3 * FSR_FORCE_CONVERSION_FACTOR_R3;
                sensors.fsr.right.fsr4 = data.fsr[0].fsr4 * FSR_FORCE_CONVERSION_FACTOR_R4;

                // Centre
                sensors.fsr.right.centreX = Convert::fsrCentre(false, true, data.fsr[0].centreX);
                sensors.fsr.right.centreY = Convert::fsrCentre(false, false, data.fsr[0].centreY);

                // Left Sensor
                // Error
                sensors.fsr.left.errorFlags = data.fsrErrorCodes[1] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.fsrErrorCodes[1]);

                // Sensors
                sensors.fsr.left.fsr1 = Convert::fsrForce(data.fsr[1].fsr1);
                sensors.fsr.left.fsr2 = Convert::fsrForce(data.fsr[1].fsr2);
                sensors.fsr.left.fsr3 = Convert::fsrForce(data.fsr[1].fsr3);
                sensors.fsr.left.fsr4 = Convert::fsrForce(data.fsr[1].fsr4);

                // Centre
                sensors.fsr.left.centreX = Convert::fsrCentre(true, true, data.fsr[1].centreX);
                sensors.fsr.left.centreY = Convert::fsrCentre(true, false, data.fsr[1].centreY);

                /*
                 Servos
                 */

                for(int i = 0; i < 20; ++i) {
                    // Get a reference to the servo we are populating
                    DarwinSensors::Servo& servo = sensors.servo[i];

                    // Error code
                    servo.errorFlags = data.servoErrorCodes[i] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.servoErrorCodes[i]);

                    // Booleans
                    servo.torqueEnabled = data.servos[i].torqueEnabled;
                    servo.led = data.servos[i].LED;

                    // Gain
                    servo.dGain = Convert::gain(data.servos[i].dGain);
                    servo.iGain = Convert::gain(data.servos[i].iGain);
                    servo.pGain = Convert::gain(data.servos[i].pGain);

                    // Targets
                    servo.goalPosition = Convert::servoPosition(i, data.servos[i].goalPosition);
                    servo.movingSpeed = Convert::servoSpeed(i, data.servos[i].movingSpeed);
                    servo.torqueLimit = Convert::torqueLimit(data.servos[i].torqueLimit);

                    // Present Data
                    servo.presentPosition = Convert::servoPosition(i, data.servos[i].presentPosition);
                    servo.presentSpeed = Convert::servoSpeed(i, data.servos[i].presentSpeed);
                    servo.load = Convert::servoLoad(i, data.servos[i].load);

                    // Diagnostic Information
                    servo.voltage = Convert::voltage(data.servos[i].voltage);
                    servo.temperature = Convert::temperature(data.servos[i].temperature);
                }

                return sensors;
            }
        }

        ServoValues convert(const ServoTarget& in) {

            return {
                static_cast<uint8_t>(static_cast<int>(command.id) + 1),  // The id's on the robot start with ID 1

                Convert::gainInverse(command.gain * 0), // Derivitive gain
                Convert::gainInverse(command.gain * 0), // Integral gain
                Convert::gainInverse(command.gain),     // Proportional gain
                0,
                Convert::servoPositionInverse(static_cast<int>(command.id), command.position),
                Convert::servoSpeedInverse(static_cast<int>(command.id), speed)
            }

            float diff = utility::math::angle::difference(command.position, sensors.servo[command.id].presentPosition);
                    NUClear::clock::duration duration = command.time - NUClear::clock::now();

                    float speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));

                    values.push_back(
                }

        }


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
            // offset and normalize the angle
            return utility::math::angle::normalizeAngle((((value - 2048) * POSITION_CONVERSION_FACTOR) + SERVO_OFFSET[id]) * SERVO_DIRECTION[id]);
        }

        uint16_t Convert::servoPositionInverse(const uint8_t id, const float value) {
            float angle = value;

            // Undo our conversion operations
            angle *= SERVO_DIRECTION[id];
            angle -= SERVO_OFFSET[id];

            // Normalize the angle
            angle = utility::math::angle::normalizeAngle(angle);

            // Convert it back
            return (angle / POSITION_CONVERSION_FACTOR) + 2048;
        }

        float Convert::servoSpeed(const uint8_t id, const uint16_t value) {

            // We only care about the lower bits
            float raw = (value & 0x3FF) * SPEED_CONVERSION_FACTOR[id];

            // If bit 10 is set we are moving Clockwise
            raw *= value & 0x400 ? -1 : 1;

            // Go the correct direction
            raw *= SERVO_DIRECTION[id];

            return raw;
        }
        uint16_t Convert::servoSpeedInverse(const uint8_t id, const float value) {
            uint16_t raw = round(value / SPEED_CONVERSION_FACTOR[id]);

            // If the value is greater then 1023, then set to max speed (0)
            return raw > 1023 ? 0 : raw;
        }

        float Convert::torqueLimit(const uint16_t value) {
            return value * TORQUE_LIMIT_CONVERSION_FACTOR;
        }

        float Convert::servoLoad(const uint8_t id, const uint16_t value) {
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
