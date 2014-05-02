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
                sensors.ledPanel.led2 = (data.cm730.ledPanel & (1 << 1)) == (1 << 1);
                sensors.ledPanel.led3 = (data.cm730.ledPanel & (1 << 2)) == (1 << 2);
                sensors.ledPanel.led4 = (data.cm730.ledPanel & (1 << 3)) == (1 << 3);

                // Head LED - Extract the colours
                sensors.headLED.r = (data.cm730.headLED & 0x001F) << 3;
                sensors.headLED.g = (data.cm730.headLED & 0x03E0) >> 2;
                sensors.headLED.b = (data.cm730.headLED & 0x7C00) >> 7;

                // Eye LED - Extract the colours
                sensors.eyeLED.r = (data.cm730.eyeLED & 0x001F) << 3;
                sensors.eyeLED.g = (data.cm730.eyeLED & 0x03E0) >> 2;
                sensors.eyeLED.b = (data.cm730.eyeLED & 0x7C00) >> 7;

                // Buttons - Extract the bits
                sensors.buttons.left = (data.cm730.buttons & (1 << 1)) == (1 << 1);
                sensors.buttons.middle = (data.cm730.buttons & (1 << 2)) == (1 << 2);

                // Voltage (in volts) - Times by the conversion factor
                sensors.voltage = data.cm730.voltage * VOLTAGE_CONVERSION_FACTOR;

                // Accelerometer (in m/s^2) - Times by the conversion factors
                sensors.accelerometer.x = (data.cm730.accelerometer.x - 512) * ACCELEROMETER_CONVERSION_FACTOR_X;
                sensors.accelerometer.y = (data.cm730.accelerometer.y - 512) * ACCELEROMETER_CONVERSION_FACTOR_Y;
                sensors.accelerometer.z = (data.cm730.accelerometer.z - 512) * ACCELEROMETER_CONVERSION_FACTOR_Z;

                // Gyroscope (in radians/second) - Times by the conversion factors
                sensors.gyroscope.x = (data.cm730.gyroscope.x - 512) * GYROSCOPE_CONVERSION_FACTOR_X;
                sensors.gyroscope.y = (data.cm730.gyroscope.y - 512) * GYROSCOPE_CONVERSION_FACTOR_Y;
                sensors.gyroscope.z = (data.cm730.gyroscope.z - 512) * GYROSCOPE_CONVERSION_FACTOR_Z;

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
                sensors.fsr.right.centreX = data.fsr[0].centreX == 0xFF ? std::numeric_limits<float>::quiet_NaN()
                                         : double(data.fsr[0].centreX - 127) / 127.0;

                sensors.fsr.right.centreY = data.fsr[0].centreY == 0xFF ? std::numeric_limits<float>::quiet_NaN()
                                         : double(127 - data.fsr[0].centreY) / 127.0;

                // Left Sensor
                // Error
                sensors.fsr.left.errorFlags = data.fsrErrorCodes[1] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.fsrErrorCodes[1]);

                // Sensors
                sensors.fsr.left.fsr1 = data.fsr[1].fsr1 * FSR_FORCE_CONVERSION_FACTOR_R1;
                sensors.fsr.left.fsr2 = data.fsr[1].fsr2 * FSR_FORCE_CONVERSION_FACTOR_R2;
                sensors.fsr.left.fsr3 = data.fsr[1].fsr3 * FSR_FORCE_CONVERSION_FACTOR_R3;
                sensors.fsr.left.fsr4 = data.fsr[1].fsr4 * FSR_FORCE_CONVERSION_FACTOR_R4;

                // Centre
                sensors.fsr.left.centreX = data.fsr[1].centreX == 0xFF ? std::numeric_limits<float>::quiet_NaN()
                                         : double(127 - data.fsr[1].centreX) / 127.0;

                sensors.fsr.left.centreY = data.fsr[0].centreY == 0xFF ? std::numeric_limits<float>::quiet_NaN()
                                         : double(data.fsr[1].centreY - 127) / 127.0;

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
                    servo.dGain = data.servos[i].dGain * GAIN_CONVERSION_FACTOR;
                    servo.iGain = data.servos[i].iGain * GAIN_CONVERSION_FACTOR;
                    servo.pGain = data.servos[i].pGain * GAIN_CONVERSION_FACTOR;

                    // Goal Position
                    servo.goalPosition  = (data.servos[i].goalPosition - 2048) * POSITION_CONVERSION_FACTOR;
                    servo.goalPosition += SERVO_OFFSET[i];
                    servo.goalPosition *= SERVO_DIRECTION[i];
                    servo.goalPosition  = utility::math::angle::normalizeAngle(servo.goalPosition);

                    // Moving Speed
                    servo.movingSpeed  = (data.servos[i].movingSpeed & 0x3FF);
                    servo.movingSpeed *= (data.servos[i].movingSpeed & 0x400) ? -1 : 1;
                    servo.movingSpeed *= SERVO_DIRECTION[i];

                    // Torque Limit
                    servo.torqueLimit = data.servos[i].torqueLimit * TORQUE_LIMIT_CONVERSION_FACTOR;

                    // Present Data
                    servo.presentPosition  = (data.servos[i].presentPosition - 2048) * POSITION_CONVERSION_FACTOR;
                    servo.presentPosition += SERVO_OFFSET[i];
                    servo.presentPosition *= SERVO_DIRECTION[i];
                    servo.presentPosition  = utility::math::angle::normalizeAngle(servo.goalPosition);

                    servo.presentSpeed  = (data.servos[i].presentSpeed & 0x3FF);
                    servo.presentSpeed *= (data.servos[i].presentSpeed & 0x400) ? -1 : 1;
                    servo.presentSpeed *= SERVO_DIRECTION[i];

                    // Diagnostic Information
                    servo.voltage = data.servos[i].voltage * VOLTAGE_CONVERSION_FACTOR;
                    servo.temperature = data.servos[i].temperature * TEMPERATURE_CONVERSION_FACTOR;
                }

                return sensors;
            }


            ServoValues Converter::convert(const ServoTarget& command, const DarwinSensors& sensors) {

                ServoValues out;

                // The id is one plus the index
                out.servoId = uint8_t(static_cast<int>(command.id) + 1);

                // Get our raw gain
                uint16_t rawGain = command.gain >= 100 ? 254 : command.gain < 0 ? 0 : std::round(command.gain / GAIN_CONVERSION_FACTOR);

                out.dGain = rawGain * D_GAIN;
                out.iGain = rawGain * I_GAIN;;
                out.pGain = rawGain * P_GAIN;;

                // This is a reserved byte, set to 0
                out.reserved = 0;

                // Inverse the position
                float angle = command.position;
                angle *= SERVO_DIRECTION[size_t(command.id)];
                angle -= SERVO_OFFSET[size_t(command.id)];
                angle = utility::math::angle::normalizeAngle(angle);
                out.goalPosition = (angle / POSITION_CONVERSION_FACTOR) + 2048;

                // Calculate our speed
                float diff = utility::math::angle::difference(command.position, sensors.servo[uint(command.id)].presentPosition);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();
                float speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                out.movingSpeed = round(speed / SPEED_CONVERSION_FACTOR[uint(command.id)]);

                return out;
            }
        }
    }
}
