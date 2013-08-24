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

#include "DarwinPlatform.h"

#include "messages/DarwinServos.h"
#include "messages/DarwinSensors.h"

namespace modules {

    /// This namespace contains all the functions that are required to convert the raw sensors into useful values
    namespace Convert {

        constexpr bool isClose(const int a, const int b) {
            return (a - b) < (1 + a * 0.01) && (a - b) > -(1 + a * 0.01);
        }

        constexpr bool isClose(const float a, const float b) {
            return a - b < 0.00001 && a - b > -0.00001;
        }

        constexpr int8_t direction[20] = {
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

        constexpr float offset[20] = {
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
            0.0f,           // [18]  HEAD_YAW
            -0.631,         // [19]  HEAD_PITCH
        };

        // Common functionality
        namespace {
            /// Returns true or false if bit is set on the value
            template <uint8_t bit>
            constexpr bool BIT(const uint16_t value) { return (value & (1 << bit)) == (1 << bit); }

            /// Returns -1 or 1 if bit is set on the value
            template <uint8_t bit>
            constexpr int SIGNBIT(const uint16_t value) { return BIT<bit>(value) ? -1 : 1; }

            /// Normalizes a value between 0 and max to be between low and high
            template <int low, int high, int max>
            constexpr float NORMALIZE(const uint16_t value) { return (float(value) / (float(max) / float(high - low))) - float(low); }
        }

        /// Converts a timeout error flag into a regular error flag
        constexpr uint8_t ERROR_FLAGS(const uint8_t value) { return value == -1 ? messages::DarwinSensors::Error::TIMEOUT : value; };

        /// Extracts the R value from an RGB led
        constexpr uint8_t COLOURED_LED_R(const uint16_t value) { return static_cast<uint8_t>((value & 0x001F) << 3); };
        /// Extracts the G value from an RGB led
        constexpr uint8_t COLOURED_LED_G(const uint16_t value) { return static_cast<uint8_t>((value & 0x03E0) >> 2); };
        /// Extracts the B value from an RGB led
        constexpr uint8_t COLOURED_LED_B(const uint16_t value) { return static_cast<uint8_t>((value & 0x7C00) >> 7); };
        constexpr uint16_t COLOURED_LED_INVERSE(const uint8_t r, const uint8_t g, const uint8_t b) {
            return ((r >> 3))
                 | ((g >> 3) << 5)
                 | ((b >> 3) << 10);
        }

        // Check that the colour inverse works
        static_assert(isClose(COLOURED_LED_INVERSE(COLOURED_LED_R(0x1402),
                                                   COLOURED_LED_G(0x1402),
                                                   COLOURED_LED_B(0x1402)), 0x1402), "There is a problem with the Inverse operation for colour");
        static_assert(isClose(COLOURED_LED_INVERSE(COLOURED_LED_R(0x7100),
                                                   COLOURED_LED_G(0x7100),
                                                   COLOURED_LED_B(0x7100)), 0x7100), "There is a problem with the Inverse operation for colour");

        /// Converts the acceleronometer value from between 0-1024 (-4g to +4g) to m/s^2
        constexpr float ACCELERONOMETER(const uint16_t value) { return float(value - 512) * 0.07661445312; }

        /// Converts the Gyroscope value from between 0 - 1024 (-1600 degrees/s to +1600 degrees/s) to radians per second
        constexpr float GYROSCOPE(const uint16_t value) { return float(value - 512) * 0.05454153912; }

        /// Converts the voltage from decivolts to volts
        constexpr float VOLTAGE(const uint8_t value) { return float(value) / 10.0; }

        /// Converts the force read from the FSR from millinewtons to newtons
        constexpr float FSR_FORCE(const uint16_t value) { return float(value) / 1000.0; }

        /// Normalizes the FSR centre value to between -1 and 1, and sets it to NAN if it is 255
        constexpr float FSR_CENTRE(const uint8_t value) { return value == 0xFF ? NAN : NORMALIZE<-1, 1, 127>(value); }

        /// Normalizes gain from 0-254 to between 0 and 1
        constexpr float GAIN(const uint8_t value) { return NORMALIZE<0, 1, 254>(value); }
        constexpr uint8_t GAIN_INVERSE(const float value) { return static_cast<uint8_t>(value * 254); }

        /// Check that our Gain and Gain Inverse are actuall inverses of eachother
        static_assert(isClose(GAIN_INVERSE(GAIN(10)), 10), "There is a problem with the Inverse operation for gain");
        static_assert(isClose(GAIN_INVERSE(GAIN(254)), 254), "There is a problem with the Inverse operation for gain");
        static_assert(isClose(GAIN(GAIN_INVERSE(0.5)), 0.5), "There is a problem with the Inverse operation for gain");
        static_assert(isClose(GAIN(GAIN_INVERSE(1)), 1.0), "There is a problem with the Inverse operation for gain");

        /// Converts a servo position from 0-4095 (-pi to pi) to radians
        constexpr float SERVO_POSITION(const uint8_t servoID, const uint16_t value) {

            return direction[servoID] * (offset[servoID] + (2 * M_PI * float(value-2048)) / 4095);
        }
        constexpr uint16_t SERVO_POSITION_INVERSE(const uint8_t servoID, const float value) {

            return (direction[servoID] * ((-4095 * offset[servoID]) + (4096 * M_PI)) + (4095 * value))/(direction[servoID] * 2 * M_PI);
        }

        // Check that our Servo position conversion is working properly
        static_assert(isClose(SERVO_POSITION_INVERSE(0, SERVO_POSITION(0, 2600)), 2600),    "There is a problem with the Inverse operation for Position");
        static_assert(isClose(SERVO_POSITION_INVERSE(3, SERVO_POSITION(3, 2100)), 2100),    "There is a problem with the Inverse operation for Position");
        static_assert(isClose(SERVO_POSITION_INVERSE(10, SERVO_POSITION(10, 2600)), 2600),  "There is a problem with the Inverse operation for Position");

        /// Converts a servo speed from its signed format into radians/second
        constexpr float SERVO_SPEED(const uint8_t servoID, const uint16_t value) {
            return direction[servoID] * SIGNBIT<10>(value) * float(value & 0x3FF) * 0.01193805208;
        }
        constexpr uint16_t SERVO_SPEED_INVERSE(const float value) {
            return ((value / 0.01193805208) < 0 ? -(value / 0.01193805208) : (value / 0.01193805208)) > 1023 ? 0 :
            ((value / 0.01193805208) < 0 ? -(value / 0.01193805208) : (value / 0.01193805208));
        }

        // Check that our Servo speed conversion is working properly
        static_assert(isClose(SERVO_SPEED_INVERSE(SERVO_SPEED(0, 900)), 900), "There is a problem with the Inverse operation for Speed");
        static_assert(isClose(SERVO_SPEED_INVERSE(SERVO_SPEED(3, 1000)), 1000), "There is a problem with the Inverse operation for Speed");
        static_assert(isClose(SERVO_SPEED_INVERSE(200), 0), "There is a problem with the Inverse operation for Speed");

        /// Normalizes torque limit from 0-1023 to between 0 and 1
        constexpr float TORQUE_LIMIT(const uint16_t value) { return NORMALIZE<0, 1, 1023>(value); }

        /// Normalizes a Servo load to between -1 and 1 (dependant on direction)
        constexpr float SERVO_LOAD(const uint8_t servoID, const uint16_t value) {
            return direction[servoID] * SIGNBIT<10>(value) * NORMALIZE<-1, 1, 1023>(value & 0x3FF);
        }

        /// Temperature is already in degrees celsius
        constexpr uint8_t TEMPERATURE(const uint8_t value) { return value; }
    };

    DarwinPlatform::DarwinPlatform(NUClear::PowerPlant* plant) : Reactor(plant), darwin("/dev/ttyUSB0") {

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<20, std::chrono::milliseconds>>>([this](const time_t& time) {

            // Read our data
            Darwin::BulkReadResults data = darwin.bulkRead();

            messages::DarwinSensors* sensors = new messages::DarwinSensors;

            /*
             CM730 Data
             */

            // Read our Error code
            sensors->cm730ErrorFlags = Convert::ERROR_FLAGS(data.cm730ErrorCode);

            // LED Panel
            sensors->ledPanel.led2 = Convert::BIT<1>(data.cm730.ledPanel);
            sensors->ledPanel.led3 = Convert::BIT<2>(data.cm730.ledPanel);
            sensors->ledPanel.led4 = Convert::BIT<3>(data.cm730.ledPanel);

            // Head LED
            sensors->headLED.r = Convert::COLOURED_LED_R(data.cm730.headLED);
            sensors->headLED.g = Convert::COLOURED_LED_G(data.cm730.headLED);
            sensors->headLED.b = Convert::COLOURED_LED_B(data.cm730.headLED);

            // Eye LEDs
            sensors->eyeLED.r = Convert::COLOURED_LED_R(data.cm730.eyeLED);
            sensors->eyeLED.g = Convert::COLOURED_LED_G(data.cm730.eyeLED);
            sensors->eyeLED.b = Convert::COLOURED_LED_B(data.cm730.eyeLED);

            // Buttons
            sensors->buttons.left = Convert::BIT<1>(data.cm730.buttons);
            sensors->buttons.middle = Convert::BIT<2>(data.cm730.buttons);

            // Voltage (in volts)
            sensors->voltage = Convert::VOLTAGE(data.cm730.voltage);

            // Acceleronometer (in m/s^2)
            sensors->acceleronometer.x = Convert::ACCELERONOMETER(data.cm730.acceleronometer.x);
            sensors->acceleronometer.y = Convert::ACCELERONOMETER(data.cm730.acceleronometer.y);
            sensors->acceleronometer.z = Convert::ACCELERONOMETER(data.cm730.acceleronometer.z);

            // Gyroscope (in radians/second)
            sensors->gyroscope.x = Convert::GYROSCOPE(data.cm730.gyroscope.x);
            sensors->gyroscope.y = Convert::GYROSCOPE(data.cm730.gyroscope.y);
            sensors->gyroscope.z = Convert::GYROSCOPE(data.cm730.gyroscope.z);

            /*
             Force Sensitive Resistor Data
             */

            // Right Sensor
            // Error
            sensors->fsr.right.errorFlags = Convert::ERROR_FLAGS(data.fsrErrorCodes[0]);

            // Sensors
            sensors->fsr.right.fsr1 = Convert::FSR_FORCE(data.fsr[0].fsr1);
            sensors->fsr.right.fsr2 = Convert::FSR_FORCE(data.fsr[0].fsr2);
            sensors->fsr.right.fsr3 = Convert::FSR_FORCE(data.fsr[0].fsr3);
            sensors->fsr.right.fsr4 = Convert::FSR_FORCE(data.fsr[0].fsr4);

            // Centre
            sensors->fsr.right.centreX = Convert::FSR_CENTRE(data.fsr[0].centreX);
            sensors->fsr.right.centreY = Convert::FSR_CENTRE(data.fsr[0].centreY);

            // Left Sensor
            // Error
            sensors->fsr.left.errorFlags = Convert::ERROR_FLAGS(data.fsrErrorCodes[1]);

            // Sensors
            sensors->fsr.left.fsr1 = Convert::FSR_FORCE(data.fsr[1].fsr1);
            sensors->fsr.left.fsr2 = Convert::FSR_FORCE(data.fsr[1].fsr2);
            sensors->fsr.left.fsr3 = Convert::FSR_FORCE(data.fsr[1].fsr3);
            sensors->fsr.left.fsr4 = Convert::FSR_FORCE(data.fsr[1].fsr4);

            // Centre
            sensors->fsr.left.centreX = Convert::FSR_CENTRE(data.fsr[1].centreX);
            sensors->fsr.left.centreY = Convert::FSR_CENTRE(data.fsr[1].centreY);

            /*
             Servos
             */

            for(int i = 0; i < 20; ++i) {
                // Get a reference to the servo we are populating
                messages::DarwinSensors::Servo& servo = sensors->servo[i];

                // Error code
                servo.errorFlags = Convert::ERROR_FLAGS(data.servoErrorCodes[i]);

                // Booleans
                servo.torqueEnabled = data.servos[i].torqueEnabled;
                servo.led = data.servos[i].LED;

                // Gain
                servo.dGain = Convert::GAIN(data.servos[i].dGain);
                servo.iGain = Convert::GAIN(data.servos[i].iGain);
                servo.pGain = Convert::GAIN(data.servos[i].pGain);

                // Targets
                servo.goalPosition = Convert::SERVO_POSITION(i, data.servos[i].goalPosition);
                servo.movingSpeed = Convert::SERVO_SPEED(i, data.servos[i].movingSpeed);
                servo.torqueLimit = Convert::TORQUE_LIMIT(data.servos[i].torqueLimit);

                // Present Data
                servo.presentPosition = Convert::SERVO_POSITION(i, data.servos[i].presentPosition);
                servo.presentSpeed = Convert::SERVO_SPEED(i, data.servos[i].presentSpeed);
                servo.load = Convert::SERVO_LOAD(i, data.servos[i].load);

                // Diagnostic Information
                servo.voltage = Convert::VOLTAGE(data.servos[i].voltage);
                servo.temperature = Convert::TEMPERATURE(data.servos[i].temperature);
            }

            // Send our nicely computed sensor data out to the world
            emit(sensors);
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<messages::DarwinServoCommands>>([this](const messages::DarwinServoCommands& servos) {

            std::vector<Darwin::Types::ServoValues> values;

            // Loop through each of our commands
            for (const auto& command : servos.commands) {
                values.push_back({
                    static_cast<uint8_t>(command.servoId + 1),  // The id's on the robot start with ID 1
                    Convert::GAIN_INVERSE(command.dGain),
                    Convert::GAIN_INVERSE(command.iGain),
                    Convert::GAIN_INVERSE(command.pGain),
                    0,
                    Convert::SERVO_POSITION_INVERSE(command.servoId, command.goalPosition),
                    Convert::SERVO_SPEED_INVERSE(command.goalPosition)
                });
            }

            // Syncwrite our values
            darwin.writeServos(values);
        });

        // If we get a HeadLED command then write it
        on<Trigger<messages::DarwinSensors::HeadLED>>([this](const messages::DarwinSensors::HeadLED& led) {
            darwin.cm730.write(Darwin::CM730::Address::LED_HEAD_L, Convert::COLOURED_LED_INVERSE(led.r, led.g, led.b));
        });

        // If we get a HeadLED command then write it
        on<Trigger<messages::DarwinSensors::EyeLED>>([this](const messages::DarwinSensors::EyeLED& led) {
            darwin.cm730.write(Darwin::CM730::Address::LED_EYE_L, Convert::COLOURED_LED_INVERSE(led.r, led.g, led.b));
        });
    }
}
