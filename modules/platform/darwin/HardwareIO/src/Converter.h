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

#include "darwin/DarwinRawSensors.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/motion/ServoTarget.h"


 namespace modules {
    namespace platform {
        namespace darwin {
            class Converter {

            public:
                // Conversion factors

                /// The value from the Darwin is between 0 and 1023, representing a value between -4g and 4g.
                /// This means 512 = 0
                double ACCELEROMETER_CONVERSION_FACTOR_X = (4 * 9.80665) / 512.0;
                double ACCELEROMETER_CONVERSION_FACTOR_Y = (4 * 9.80665) / 512.0;
                double ACCELEROMETER_CONVERSION_FACTOR_Z = (4 * 9.80665) / 512.0;

                /// The Gyrosocope value from the Darwin is between 0 and 1023, representing a value between -1600 and 1600
                /// degrees per second. This means 512 = 0
                double GYROSCOPE_CONVERSION_FACTOR_X = (1800.0 * (M_PI / 180.0)) / 512.0;
                double GYROSCOPE_CONVERSION_FACTOR_Y = (1800.0 * (M_PI / 180.0)) / 512.0;
                double GYROSCOPE_CONVERSION_FACTOR_Z = (1800.0 * (M_PI / 180.0)) / 512.0;

                /// The value that comes from the darwin is measured in decivolts (0.1 of a volt)
                double VOLTAGE_CONVERSION_FACTOR = 0.1;

                /// The FSR values that are measured by the darwins feet are measured in millinewtons
                double FSR_FORCE_CONVERSION_FACTOR_L1 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_L2 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_L3 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_L4 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_R1 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_R2 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_R3 = 0.001;
                double FSR_FORCE_CONVERSION_FACTOR_R4 = 0.001;

                /// The gain is a value between 0 and 254, we want a value between 0 and 100
                double GAIN_CONVERSION_FACTOR = 100.0 / 254.0;

                /// The angle is given as a value between 0 and 4095
                double POSITION_CONVERSION_FACTOR = (2.0 * M_PI) / 4095.0;

                /// The load is measured as a value between 0 and 2047 where the 10th bit specifies direction and 1024 = 0
                /// We convert it to a value between -100 and 100 (percentage)
                double LOAD_CONVERSION_FACTOR = 100.0 / 1023.0;

                /// The torque limit is measured as a value between 0 and 1023
                double TORQUE_LIMIT_CONVERSION_FACTOR = 100.0 / 1023.0;

                /// The temperatures are given in degrees anyway
                double TEMPERATURE_CONVERSION_FACTOR = 1.0;

                double P_GAIN = 1.0;
                double I_GAIN = 0.0;
                double D_GAIN = 0.0;

                /// Picks which direction a motor should be measured in (forward or reverse)
                int8_t SERVO_DIRECTION[20] = {
                    -1,            // [0]  R_SHOULDER_PITCH
                     1,            // [1]  L_SHOULDER_PITCH
                    -1,            // [2]  R_SHOULDER_ROLL
                    -1,            // [3]  L_SHOULDER_ROLL
                    -1,            // [4]  R_ELBOW
                     1,            // [5]  L_ELBOW
                    -1,            // [6]  R_HIP_YAW
                    -1,            // [7]  L_HIP_YAW
                    -1,            // [8]  R_HIP_ROLL
                    -1,            // [9]  L_HIP_ROLL
                     1,            // [10] R_HIP_PITCH
                    -1,            // [11] L_HIP_PITCH
                     1,            // [12] R_KNEE
                    -1,            // [13] L_KNEE
                    -1,            // [14] R_ANKLE_PITCH
                     1,            // [15] L_ANKLE_PITCH
                     1,            // [16] R_ANKLE_ROLL
                     1,            // [17] L_ANKLE_ROLL
                     1,            // [18]  HEAD_YAW
                    -1,            // [19]  HEAD_PITCH
                };

                /// Offsets the radian angles of motors to change their 0 position
                double SERVO_OFFSET[20] = {
                    -M_PI_2,    // [0]  R_SHOULDER_PITCH
                    M_PI_2,     // [1]  L_SHOULDER_PITCH
                    M_PI_4,     // [2]  R_SHOULDER_ROLL
                    -M_PI_4,    // [3]  L_SHOULDER_ROLL
                    M_PI_2,     // [4]  R_ELBOW
                    -M_PI_2,    // [5]  L_ELBOW
                    0.0,        // [6]  R_HIP_YAW
                    0.0,        // [7]  L_HIP_YAW
                    0.0,        // [8]  R_HIP_ROLL
                    0.0,        // [9]  L_HIP_ROLL
                    0.0,        // [10] R_HIP_PITCH
                    0.0,        // [11] L_HIP_PITCH
                    0.0,        // [12] R_KNEE
                    0.0,        // [13] L_KNEE
                    0.0,        // [14] R_ANKLE_PITCH
                    0.0,        // [15] L_ANKLE_PITCH
                    0.0,        // [16] R_ANKLE_ROLL
                    0.0,        // [17] L_ANKLE_ROLL
                    0.0,        // [18] HEAD_YAW
                    -0.631,     // [19] HEAD_PITCH
                };

                /// The MX28 measures its speed between 0 and 1023 where 1023 means a speed of 117.07rpm
                double MX28_SPEED_CONVERSION_FACTOR = (117.07 * 2.0 * M_PI) / (1023.0 * 60);
                /// The RX28 measures its speed between 0 and 1023 where 1023 means a speed of 54rpm
                double RX28_SPEED_CONVERSION_FACTOR = (54 * 2.0 * M_PI) / (1023.0 * 60);

                /// Calculates the speed conversion factor per servo
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

                messages::platform::darwin::DarwinSensors convert(const Darwin::BulkReadResults& in);
                Darwin::Types::ServoValues convert(const messages::motion::ServoTarget& in, const messages::platform::darwin::DarwinSensors& sensors);

            private:
                // Conversion functions
            };
        }
    }
 }

// Hold all the conversion factors (from the config file)

// Have a public method that takes a Darwin data object and converts it

// Have methods that take servo waypoints and convert them to our hardware format