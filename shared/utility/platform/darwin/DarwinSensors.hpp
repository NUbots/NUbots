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
#ifndef UTILITY_PLATFORM_DARWIN_DARWINSENSORS_HPP
#define UTILITY_PLATFORM_DARWIN_DARWINSENSORS_HPP

#include "message/platform/darwin/DarwinSensors.hpp"

#include "utility/input/ServoID.hpp"

namespace utility {
namespace platform {
    namespace darwin {

        using ServoID = utility::input::ServoID;
        using message::platform::darwin::DarwinSensors;

        inline const DarwinSensors::Servo& getDarwinServo(ServoID servoId, const DarwinSensors& sensors) {

            switch (servoId.value) {
                case ServoID::R_SHOULDER_PITCH: return sensors.servo.rShoulderPitch;
                case ServoID::L_SHOULDER_PITCH: return sensors.servo.lShoulderPitch;
                case ServoID::R_SHOULDER_ROLL: return sensors.servo.rShoulderRoll;
                case ServoID::L_SHOULDER_ROLL: return sensors.servo.lShoulderRoll;
                case ServoID::R_ELBOW: return sensors.servo.rElbow;
                case ServoID::L_ELBOW: return sensors.servo.lElbow;
                case ServoID::R_HIP_YAW: return sensors.servo.rHipYaw;
                case ServoID::L_HIP_YAW: return sensors.servo.lHipYaw;
                case ServoID::R_HIP_ROLL: return sensors.servo.rHipRoll;
                case ServoID::L_HIP_ROLL: return sensors.servo.lHipRoll;
                case ServoID::R_HIP_PITCH: return sensors.servo.rHipPitch;
                case ServoID::L_HIP_PITCH: return sensors.servo.lHipPitch;
                case ServoID::R_KNEE: return sensors.servo.rKnee;
                case ServoID::L_KNEE: return sensors.servo.lKnee;
                case ServoID::R_ANKLE_PITCH: return sensors.servo.rAnklePitch;
                case ServoID::L_ANKLE_PITCH: return sensors.servo.lAnklePitch;
                case ServoID::R_ANKLE_ROLL: return sensors.servo.rAnkleRoll;
                case ServoID::L_ANKLE_ROLL: return sensors.servo.lAnkleRoll;
                case ServoID::HEAD_YAW: return sensors.servo.headPan;
                case ServoID::HEAD_PITCH: return sensors.servo.headTilt;

                default: throw std::runtime_error("Out of bounds");
            }
        }

        inline DarwinSensors::Servo& getDarwinServo(ServoID servoId, DarwinSensors& sensors) {

            switch (servoId.value) {
                case ServoID::R_SHOULDER_PITCH: return sensors.servo.rShoulderPitch;
                case ServoID::L_SHOULDER_PITCH: return sensors.servo.lShoulderPitch;
                case ServoID::R_SHOULDER_ROLL: return sensors.servo.rShoulderRoll;
                case ServoID::L_SHOULDER_ROLL: return sensors.servo.lShoulderRoll;
                case ServoID::R_ELBOW: return sensors.servo.rElbow;
                case ServoID::L_ELBOW: return sensors.servo.lElbow;
                case ServoID::R_HIP_YAW: return sensors.servo.rHipYaw;
                case ServoID::L_HIP_YAW: return sensors.servo.lHipYaw;
                case ServoID::R_HIP_ROLL: return sensors.servo.rHipRoll;
                case ServoID::L_HIP_ROLL: return sensors.servo.lHipRoll;
                case ServoID::R_HIP_PITCH: return sensors.servo.rHipPitch;
                case ServoID::L_HIP_PITCH: return sensors.servo.lHipPitch;
                case ServoID::R_KNEE: return sensors.servo.rKnee;
                case ServoID::L_KNEE: return sensors.servo.lKnee;
                case ServoID::R_ANKLE_PITCH: return sensors.servo.rAnklePitch;
                case ServoID::L_ANKLE_PITCH: return sensors.servo.lAnklePitch;
                case ServoID::R_ANKLE_ROLL: return sensors.servo.rAnkleRoll;
                case ServoID::L_ANKLE_ROLL: return sensors.servo.lAnkleRoll;
                case ServoID::HEAD_YAW: return sensors.servo.headPan;
                case ServoID::HEAD_PITCH: return sensors.servo.headTilt;

                default: throw std::runtime_error("Out of bounds");
            }
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace utility

#endif
