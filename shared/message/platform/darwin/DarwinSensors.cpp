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

#include "DarwinSensors.h"
#include "message/input/ServoID.h"

namespace message {
    namespace platform {
        namespace darwin {

            using ServoID = message::input::proto::Sensors::ServoID::Value;

            const DarwinSensors::Servo& DarwinSensors::Servos::operator[](input::ServoID servoId) const {

                switch (servoId) {
                    case ServoID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case ServoID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case ServoID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case ServoID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case ServoID::R_ELBOW:            return rElbow;
                    case ServoID::L_ELBOW:            return lElbow;
                    case ServoID::R_HIP_YAW:          return rHipYaw;
                    case ServoID::L_HIP_YAW:          return lHipYaw;
                    case ServoID::R_HIP_ROLL:         return rHipRoll;
                    case ServoID::L_HIP_ROLL:         return lHipRoll;
                    case ServoID::R_HIP_PITCH:        return rHipPitch;
                    case ServoID::L_HIP_PITCH:        return lHipPitch;
                    case ServoID::R_KNEE:             return rKnee;
                    case ServoID::L_KNEE:             return lKnee;
                    case ServoID::R_ANKLE_PITCH:      return rAnklePitch;
                    case ServoID::L_ANKLE_PITCH:      return lAnklePitch;
                    case ServoID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case ServoID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case ServoID::HEAD_YAW:           return headPan;
                    case ServoID::HEAD_PITCH:         return headTilt;

                    case ServoID::NUMBER_OF_SERVOS:   throw std::runtime_error("Out of bounds");
                }

                throw std::runtime_error("Out of bounds");
            }

            DarwinSensors::Servo& DarwinSensors::Servos::operator[](ServoID servoId) {

                switch (servoId) {
                    case ServoID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case ServoID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case ServoID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case ServoID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case ServoID::R_ELBOW:            return rElbow;
                    case ServoID::L_ELBOW:            return lElbow;
                    case ServoID::R_HIP_YAW:          return rHipYaw;
                    case ServoID::L_HIP_YAW:          return lHipYaw;
                    case ServoID::R_HIP_ROLL:         return rHipRoll;
                    case ServoID::L_HIP_ROLL:         return lHipRoll;
                    case ServoID::R_HIP_PITCH:        return rHipPitch;
                    case ServoID::L_HIP_PITCH:        return lHipPitch;
                    case ServoID::R_KNEE:             return rKnee;
                    case ServoID::L_KNEE:             return lKnee;
                    case ServoID::R_ANKLE_PITCH:      return rAnklePitch;
                    case ServoID::L_ANKLE_PITCH:      return lAnklePitch;
                    case ServoID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case ServoID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case ServoID::HEAD_YAW:           return headPan;
                    case ServoID::HEAD_PITCH:         return headTilt;

                    case ServoID::NUMBER_OF_SERVOS:   throw std::runtime_error("Out of bounds");
                }

                throw std::runtime_error("Out of bounds");
            }

            const DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) const {
                return (*this)[static_cast<ServoID>(index)];
            }

            DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) {
                return (*this)[static_cast<ServoID>(index)];
            }

        }  // darwin
    }  // platform
}  // message
