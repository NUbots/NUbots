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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DarwinSensors.h"
#include "messages/input/ServoID.h"

namespace messages {
    namespace platform {
        namespace darwin {
            
            const DarwinSensors::Servo& DarwinSensors::Servos::operator[](input::ServoID servoId) const {

                switch (servoId) {
                    case input::ServoID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case input::ServoID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case input::ServoID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case input::ServoID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case input::ServoID::R_ELBOW:            return rElbow;
                    case input::ServoID::L_ELBOW:            return lElbow;
                    case input::ServoID::R_HIP_YAW:          return rHipYaw;
                    case input::ServoID::L_HIP_YAW:          return lHipYaw;
                    case input::ServoID::R_HIP_ROLL:         return rHipRoll;
                    case input::ServoID::L_HIP_ROLL:         return lHipRoll;
                    case input::ServoID::R_HIP_PITCH:        return rHipPitch;
                    case input::ServoID::L_HIP_PITCH:        return lHipPitch;
                    case input::ServoID::R_KNEE:             return rKnee;
                    case input::ServoID::L_KNEE:             return lKnee;
                    case input::ServoID::R_ANKLE_PITCH:      return rAnklePitch;
                    case input::ServoID::L_ANKLE_PITCH:      return lAnklePitch;
                    case input::ServoID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case input::ServoID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case input::ServoID::HEAD_PAN:           return headPan;
                    case input::ServoID::HEAD_TILT:          return headTilt;
                }

                throw std::runtime_error("Out of bounds");
            }

            DarwinSensors::Servo& DarwinSensors::Servos::operator[](input::ServoID servoId) {

                switch (servoId) {
                    case input::ServoID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case input::ServoID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case input::ServoID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case input::ServoID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case input::ServoID::R_ELBOW:            return rElbow;
                    case input::ServoID::L_ELBOW:            return lElbow;
                    case input::ServoID::R_HIP_YAW:          return rHipYaw;
                    case input::ServoID::L_HIP_YAW:          return lHipYaw;
                    case input::ServoID::R_HIP_ROLL:         return rHipRoll;
                    case input::ServoID::L_HIP_ROLL:         return lHipRoll;
                    case input::ServoID::R_HIP_PITCH:        return rHipPitch;
                    case input::ServoID::L_HIP_PITCH:        return lHipPitch;
                    case input::ServoID::R_KNEE:             return rKnee;
                    case input::ServoID::L_KNEE:             return lKnee;
                    case input::ServoID::R_ANKLE_PITCH:      return rAnklePitch;
                    case input::ServoID::L_ANKLE_PITCH:      return lAnklePitch;
                    case input::ServoID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case input::ServoID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case input::ServoID::HEAD_PAN:           return headPan;
                    case input::ServoID::HEAD_TILT:          return headTilt;
                }
                
                throw std::runtime_error("Out of bounds");
            }

            const DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) const {
				return (*this)[static_cast<input::ServoID>(index)];
			}

            DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) {
				return (*this)[static_cast<input::ServoID>(index)];
			}

        }  // darwin
    }  // platform
}  // messages