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

namespace messages {
    namespace platform {
        namespace darwin {

            const DarwinSensors::Servo::ID DarwinSensors::Servo::idFromString(const std::string str) {

                return    str == "R_SHOULDER_PITCH" ? ID::R_SHOULDER_PITCH
                        : str == "L_SHOULDER_PITCH" ? ID::L_SHOULDER_PITCH
                        : str == "R_SHOULDER_ROLL"  ? ID::R_SHOULDER_ROLL
                        : str == "L_SHOULDER_ROLL"  ? ID::L_SHOULDER_ROLL
                        : str == "R_ELBOW"          ? ID::R_ELBOW
                        : str == "L_ELBOW"          ? ID::L_ELBOW
                        : str == "R_HIP_YAW"        ? ID::R_HIP_YAW
                        : str == "L_HIP_YAW"        ? ID::L_HIP_YAW
                        : str == "R_HIP_ROLL"       ? ID::R_HIP_ROLL
                        : str == "L_HIP_ROLL"       ? ID::L_HIP_ROLL
                        : str == "R_HIP_PITCH"      ? ID::R_HIP_PITCH
                        : str == "L_HIP_PITCH"      ? ID::L_HIP_PITCH
                        : str == "R_KNEE"           ? ID::R_KNEE
                        : str == "L_KNEE"           ? ID::L_KNEE
                        : str == "R_ANKLE_PITCH"    ? ID::R_ANKLE_PITCH
                        : str == "L_ANKLE_PITCH"    ? ID::L_ANKLE_PITCH
                        : str == "R_ANKLE_ROLL"     ? ID::R_ANKLE_ROLL
                        : str == "L_ANKLE_ROLL"     ? ID::L_ANKLE_ROLL
                        : str == "HEAD_PAN"         ? ID::HEAD_PAN
                        : str == "HEAD_TILT"        ? ID::HEAD_TILT
                        : static_cast<ID>(-1);
            }

            const std::string DarwinSensors::Servo::stringFromId(const DarwinSensors::Servo::ID id) {

                switch(id) {
                    case ID::R_SHOULDER_PITCH:  return "R_SHOULDER_PITCH";
                    case ID::L_SHOULDER_PITCH:  return "L_SHOULDER_PITCH";
                    case ID::R_SHOULDER_ROLL:   return "R_SHOULDER_ROLL";
                    case ID::L_SHOULDER_ROLL:   return "L_SHOULDER_ROLL";
                    case ID::R_ELBOW:           return "R_ELBOW";
                    case ID::L_ELBOW:           return "L_ELBOW";
                    case ID::R_HIP_YAW:         return "R_HIP_YAW";
                    case ID::L_HIP_YAW:         return "L_HIP_YAW";
                    case ID::R_HIP_ROLL:        return "R_HIP_ROLL";
                    case ID::L_HIP_ROLL:        return "L_HIP_ROLL";
                    case ID::R_HIP_PITCH:       return "R_HIP_PITCH";
                    case ID::L_HIP_PITCH:       return "L_HIP_PITCH";
                    case ID::R_KNEE:            return "R_KNEE";
                    case ID::L_KNEE:            return "L_KNEE";
                    case ID::R_ANKLE_PITCH:     return "R_ANKLE_PITCH";
                    case ID::L_ANKLE_PITCH:     return "L_ANKLE_PITCH";
                    case ID::R_ANKLE_ROLL:      return "R_ANKLE_ROLL";
                    case ID::L_ANKLE_ROLL:      return "L_ANKLE_ROLL";
                    case ID::HEAD_PAN:          return "HEAD_PAN";
                    case ID::HEAD_TILT:         return "HEAD_TILT";
                    default:                    return "UNKNOWN";
                }
            }

            const DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) const {

                switch (static_cast<Servo::ID>(index)) {
                    case Servo::ID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case Servo::ID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case Servo::ID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case Servo::ID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case Servo::ID::R_ELBOW:            return rElbow;
                    case Servo::ID::L_ELBOW:            return lElbow;
                    case Servo::ID::R_HIP_YAW:          return rHipYaw;
                    case Servo::ID::L_HIP_YAW:          return lHipYaw;
                    case Servo::ID::R_HIP_ROLL:         return rHipRoll;
                    case Servo::ID::L_HIP_ROLL:         return lHipRoll;
                    case Servo::ID::R_HIP_PITCH:        return rHipPitch;
                    case Servo::ID::L_HIP_PITCH:        return lHipPitch;
                    case Servo::ID::R_KNEE:             return rKnee;
                    case Servo::ID::L_KNEE:             return lKnee;
                    case Servo::ID::R_ANKLE_PITCH:      return rAnklePitch;
                    case Servo::ID::L_ANKLE_PITCH:      return lAnklePitch;
                    case Servo::ID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case Servo::ID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case Servo::ID::HEAD_PAN:           return headPan;
                    case Servo::ID::HEAD_TILT:          return headTilt;
                }

                throw std::runtime_error("Out of bounds");
            }

            DarwinSensors::Servo& DarwinSensors::Servos::operator[](int index) {

                switch (static_cast<Servo::ID>(index)) {
                    case Servo::ID::R_SHOULDER_PITCH:   return rShoulderPitch;
                    case Servo::ID::L_SHOULDER_PITCH:   return lShoulderPitch;
                    case Servo::ID::R_SHOULDER_ROLL:    return rShoulderRoll;
                    case Servo::ID::L_SHOULDER_ROLL:    return lShoulderRoll;
                    case Servo::ID::R_ELBOW:            return rElbow;
                    case Servo::ID::L_ELBOW:            return lElbow;
                    case Servo::ID::R_HIP_YAW:          return rHipYaw;
                    case Servo::ID::L_HIP_YAW:          return lHipYaw;
                    case Servo::ID::R_HIP_ROLL:         return rHipRoll;
                    case Servo::ID::L_HIP_ROLL:         return lHipRoll;
                    case Servo::ID::R_HIP_PITCH:        return rHipPitch;
                    case Servo::ID::L_HIP_PITCH:        return lHipPitch;
                    case Servo::ID::R_KNEE:             return rKnee;
                    case Servo::ID::L_KNEE:             return lKnee;
                    case Servo::ID::R_ANKLE_PITCH:      return rAnklePitch;
                    case Servo::ID::L_ANKLE_PITCH:      return lAnklePitch;
                    case Servo::ID::R_ANKLE_ROLL:       return rAnkleRoll;
                    case Servo::ID::L_ANKLE_ROLL:       return lAnkleRoll;
                    case Servo::ID::HEAD_PAN:           return headPan;
                    case Servo::ID::HEAD_TILT:          return headTilt;
                }

                throw std::runtime_error("Out of bounds");
            }

        }  // darwin
    }  // platform
}  // messages