/*
 * This file is part of NUBots.
 *
 * NUBots is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ServoID.h"

namespace messages {
    namespace input {

        const ServoID idFromString(const std::string str) {

            return    str == "R_SHOULDER_PITCH" ? ServoID::R_SHOULDER_PITCH
                    : str == "L_SHOULDER_PITCH" ? ServoID::L_SHOULDER_PITCH
                    : str == "R_SHOULDER_ROLL"  ? ServoID::R_SHOULDER_ROLL
                    : str == "L_SHOULDER_ROLL"  ? ServoID::L_SHOULDER_ROLL
                    : str == "R_ELBOW"          ? ServoID::R_ELBOW
                    : str == "L_ELBOW"          ? ServoID::L_ELBOW
                    : str == "R_HIP_YAW"        ? ServoID::R_HIP_YAW
                    : str == "L_HIP_YAW"        ? ServoID::L_HIP_YAW
                    : str == "R_HIP_ROLL"       ? ServoID::R_HIP_ROLL
                    : str == "L_HIP_ROLL"       ? ServoID::L_HIP_ROLL
                    : str == "R_HIP_PITCH"      ? ServoID::R_HIP_PITCH
                    : str == "L_HIP_PITCH"      ? ServoID::L_HIP_PITCH
                    : str == "R_KNEE"           ? ServoID::R_KNEE
                    : str == "L_KNEE"           ? ServoID::L_KNEE
                    : str == "R_ANKLE_PITCH"    ? ServoID::R_ANKLE_PITCH
                    : str == "L_ANKLE_PITCH"    ? ServoID::L_ANKLE_PITCH
                    : str == "R_ANKLE_ROLL"     ? ServoID::R_ANKLE_ROLL
                    : str == "L_ANKLE_ROLL"     ? ServoID::L_ANKLE_ROLL
                    : str == "HEAD_PAN"         ? ServoID::HEAD_PAN
                    : str == "HEAD_TILT"        ? ServoID::HEAD_TILT
                    : static_cast<ServoID>(-1);
        }

        const std::string stringFromId(const ServoID id) {

            switch(id) {
                case ServoID::R_SHOULDER_PITCH:  return "R_SHOULDER_PITCH";
                case ServoID::L_SHOULDER_PITCH:  return "L_SHOULDER_PITCH";
                case ServoID::R_SHOULDER_ROLL:   return "R_SHOULDER_ROLL";
                case ServoID::L_SHOULDER_ROLL:   return "L_SHOULDER_ROLL";
                case ServoID::R_ELBOW:           return "R_ELBOW";
                case ServoID::L_ELBOW:           return "L_ELBOW";
                case ServoID::R_HIP_YAW:         return "R_HIP_YAW";
                case ServoID::L_HIP_YAW:         return "L_HIP_YAW";
                case ServoID::R_HIP_ROLL:        return "R_HIP_ROLL";
                case ServoID::L_HIP_ROLL:        return "L_HIP_ROLL";
                case ServoID::R_HIP_PITCH:       return "R_HIP_PITCH";
                case ServoID::L_HIP_PITCH:       return "L_HIP_PITCH";
                case ServoID::R_KNEE:            return "R_KNEE";
                case ServoID::L_KNEE:            return "L_KNEE";
                case ServoID::R_ANKLE_PITCH:     return "R_ANKLE_PITCH";
                case ServoID::L_ANKLE_PITCH:     return "L_ANKLE_PITCH";
                case ServoID::R_ANKLE_ROLL:      return "R_ANKLE_ROLL";
                case ServoID::L_ANKLE_ROLL:      return "L_ANKLE_ROLL";
                case ServoID::HEAD_PAN:          return "HEAD_PAN";
                case ServoID::HEAD_TILT:         return "HEAD_TILT";
                default:                         return "UNKNOWN";
            }
        }
    }
}