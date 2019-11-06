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
#include "LimbID.h"

#include <stdexcept>

namespace utility {
namespace input {

    using ServoID = utility::input::ServoID;

    LimbID::LimbID(std::string const& str) : value(Value::UNKNOWN) {
        if (str == "UNKNOWN")
            value = Value::UNKNOWN;
        else if (str == "LEFT_LEG")
            value = Value::LEFT_LEG;
        else if (str == "RIGHT_LEG")
            value = Value::RIGHT_LEG;
        else if (str == "LEFT_ARM")
            value = Value::LEFT_ARM;
        else if (str == "RIGHT_ARM")
            value = Value::RIGHT_ARM;
        else if (str == "HEAD")
            value = Value::HEAD;
        else
            throw std::runtime_error("String did not match any enum for LimbID");
    }


    LimbID::operator std::string() const {
        switch (value) {
            case Value::UNKNOWN: return "UNKNOWN";
            case Value::LEFT_LEG: return "LEFT_LEG";
            case Value::RIGHT_LEG: return "RIGHT_LEG";
            case Value::LEFT_ARM: return "LEFT_ARM";
            case Value::RIGHT_ARM: return "RIGHT_ARM";
            case Value::HEAD: return "HEAD";
            default: throw std::runtime_error("enum LimbID's value is corrupt, unknown value stored");
        }
    }

    std::set<ServoID> LimbID::servosForLimb(const LimbID& limb) {
        switch (limb.value) {
            case LimbID::HEAD: return {ServoID::HEAD_PITCH, ServoID::HEAD_YAW};

            case LimbID::LEFT_LEG:
                return {ServoID::L_ANKLE_PITCH,
                        ServoID::L_ANKLE_ROLL,
                        ServoID::L_HIP_PITCH,
                        ServoID::L_HIP_ROLL,
                        ServoID::L_HIP_YAW,
                        ServoID::L_KNEE};

            case LimbID::RIGHT_LEG:
                return {ServoID::R_ANKLE_PITCH,
                        ServoID::R_ANKLE_ROLL,
                        ServoID::R_HIP_PITCH,
                        ServoID::R_HIP_ROLL,
                        ServoID::R_HIP_YAW,
                        ServoID::R_KNEE};

            case LimbID::LEFT_ARM: return {ServoID::L_SHOULDER_PITCH, ServoID::L_SHOULDER_ROLL, ServoID::L_ELBOW};

            case LimbID::RIGHT_ARM: return {ServoID::R_SHOULDER_PITCH, ServoID::R_SHOULDER_ROLL, ServoID::R_ELBOW};

            default: {
                // Can't really happen but in case it does make sure someone pays!
                return std::set<ServoID>{};
            }
        }
    }

    LimbID LimbID::limbForServo(const ServoID& servo) {
        switch (servo.value) {
            case ServoID::HEAD_PITCH:
            case ServoID::HEAD_YAW: return LimbID::HEAD;

            case ServoID::L_ANKLE_PITCH:
            case ServoID::L_ANKLE_ROLL:
            case ServoID::L_HIP_PITCH:
            case ServoID::L_HIP_ROLL:
            case ServoID::L_HIP_YAW:
            case ServoID::L_KNEE: return LimbID::LEFT_LEG;

            case ServoID::R_ANKLE_PITCH:
            case ServoID::R_ANKLE_ROLL:
            case ServoID::R_HIP_PITCH:
            case ServoID::R_HIP_ROLL:
            case ServoID::R_HIP_YAW:
            case ServoID::R_KNEE: return LimbID::RIGHT_LEG;

            case ServoID::L_SHOULDER_PITCH:
            case ServoID::L_SHOULDER_ROLL:
            case ServoID::L_ELBOW: return LimbID::LEFT_ARM;

            case ServoID::R_SHOULDER_PITCH:
            case ServoID::R_SHOULDER_ROLL:
            case ServoID::R_ELBOW: return LimbID::RIGHT_ARM;
            case ServoID::NUMBER_OF_SERVOS:
            default:
                // Can't really happen but in case it does make sure someone pays!
                return static_cast<LimbID>(-1);
                ;
        }
    }

    std::ostream& operator<<(std::ostream& out, const LimbID& val) {
        return out << static_cast<std::string>(val);
    }
}  // namespace input
}  // namespace utility
