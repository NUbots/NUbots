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
#include "ServoID.h"

#include <stdexcept>

namespace utility {
namespace input {

    ServoID::ServoID(std::string const& str) : value(Value::R_SHOULDER_PITCH) {
        if (str == "R_SHOULDER_PITCH")
            value = Value::R_SHOULDER_PITCH;
        else if (str == "L_SHOULDER_PITCH")
            value = Value::L_SHOULDER_PITCH;
        else if (str == "R_SHOULDER_ROLL")
            value = Value::R_SHOULDER_ROLL;
        else if (str == "L_SHOULDER_ROLL")
            value = Value::L_SHOULDER_ROLL;
        else if (str == "R_ELBOW")
            value = Value::R_ELBOW;
        else if (str == "L_ELBOW")
            value = Value::L_ELBOW;
        else if (str == "R_HIP_YAW")
            value = Value::R_HIP_YAW;
        else if (str == "L_HIP_YAW")
            value = Value::L_HIP_YAW;
        else if (str == "R_HIP_ROLL")
            value = Value::R_HIP_ROLL;
        else if (str == "L_HIP_ROLL")
            value = Value::L_HIP_ROLL;
        else if (str == "R_HIP_PITCH")
            value = Value::R_HIP_PITCH;
        else if (str == "L_HIP_PITCH")
            value = Value::L_HIP_PITCH;
        else if (str == "R_KNEE")
            value = Value::R_KNEE;
        else if (str == "L_KNEE")
            value = Value::L_KNEE;
        else if (str == "R_ANKLE_PITCH")
            value = Value::R_ANKLE_PITCH;
        else if (str == "L_ANKLE_PITCH")
            value = Value::L_ANKLE_PITCH;
        else if (str == "R_ANKLE_ROLL")
            value = Value::R_ANKLE_ROLL;
        else if (str == "L_ANKLE_ROLL")
            value = Value::L_ANKLE_ROLL;
        else if (str == "HEAD_YAW")
            value = Value::HEAD_YAW;
        else if (str == "HEAD_PITCH")
            value = Value::HEAD_PITCH;
        else
            throw std::runtime_error("String " + str + " did not match any enum for ServoID");
    }

    ServoID::ServoID(std::string const& str, ServoSide const& side) : value(Value::R_SHOULDER_PITCH) {
        if (str == "SHOULDER_PITCH")
            value = static_cast<Value>(static_cast<int>(Value::R_SHOULDER_PITCH) + static_cast<int>(side));
        else if (str == "SHOULDER_ROLL")
            value = static_cast<Value>(static_cast<int>(Value::R_SHOULDER_ROLL) + static_cast<int>(side));
        else if (str == "ELBOW")
            value = static_cast<Value>(static_cast<int>(Value::R_ELBOW) + static_cast<int>(side));
        else if (str == "HIP_YAW")
            value = static_cast<Value>(static_cast<int>(Value::R_HIP_YAW) + static_cast<int>(side));
        else if (str == "HIP_ROLL")
            value = static_cast<Value>(static_cast<int>(Value::R_HIP_ROLL) + static_cast<int>(side));
        else if (str == "HIP_PITCH")
            value = static_cast<Value>(static_cast<int>(Value::R_HIP_PITCH) + static_cast<int>(side));
        else if (str == "KNEE")
            value = static_cast<Value>(static_cast<int>(Value::R_KNEE) + static_cast<int>(side));
        else if (str == "ANKLE_PITCH")
            value = static_cast<Value>(static_cast<int>(Value::R_ANKLE_PITCH) + static_cast<int>(side));
        else if (str == "ANKLE_ROLL")
            value = static_cast<Value>(static_cast<int>(Value::R_ANKLE_ROLL) + static_cast<int>(side));
        else if (str == "HEAD_YAW")
            value = static_cast<Value>(static_cast<int>(Value::HEAD_YAW) + static_cast<int>(side));
        else if (str == "HEAD_PITCH")
            value = static_cast<Value>(static_cast<int>(Value::HEAD_PITCH) + static_cast<int>(side));
        else
            throw std::runtime_error("String " + str + " on side " + std::to_string(side)
                                     + " did not match any enum for ServoID");
    }

    ServoID::operator std::string() const {
        switch (value) {
            case Value::R_SHOULDER_PITCH: return "R_SHOULDER_PITCH";
            case Value::L_SHOULDER_PITCH: return "L_SHOULDER_PITCH";
            case Value::R_SHOULDER_ROLL: return "R_SHOULDER_ROLL";
            case Value::L_SHOULDER_ROLL: return "L_SHOULDER_ROLL";
            case Value::R_ELBOW: return "R_ELBOW";
            case Value::L_ELBOW: return "L_ELBOW";
            case Value::R_HIP_YAW: return "R_HIP_YAW";
            case Value::L_HIP_YAW: return "L_HIP_YAW";
            case Value::R_HIP_ROLL: return "R_HIP_ROLL";
            case Value::L_HIP_ROLL: return "L_HIP_ROLL";
            case Value::R_HIP_PITCH: return "R_HIP_PITCH";
            case Value::L_HIP_PITCH: return "L_HIP_PITCH";
            case Value::R_KNEE: return "R_KNEE";
            case Value::L_KNEE: return "L_KNEE";
            case Value::R_ANKLE_PITCH: return "R_ANKLE_PITCH";
            case Value::L_ANKLE_PITCH: return "L_ANKLE_PITCH";
            case Value::R_ANKLE_ROLL: return "R_ANKLE_ROLL";
            case Value::L_ANKLE_ROLL: return "L_ANKLE_ROLL";
            case Value::HEAD_YAW: return "HEAD_YAW";
            case Value::HEAD_PITCH: return "HEAD_PITCH";
            case Value::NUMBER_OF_SERVOS:
            default: throw std::runtime_error("enum ServoID's value is corrupt, unknown value stored");
        }
    }

    std::ostream& operator<<(std::ostream& out, const ServoID& val) {
        return out << static_cast<std::string>(val);
    }
}  // namespace input
}  // namespace utility
