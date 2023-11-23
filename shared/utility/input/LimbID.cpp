/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "LimbID.hpp"

#include <stdexcept>

namespace utility::input {

    using ServoID = utility::input::ServoID;

    LimbID::LimbID(std::string const& str) {
        // clang-format off
        if      (str == "UNKNOWN")   { value = Value::UNKNOWN;   }
        else if (str == "LEFT_LEG")  { value = Value::LEFT_LEG;  }
        else if (str == "RIGHT_LEG") { value = Value::RIGHT_LEG; }
        else if (str == "LEFT_ARM")  { value = Value::LEFT_ARM;  }
        else if (str == "RIGHT_ARM") { value = Value::RIGHT_ARM; }
        else if (str == "HEAD")      { value = Value::HEAD;      }
        else {
            throw std::runtime_error("String did not match any enum for LimbID");
        }
        // clang-format on
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

    std::set<ServoID> LimbID::servos_for_limb(const LimbID& limb) {
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

    LimbID LimbID::limb_for_servo(const ServoID& servo) {
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
        }
    }

    std::ostream& operator<<(std::ostream& out, const LimbID& val) {
        return out << static_cast<std::string>(val);
    }

    std::set<ServoID> LimbID::servos_for_legs() {
        return {ServoID::L_ANKLE_PITCH,
                ServoID::L_ANKLE_ROLL,
                ServoID::L_HIP_PITCH,
                ServoID::L_HIP_ROLL,
                ServoID::L_HIP_YAW,
                ServoID::L_KNEE,
                ServoID::R_ANKLE_PITCH,
                ServoID::R_ANKLE_ROLL,
                ServoID::R_HIP_PITCH,
                ServoID::R_HIP_ROLL,
                ServoID::R_HIP_YAW,
                ServoID::R_KNEE};
    }

    std::set<ServoID> LimbID::servos_for_arms() {
        return {ServoID::L_SHOULDER_PITCH,
                ServoID::L_SHOULDER_ROLL,
                ServoID::L_ELBOW,
                ServoID::R_SHOULDER_PITCH,
                ServoID::R_SHOULDER_ROLL,
                ServoID::R_ELBOW};
    }
}  // namespace utility::input
