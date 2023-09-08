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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */
#include "FrameID.hpp"

#include <stdexcept>

namespace utility::input {

    FrameID::FrameID(std::string const& str) {
        // clang-format off
        if      (str == "R_SHOULDER_PITCH") { value = Value::R_SHOULDER_PITCH; }
        else if (str == "L_SHOULDER_PITCH") { value = Value::L_SHOULDER_PITCH; }
        else if (str == "R_SHOULDER_ROLL")  { value = Value::R_SHOULDER_ROLL; }
        else if (str == "L_SHOULDER_ROLL")  { value = Value::L_SHOULDER_ROLL; }
        else if (str == "R_ELBOW")          { value = Value::R_ELBOW; }
        else if (str == "L_ELBOW")          { value = Value::L_ELBOW; }
        else if (str == "R_HIP_YAW")        { value = Value::R_HIP_YAW; }
        else if (str == "L_HIP_YAW")        { value = Value::L_HIP_YAW; }
        else if (str == "R_HIP_ROLL")       { value = Value::R_HIP_ROLL; }
        else if (str == "L_HIP_ROLL")       { value = Value::L_HIP_ROLL; }
        else if (str == "R_HIP_PITCH")      { value = Value::R_HIP_PITCH; }
        else if (str == "L_HIP_PITCH")      { value = Value::L_HIP_PITCH; }
        else if (str == "R_KNEE")           { value = Value::R_KNEE; }
        else if (str == "L_KNEE")           { value = Value::L_KNEE; }
        else if (str == "R_ANKLE_PITCH")    { value = Value::R_ANKLE_PITCH; }
        else if (str == "L_ANKLE_PITCH")    { value = Value::L_ANKLE_PITCH; }
        else if (str == "R_ANKLE_ROLL")     { value = Value::R_ANKLE_ROLL; }
        else if (str == "L_ANKLE_ROLL")     { value = Value::L_ANKLE_ROLL; }
        else if (str == "HEAD_YAW")         { value = Value::HEAD_YAW; }
        else if (str == "HEAD_PITCH")       { value = Value::HEAD_PITCH; }
        else if (str == "L_FOOT_BASE")      { value = Value::L_FOOT_BASE; }
        else if (str == "R_FOOT_BASE")      { value = Value::R_FOOT_BASE; }
        else if (str == "L_CAMERA")      { value = Value::L_CAMERA; }
        else if (str == "R_CAMERA")      { value = Value::R_CAMERA; }
        else {
            throw std::runtime_error("String " + str + " did not match any enum for FrameID");
        }
        // clang-format on
    }

    FrameID::operator std::string() const {
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
            case Value::L_FOOT_BASE: return "L_FOOT_BASE";
            case Value::R_FOOT_BASE: return "R_FOOT_BASE";
            case Value::L_CAMERA: return "L_CAMERA";
            case Value::R_CAMERA: return "R_CAMERA";
            case Value::NUMBER_OF_LINKS:
            default: throw std::runtime_error("enum FrameID's value is corrupt, unknown value stored");
        }
    }

    std::ostream& operator<<(std::ostream& out, const FrameID& val) {
        return out << static_cast<std::string>(val);
    }
}  // namespace utility::input