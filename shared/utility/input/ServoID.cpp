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
#include "ServoID.hpp"

#include <stdexcept>

namespace utility::input {

    ServoID::ServoID(std::string const& str) {
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
        else {
            throw std::runtime_error("String " + str + " did not match any enum for ServoID");
        }
        // clang-format on
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
}  // namespace utility::input
