/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef UTILITY_INPUT_FRAMEID_HPP
#define UTILITY_INPUT_FRAMEID_HPP

#include <set>
#include <string>

namespace utility::input {

    struct FrameID {
        enum Value {
            R_SHOULDER_PITCH = 0,
            L_SHOULDER_PITCH = 1,
            R_SHOULDER_ROLL  = 2,
            L_SHOULDER_ROLL  = 3,
            R_ELBOW          = 4,
            L_ELBOW          = 5,
            R_HIP_YAW        = 6,
            L_HIP_YAW        = 7,
            R_HIP_ROLL       = 8,
            L_HIP_ROLL       = 9,
            R_HIP_PITCH      = 10,
            L_HIP_PITCH      = 11,
            R_KNEE           = 12,
            L_KNEE           = 13,
            R_ANKLE_PITCH    = 14,
            L_ANKLE_PITCH    = 15,
            R_ANKLE_ROLL     = 16,
            L_ANKLE_ROLL     = 17,
            HEAD_YAW         = 18,
            HEAD_PITCH       = 19,
            L_FOOT_BASE      = 20,
            R_FOOT_BASE      = 21,
            L_CAMERA         = 22,
            R_CAMERA         = 23,
            NUMBER_OF_LINKS  = 24
        };
        Value value = Value::R_SHOULDER_PITCH;

        // Constructors
        FrameID() = default;
        FrameID(uint8_t const& v) : value(static_cast<Value>(v)) {}
        FrameID(uint32_t const& v) : value(static_cast<Value>(v)) {}
        FrameID(uint64_t const& v) : value(static_cast<Value>(v)) {}
        FrameID(int const& v) : value(static_cast<Value>(v)) {}
        FrameID(Value const& v) : value(v) {}
        FrameID(std::string const& str);

        // Operators
        bool operator<(FrameID const& other) const {
            return value < other.value;
        }
        bool operator>(FrameID const& other) const {
            return value > other.value;
        }
        bool operator<=(FrameID const& other) const {
            return value <= other.value;
        }
        bool operator>=(FrameID const& other) const {
            return value >= other.value;
        }
        bool operator==(FrameID const& other) const {
            return value == other.value;
        }
        bool operator!=(FrameID const& other) const {
            return value != other.value;
        }
        bool operator<(FrameID::Value const& other) const {
            return value < other;
        }
        bool operator>(FrameID::Value const& other) const {
            return value > other;
        }
        bool operator<=(FrameID::Value const& other) const {
            return value <= other;
        }
        bool operator>=(FrameID::Value const& other) const {
            return value >= other;
        }
        bool operator==(FrameID::Value const& other) const {
            return value == other;
        }
        bool operator!=(FrameID::Value const& other) const {
            return value != other;
        }

        // Conversions
        operator Value() const {
            return value;
        }
        operator uint8_t() const {
            return value;
        }
        operator uint32_t() const {
            return value;
        }
        operator uint64_t() const {
            return value;
        }
        operator int() const {
            return value;
        }
        operator std::string() const;

        friend std::ostream& operator<<(std::ostream& out, const FrameID& val);

    private:
        static const std::set<FrameID> values;
    };
}  // namespace utility::input

#endif  // UTILITY_INPUT_FRAMEID_HPP
