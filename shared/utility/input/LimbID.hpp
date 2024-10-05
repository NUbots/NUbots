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

#ifndef UTILITY_INPUT_LIMBID_HPP
#define UTILITY_INPUT_LIMBID_HPP

#include <set>

#include "message/actuation/Servos.hpp"

#include "utility/input/ServoID.hpp"

namespace utility::input {
    // LimbID is a higher level of ServoID (see ServoID.h),
    // which contains all the constituent servos (e.g. An arm contains shoulder (pitch + roll)) and elbow.
    using ServoID = utility::input::ServoID;

    struct LimbID {
        enum Value { UNKNOWN = 0, LEFT_LEG = 1, RIGHT_LEG = 2, LEFT_ARM = 3, RIGHT_ARM = 4, HEAD = 5 };
        Value value = Value::UNKNOWN;

        // Constructors
        LimbID() = default;
        LimbID(uint8_t const& v) : value(static_cast<Value>(v)) {}
        LimbID(uint32_t const& v) : value(static_cast<Value>(v)) {}
        LimbID(uint64_t const& v) : value(static_cast<Value>(v)) {}
        LimbID(int const& v) : value(static_cast<Value>(v)) {}
        LimbID(Value const& v) : value(v) {}
        LimbID(std::string const& str);

        // Operators
        bool operator<(LimbID const& other) const {
            return value < other.value;
        }
        bool operator>(LimbID const& other) const {
            return value > other.value;
        }
        bool operator<=(LimbID const& other) const {
            return value <= other.value;
        }
        bool operator>=(LimbID const& other) const {
            return value >= other.value;
        }
        bool operator==(LimbID const& other) const {
            return value == other.value;
        }
        bool operator!=(LimbID const& other) const {
            return value != other.value;
        }
        bool operator<(LimbID::Value const& other) const {
            return value < other;
        }
        bool operator>(LimbID::Value const& other) const {
            return value > other;
        }
        bool operator<=(LimbID::Value const& other) const {
            return value <= other;
        }
        bool operator>=(LimbID::Value const& other) const {
            return value >= other;
        }
        bool operator==(LimbID::Value const& other) const {
            return value == other;
        }
        bool operator!=(LimbID::Value const& other) const {
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

        static std::set<ServoID> servos_for_limb(const LimbID& limb);
        static std::set<ServoID> servos_for_legs();
        static std::set<ServoID> servos_for_arms();
        static LimbID limb_for_servo(const ServoID& servo);

        friend std::ostream& operator<<(std::ostream& out, const LimbID& val);
    };

}  // namespace utility::input

#endif
