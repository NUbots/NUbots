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

#ifndef UTILITY_INPUT_LIMBID_HPP
#define UTILITY_INPUT_LIMBID_HPP

#include <set>

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

        static std::set<ServoID> servosForLimb(const LimbID& limb);
        static LimbID limbForServo(const ServoID& servo);

        friend std::ostream& operator<<(std::ostream& out, const LimbID& val);
    };
}  // namespace utility::input

#endif
