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
