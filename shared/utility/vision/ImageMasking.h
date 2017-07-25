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
#ifndef UTILITY_VISION_IMAGE_MASKING_H
#define UTILITY_VISION_IMAGE_MASKING_H

#include "message/vision/ImageMask.h"

namespace utility {
namespace vision {

    struct MaskClass {
        enum Value : char {
            // Main classifications
            UNMASKED = 'u',
            MASKED   = 'm'
        };
        Value value;

        // Constructors
        MaskClass() : value(Value::UNMASKED) {}
        MaskClass(int const& value) : value(static_cast<Value>(value)) {}
        MaskClass(uint8_t const& value) : value(static_cast<Value>(value)) {}
        MaskClass(uint32_t const& value) : value(static_cast<Value>(value)) {}
        MaskClass(char const& value) : value(static_cast<Value>(value)) {}
        MaskClass(Value const& value) : value(value) {}


        // Operators
        bool operator<(MaskClass const& other) const {
            return value < other.value;
        }
        bool operator>(MaskClass const& other) const {
            return value > other.value;
        }
        bool operator<=(MaskClass const& other) const {
            return value <= other.value;
        }
        bool operator>=(MaskClass const& other) const {
            return value >= other.value;
        }
        bool operator==(MaskClass const& other) const {
            return value == other.value;
        }
        bool operator!=(MaskClass const& other) const {
            return value != other.value;
        }
        bool operator<(MaskClass::Value const& other) const {
            return value < other;
        }
        bool operator>(MaskClass::Value const& other) const {
            return value > other;
        }
        bool operator<=(MaskClass::Value const& other) const {
            return value <= other;
        }
        bool operator>=(MaskClass::Value const& other) const {
            return value >= other;
        }
        bool operator==(MaskClass::Value const& other) const {
            return value == other;
        }
        bool operator!=(MaskClass::Value const& other) const {
            return value != other;
        }
    };

    inline bool pixelIsMasked(const int& x, const int& y, std::shared_ptr<const message::vision::ImageMask> mask) {
        return mask && mask->values(x, y) == MaskClass::MASKED;
    }

}  // namespace vision
}  // namespace utility

#endif  // MESSAGE_VISION_VISION_H
