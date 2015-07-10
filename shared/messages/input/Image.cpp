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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Image.h"
#include <cstring>

namespace messages {
    namespace input {

        Image::Image(uint width, uint height, NUClear::clock::time_point timestamp, std::vector<uint8_t>&& data)
            : width(width)
            , height(height)
            , timestamp(timestamp)
            , data(std::move(data)) {
        }

        Image::Pixel Image::operator()(uint x, uint y) const {
            int origin = (y * width + x) * 2;
            int shift = (x % 2) * 2;

            return {
                data[origin + 0],
                data[origin + 1 - shift],
                data[origin + 3 - shift]
            };
        }

        Image::Pixel Image::operator()(const arma::ivec2& p) const {
            return operator()(p[0],p[1]);
        }


        const std::vector<uint8_t>& Image::source() const {
            return data;
        }

    }  // input
}  // messages
