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

        Image::Image(size_t width, size_t height, std::vector<Pixel>&& data)
            : imgWidth(width)
            , imgHeight(height)
            , data(std::move(data)) {
        }

        Image::Image(size_t width, size_t height, std::vector<Pixel>&& data, std::vector<uint8_t>&& source)
            : imgWidth(width)
            , imgHeight(height)
            , data(std::move(data))
            , src(std::move(source)) {
        }

        Image::Pixel& Image::operator ()(size_t x, size_t y) {
            return data[y * imgWidth + x];
        }

        const Image::Pixel& Image::operator ()(size_t x, size_t y) const {
            return data[y * imgWidth + x];
        }

        size_t Image::width() const {
            return imgWidth;
        }

        size_t Image::height() const {
            return imgHeight;
        }

        const std::vector<Image::Pixel>& Image::raw() const {
            return data;
        }

        const std::vector<uint8_t>& Image::source() const {
            return src;
        }

    }  // input
}  // messages
