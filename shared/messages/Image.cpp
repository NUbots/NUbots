/*!
  @file Image.cpp
  @brief Implementation of the Image class.
  @author Steven Nicklin

  Copyright (c) 2009 Jason Kulk

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Image.h"
#include <cstring>

namespace messages {

    Image::Image(size_t width, size_t height, std::unique_ptr<Pixel[]>&& data) :
    imgWidth(width),
    imgHeight(height),
    data(std::move(data)) {
    }

    Image::Pixel& Image::operator ()(size_t x, size_t y) {
        return data.get()[y * imgWidth + x];
    }

    const Image::Pixel& Image::operator ()(size_t x, size_t y) const {
        return data.get()[y * imgWidth + x];
    }

    const size_t Image::width() const {
        return imgWidth;
    }

    const size_t Image::height() const {
        return imgHeight;
    }

    const size_t Image::size() const {
        return imgWidth * imgHeight;
    }
}
