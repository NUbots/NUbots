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

    void Image::copyFromYUV422Buffer(const Pixel* buffer, std::size_t width, std::size_t height, bool flip) {
        imageWidth = width / 2;
        imageHeight = height / 2;
        image.reset(new Pixel[imageWidth * imageHeight]);
        if(flip) {
            Pixel* dest = image.get() + imageWidth * imageHeight - 1;
            for(size_t y = 0; y < imageHeight; y++) {
                for(size_t x = 0; x < imageWidth; x++) {
                    *dest-- = *buffer++;
                }
                buffer += imageWidth;
            }
        } else {
            Pixel* dest = image.get();
            for(size_t y = 0; y < imageHeight; y++) {
                for(size_t x = 0; x < imageWidth; x++) {
                    *dest++ = *buffer++;
                }
                buffer += imageWidth;
            }
        }
    }

}
