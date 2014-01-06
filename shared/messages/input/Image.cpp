/*
 * This file is part of LinuxCameraStreamer.
 *
 * LinuxCameraStreamer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LinuxCameraStreamer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LinuxCameraStreamer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Image.h"
#include <cstring>

namespace messages {
    namespace input {

        Image::Image(size_t width, size_t height, std::vector<Pixel>&& data, bool flipped_) :
        imgWidth(width),
        imgHeight(height),
        flipped(flipped_),
        data(std::move(data)){
        }
        
        Image::Image(size_t width, size_t height, std::vector<Pixel>&& data, std::vector<uint8_t>&& source, bool flipped_) :
        imgWidth(width),
        imgHeight(height),
        flipped(flipped_),
        data(std::move(data)),
        src(std::move(source)) {
        }

        Image::Pixel& Image::operator ()(size_t x, size_t y) {
            int new_y = y;
            int new_x = x;
            if(flipped){
                new_y = imgHeight-y-1;
                new_x = imgWidth-x-1;
            }
            return data[new_y*imgWidth+new_x];            
        }

        const Image::Pixel& Image::operator ()(size_t x, size_t y) const {
            int new_y = y;
            int new_x = x;
            if(flipped){
                new_y = imgHeight-y-1;
                new_x = imgWidth-x-1;
            }
            return data[new_y*imgWidth+new_x];     
        }

        const size_t Image::width() const {
            return imgWidth;
        }

        const size_t Image::height() const {
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
