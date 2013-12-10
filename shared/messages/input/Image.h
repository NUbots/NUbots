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

#ifndef MESSAGES_INPUT_IMAGE_H
#define MESSAGES_INPUT_IMAGE_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace messages {
    namespace input {

        /**
         * TODO document
         *
         * @author Michael Burton
         */
        class Image {
        public:
            struct Pixel {
                uint8_t y;
                uint8_t cb;
                uint8_t cr;
            };

            Image(size_t width, size_t height, std::vector<Pixel>&& data);
            Image(size_t width, size_t height, std::vector<Pixel>&& data, std::vector<uint8_t>&& src);
            
            Pixel& operator()(size_t x, size_t y);

            const Pixel& operator()(size_t x, size_t y) const;
            const size_t width() const;
            const size_t height() const;
            const std::vector<Pixel>& raw() const;
            const std::vector<uint8_t>& source() const;

        private:
            size_t imgWidth;
            size_t imgHeight;
            std::vector<Pixel> data;
            std::vector<uint8_t> src;
        };
        
    }  // input
}  // messages

#endif  // MESSAGES_INPUT_IMAGE_H
