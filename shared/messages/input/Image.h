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

#ifndef MESSAGES_INPUT_IMAGE_H
#define MESSAGES_INPUT_IMAGE_H

#include <nuclear>
#include <armadillo>
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

            Image(uint width, uint height, NUClear::clock::time_point, std::vector<uint8_t>&& data);

            Pixel operator()(uint x, uint y) const;
            Pixel operator()(const arma::ivec2& p) const;

            uint width;
            uint height;
            NUClear::clock::time_point timestamp;

            // Returns the raw data that this is using
            const std::vector<uint8_t>& source() const;

        private:
            std::vector<uint8_t> data;
        };

    }  // input
}  // messages

#endif  // MESSAGES_INPUT_IMAGE_H
