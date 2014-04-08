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

#include "LookUpTable.h"
#include <fstream>
#include <cstring>

namespace messages {
    namespace vision {

        void LookUpTable::loadLUTFromFile(const std::string& fileName) {
            std::ifstream lutfile(fileName, std::ios::binary);
            std::copy(std::istreambuf_iterator<char>(lutfile), std::istreambuf_iterator<char>(), std::begin(LUT));
        }

        void LookUpTable::loadLUTFromArray(const char* array) {
            std::memcpy(&LUT, array, LUT_SIZE);
        }

        messages::vision::Colour LookUpTable::classifyPixel(const messages::input::Image::Pixel& p) const {
            return messages::vision::Colour(LUT[getLUTIndex(p)]); // 7bit LUT
        }

        uint LookUpTable::getLUTIndex(const messages::input::Image::Pixel& colour) {
            unsigned int index = 0;

            index += ((colour.y >> (8 - BITS_PER_COLOUR)) << 2 * BITS_PER_COLOUR);
            index += ((colour.cb >> (8 - BITS_PER_COLOUR)) << BITS_PER_COLOUR);
            index += (colour.cr >> (8 - BITS_PER_COLOUR));

            return index;
        }

        void LookUpTable::save(const std::string& fileName) const {
            std::ofstream lutfile(fileName, std::ios::binary);
            std::copy(std::begin(LUT), std::end(LUT), std::ostreambuf_iterator<char>(lutfile));
        }

    } //vision
} // messages
