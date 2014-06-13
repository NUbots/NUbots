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
#include <iostream>

namespace messages {
    namespace vision {

        LookUpTable::LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr, std::unique_ptr<char[]>&& data)
            : BITS_Y(bitsY)
            , BITS_CB(bitsCb)
            , BITS_CR(bitsCr)
            , LUT_SIZE(1 << (BITS_Y + BITS_CB + BITS_CR))
            , BITS_Y_REMOVED(sizeof(uint8_t) * 8 - BITS_Y)
            , BITS_CB_REMOVED(sizeof(uint8_t) * 8 - BITS_CB)
            , BITS_CR_REMOVED(sizeof(uint8_t) * 8 - BITS_CR)
            , BITS_CB_CR(BITS_CB + BITS_CR)
            , data(std::move(data)) {

        }

        LookUpTable::LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr)
            : LookUpTable(bitsY, bitsCb, bitsCr, std::unique_ptr<char[]>(new char[1 << (bitsY + bitsCb + bitsCr)])) {
            std::fill(data.get(), data.get() + LUT_SIZE, 0);
        }

        messages::vision::Colour LookUpTable::classify(const messages::input::Image::Pixel& p) const {
            return messages::vision::Colour(data[getLUTIndex(p)]); // 7bit LUT
        }

        std::string LookUpTable::getData() const {
            return std::string(data.get(), LUT_SIZE);
        }

        uint LookUpTable::getLUTIndex(const messages::input::Image::Pixel& colour) const {
            unsigned int index = 0;

            index += ((colour.y >> BITS_Y_REMOVED) << BITS_CB_CR);
            index += ((colour.cb >> BITS_CB_REMOVED) << BITS_CR);
            index += (colour.cr >> BITS_CR_REMOVED);

            return index;
        }

    } //vision
} // messages
