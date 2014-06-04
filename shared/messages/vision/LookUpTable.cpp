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


        LookUpTable::LookUpTable(std::tuple<uint8_t, uint8_t, uint8_t, std::unique_ptr<char[]>> data)
            : LookUpTable(std::get<0>(data), std::get<1>(data), std::get<2>(data), std::move(std::get<3>(data))) {
        }

        LookUpTable::LookUpTable(std::string& filename) : LookUpTable(createLookUpTableFromFile(filename)) {
        }

        std::tuple<uint8_t, uint8_t, uint8_t, std::unique_ptr<char[]>> LookUpTable::createLookUpTableFromFile(std::string& filename) {
            // read
            std::ifstream lutfile(filename, std::ios::binary);
            auto input = std::istreambuf_iterator<char>(lutfile);
            uint8_t bitsY = *(input++) - 48;  // Convert the string to a number
            uint8_t bitsCb = *(input++) - 48; // Convert the string to a number
            uint8_t bitsCr = *(input++) - 48; // Convert the string to a number
            const size_t size = std::exp2(bitsY + bitsCb + bitsCr);
            auto data = std::unique_ptr<char[]>(new char[size]);
            std::copy(input, std::istreambuf_iterator<char>(), data.get());
            return std::make_tuple(bitsY, bitsCb, bitsCr, std::move(data));
        }

        messages::vision::Colour LookUpTable::classify(const messages::input::Image::Pixel& p) const {
            return messages::vision::Colour(data[getLUTIndex(p)]); // 7bit LUT
        }

        uint LookUpTable::getLUTIndex(const messages::input::Image::Pixel& colour) const {
            unsigned int index = 0;

            index += ((colour.y >> BITS_Y_REMOVED) << BITS_CB_CR);
            index += ((colour.cb >> BITS_CB_REMOVED) << BITS_CR);
            index += (colour.cr >> BITS_CR_REMOVED);

            return index;
        }

        void LookUpTable::save(const std::string& fileName) const {
            std::ofstream lutfile(fileName, std::ios::binary);
            auto output = std::ostreambuf_iterator<char>(lutfile);
            output = BITS_Y + 48;  // Convert the number to a string
            output = BITS_CB + 48; // Convert the number to a string
            output = BITS_CR + 48; // Convert the number to a string
            std::copy(data.get(), data.get() + LUT_SIZE, output);
        }

    } //vision
} // messages
