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

/**
 *       @name LookUpTable
 *       @file lookuptable.h
 *       @brief Wraps LUT buffer with access methods for pixel classification
 *       @author Shannon Fenn
 *       @date 17-02-12
 *       @note ported by Jake Fountain Dec 2013 to NUClear system
 */

#ifndef MESSAGES_VISION_LOOKUPTABLE_H
#define MESSAGES_VISION_LOOKUPTABLE_H

#include <memory>
#include <string>
#include <cmath>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include "messages/input/Image.h"

namespace messages {
    namespace vision {

        enum Colour : char {
            // Main classifications
            UNCLASSIFIED = 'u',
            WHITE        = 'w',
            GREEN        = 'g',
            ORANGE       = 'o',
            YELLOW       = 'y',
            CYAN         = 'c',
            MAGENTA      = 'm',

            // Ambiguous Classifications
            WHITE_GREEN  = 'f'
        };

        class LookUpTable {
        public:
            uint8_t BITS_Y;
            uint8_t BITS_CB;
            uint8_t BITS_CR;
            size_t LUT_SIZE; //!< The size of a lookup table in bytes.

            LookUpTable();
            LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr, std::vector<char>&& data);

            std::string getData() const;

            /*!
                @brief Classifies a pixel
                @param p the pixel
                @return Returns the colour classification of this pixel
             */
            messages::vision::Colour classify(const messages::input::Image::Pixel& p) const;
        private:

            uint8_t BITS_Y_REMOVED;
            uint8_t BITS_CB_REMOVED;
            uint8_t BITS_CR_REMOVED;
            uint8_t BITS_CB_CR;

            /*!
             *   @brief Gets the index of the pixel in the LUT
             *   @param p The pixel to be classified.
             *   @return Returns the colour index for the given pixel.
             */
            uint getLUTIndex(const messages::input::Image::Pixel& colour) const;
            std::vector<char> data;
        };

        struct SaveLookUpTable {
        };

    } //vision
} // messages

// YAML conversions
namespace YAML {

    template<>
    struct convert<messages::vision::LookUpTable> {
        static Node encode(const messages::vision::LookUpTable& rhs) {
            Node node;

            node["bits"]["y"] = uint(rhs.BITS_Y);
            node["bits"]["cb"] = uint(rhs.BITS_CB);
            node["bits"]["cr"] = uint(rhs.BITS_CR);

            node["lut"] = rhs.getData();

            return node;
        }

        static bool decode(const Node& node, messages::vision::LookUpTable& rhs) {

            uint8_t bitsY = node["bits"]["y"].as<uint>();
            uint8_t bitsCb = node["bits"]["cb"].as<uint>();
            uint8_t bitsCr = node["bits"]["cr"].as<uint>();

            std::string dataString = node["lut"].as<std::string>();
            std::vector<char> data(dataString.begin(), dataString.end());

            rhs = messages::vision::LookUpTable(bitsY, bitsCb, bitsCr, std::move(data));

            return true;
        }
    };
}

#endif // MESSAGES_VISION_LOOKUPTABLE_H
