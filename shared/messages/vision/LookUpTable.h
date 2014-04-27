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

#include <string>
#include <cmath>
#include <cstring>
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

        enum ColourClass {
            BALL,
            GOAL,
            LINE,
            TEAM_CYAN,
            TEAM_MAGENTA,
            FIELD,
            UNKNOWN
        };

        class LookUpTable {
        public:
            static constexpr const int BITS_PER_COLOUR = 7;
            static constexpr const int LUT_SIZE = std::pow(2, 3 * BITS_PER_COLOUR); //!< The size of a lookup table in bytes.

            /*!
                @brief Loads a new LUT from a given file.
                @param filename The filename std::string.
                @return Returns the success of the operation.
             */
            void loadLUTFromFile(const std::string& fileName);
            void loadLUTFromArray(const char* array);
            void save(const std::string& fileName) const;
            std::string getData() const {
                return std::string(LUT, LUT_SIZE);
            }
            /*!
                @brief Classifies a pixel
                @param p the pixel
                @return Returns the colour classification of this pixel
             */
            messages::vision::Colour classifyPixel(const messages::input::Image::Pixel& p) const;
        private:

            /*!
             *   @brief Gets the index of the pixel in the LUT
             *   @param p The pixel to be classified.
             *   @return Returns the colour index for the given pixel.
             */
            static uint getLUTIndex(const messages::input::Image::Pixel& colour);

            char LUT[LUT_SIZE] = {0}; //! @variable temp LUT for loading.
        };

    } //vision
} // messages

#endif // MESSAGES_VISION_LOOKUPTABLE_H
