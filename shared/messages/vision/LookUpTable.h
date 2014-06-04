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
#include "messages/input/Image.h"

namespace messages {
    namespace vision {

        enum Colour {
            unclassified, //!< Colour has not be given a category.
            white, //!< Colour is in the White region.
            green, //!< Colour is in the Green region.
            shadow_object, //!< Colour is part of a shadowed area.
            pink, //!< Colour is in the Red region.
            pink_orange, //!< Colour is in the region of overlap between Red and Orange.
            orange, //!< Colour is in the Orange region.
            yellow_orange, //!< Colour is in the region of overlap between Yellow and Orange.
            yellow, //!< Colour is in the Yellow region.
            blue, //!< Colour is in the Sky Blue region.
            shadow_blue, //!< Colour is in the Dark Blue region.
            num_colours, //!< Total number of colour categories.
            invalid
        };

        enum COLOUR_CLASS {
            BALL_COLOUR,
            GOAL_COLOUR,
            // GOAL_Y_COLOUR,
            // GOAL_B_COLOUR,
            LINE_COLOUR,
            TEAM_CYAN_COLOUR,
            TEAM_MAGENTA_COLOUR,
            FIELD_COLOUR,
            UNKNOWN_COLOUR
        };

        class LookUpTable {
        public:
            const uint8_t BITS_Y;
            const uint8_t BITS_CB;
            const uint8_t BITS_CR;
            const size_t LUT_SIZE; //!< The size of a lookup table in bytes.

            LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr);
            LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr, std::unique_ptr<char[]>&& data);
            LookUpTable(std::string& filename);

            void save(const std::string& fileName) const;
            std::string getData() const {
                return std::string(data.get(), LUT_SIZE);
            }
            /*!
                @brief Classifies a pixel
                @param p the pixel
                @return Returns the colour classification of this pixel
             */
            messages::vision::Colour classifyPixel(const messages::input::Image::Pixel& p) const;
        private:
            LookUpTable(std::tuple<uint8_t, uint8_t, uint8_t, std::unique_ptr<char[]>> data);

            const uint8_t BITS_Y_REMOVED;
            const uint8_t BITS_CB_REMOVED;
            const uint8_t BITS_CR_REMOVED;
            const uint8_t BITS_CB_CR;

            /*!
             *   @brief Gets the index of the pixel in the LUT
             *   @param p The pixel to be classified.
             *   @return Returns the colour index for the given pixel.
             */
            uint getLUTIndex(const messages::input::Image::Pixel& colour) const;
            static std::tuple<uint8_t, uint8_t, uint8_t, std::unique_ptr<char[]>> createLookUpTableFromFile(std::string& filename);
            std::unique_ptr<char[]> data;
        };

    } //vision
} // messages

#endif // MESSAGES_VISION_LOOKUPTABLE_H
