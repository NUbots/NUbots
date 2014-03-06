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
            static constexpr const int BITS_PER_COLOUR = 7;
            static constexpr const int LUT_SIZE = std::pow(2, 3 * BITS_PER_COLOUR); //!< The size of a lookup table in bytes.

            /*!
                @brief Loads a new LUT from a given file.
                @param filename The filename std::string.
                @return Returns the success of the operation.
             */
            void loadLUTFromFile(const std::string& fileName);
            void loadLUTFromArray(const char* array);
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
