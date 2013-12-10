/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_VISION_CLASSIFIEDIMAGE_H
#define MESSAGES_VISION_CLASSIFIEDIMAGE_H

#include <string>

namespace messages {
    namespace vision {

        /**
         * This class contains all information about a classified image.
         *
         * @author Jake Fountain
         */
        class ClassifiedImage {
        public:
            enum Colour{
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
                UNKNOWN_COLOUR
            };

            ClassifiedImage();
			
            /*!
            Gets the name of the given colour.
            @param colour The colour name desired.
            @return The name of the colour.
            */
            static std::string getColourName(const Colour& colour);

			/*!
			  Gets the colour matching given name.
			  @param name String name of the colour desired.
			  @return The method mathing the given name.
			  */
			static Colour getColourFromName(const std::string& name);
            
			//! @brief converts a string into a colour class.
			static COLOUR_CLASS getColourClassFromName(const std::string& name);

			//! @brief converts a colour class into a string.
			static std::string getColourClassName(const COLOUR_CLASS& id);
			
        private:
            
        };
        
    }  // vision
}  // messages

#endif  // MESSAGES_VISION_CLASSIFIEDIMAGE_H
