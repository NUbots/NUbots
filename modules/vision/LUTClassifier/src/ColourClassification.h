/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_COLOURSCLASSIFICATION_H
#define MODULES_VISION_COLOURSCLASSIFICATION_H

#include <string>

#include "messages/vision/ClassifiedImage.h"

namespace modules {
    namespace vision {

        using namespace messages::vision;

        /**
         * Provides some simple routines for handling colours.
         *
         * @author Alex Biddulph
         */
         
        /*!
        Gets the name of the given colour.
        @param colour The colour name desired.
        @return The name of the colour.
        */
        std::string getColourName(const Colour& colour) {
            switch (colour) {
                case unclassified: {
                    return "unclassified";
                }

                case white: {
                    return "white";
                }

                case green: {
                    return "green";
                }

                case shadow_object: {
                    return "shadow-object";
                }

                case pink: {
                    return "pink";
                }

                case pink_orange: {
                    return "pink-orange";
                }

                case orange: {
                    return "orange";
                }

                case yellow_orange: {
                    return "yellow-orange";
                }

                case yellow: {
                    return "yellow";
                }

                case blue: {
                    return "blue";
                }

                case shadow_blue: {
                    return "shadow-blue";
                }

                default: {
                    return "unknown colour!";
                }
            }
        }

        /*!
          Gets the colour matching given name.
          @param name String name of the colour desired.
          @return The method mathing the given name.
          */
        Colour getColourFromName(const std::string& name) {
            if (name.compare("unclassified") == 0) {
                return unclassified;
            }
            
            else if (name.compare("white") == 0) {
                return white;
            }
            
            else if (name.compare("green") == 0) {
                return green;
            }
            
            else if (name.compare("shadow-object") == 0) {
                return shadow_object;
            }
            
            else if (name.compare("pink") == 0) {
                return pink;
            }
            
            else if (name.compare("pink-orange") == 0) {
                return pink_orange;
            }
            
            else if (name.compare("orange") == 0) {
                return orange;
            }
            
            else if (name.compare("yellow-orange") == 0) {
                return yellow_orange;
            }
            
            else if (name.compare("yellow") == 0) {
                return yellow;
            }
            
            else if (name.compare("blue") == 0) {
                return blue;
            }
            
            else if (name.compare("shadow-blue") == 0) {
                return shadow_blue;
            }
            
            else {
                return invalid;
            }
        }
        
        //! @brief converts a string into a colour class.
        COLOUR_CLASS getColourClassFromName(const std::string& name) {
            if (name.compare("BALL_COLOUR") == 0) {
                return BALL_COLOUR;
            }
            
            else if (name.compare("GOAL_COLOUR") == 0) {
                return GOAL_COLOUR;
            }
            
/*          
            else if (name.compare("GOAL_Y_COLOUR") == 0) {
                return GOAL_Y_COLOUR;
            }
            
            else if (name.compare("GOAL_B_COLOUR") == 0) {
                return GOAL_B_COLOUR;
            }
*/
        
            else if (name.compare("LINE_COLOUR") == 0) {
                return LINE_COLOUR;
            }
                
            else {
                return UNKNOWN_COLOUR;
            }
        }

        //! @brief converts a colour class into a string.
        std::string getColourClassName(const COLOUR_CLASS& id) {
            switch (id) {
                case BALL_COLOUR: {
                    return "BALL_COLOUR";
                }
                    
                case GOAL_COLOUR: {
                    return "GOAL_COLOUR";
                }

/*              
                case GOAL_Y_COLOUR: {
                    return "GOAL_Y_COLOUR";
                }
                    
                case GOAL_B_COLOUR: {
                    return "GOAL_B_COLOUR";
                }
*/

                case LINE_COLOUR: {
                    return "LINE_COLOUR";
                }
                    
                default: {
                    return "UNKNOWN_COLOUR";
                }
            }
        }    

        //! gets the colour class corresponding to the colour
        COLOUR_CLASS getClassOfColour(const Colour& c) {
            switch (c) {
                case yellow: {
                    return GOAL_COLOUR;
                }

                case orange: {
                    return BALL_COLOUR;
                }

                case white: {
                    return LINE_COLOUR;
                }

                case blue: {
                    return TEAM_CYAN_COLOUR;
                }

                case pink: {
                    return TEAM_MAGENTA_COLOUR;
                }

                default: {
                    return UNKNOWN_COLOUR;
                }
            }
        }

    }
}

#endif // MODULES_VISION_COLOURSCLASSIFICATION_H