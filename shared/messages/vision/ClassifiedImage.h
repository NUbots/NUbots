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
#include <vector>
#include <map>
#include <armadillo>
#include "utility/vision/LookUpTable.h"
#include "messages/input/Image.h"

namespace messages {
    namespace vision {
           
        /**
        * The possible alignment for segments in a segmented region.
        */     
        enum ScanDirection {
            VERTICAL,
            HORIZONTAL
        };

        typedef struct {
            Colour m_colour;
            unsigned int m_lengthPixels;
            arma::vec2 m_start;             //! @variable The start pixel location.
            arma::vec2 m_end;               //! @variable The end  pixellocation.
            arma::vec2 m_centre;            //! @variable The centre pixellocation.
        } ColourSegment;

        typedef struct {
            std::vector<std::vector<ColourSegment> > m_segmentedScans;       //! @variable The segments in this region.
            ScanDirection m_direction;                                      //! @variable The alignment of the scans in this region.
        } SegmentedRegion;

        /**
         * This class contains all information about a classified image.
         *
         * @author Jake Fountain
         */
        class ClassifiedImage {
        public:

            ClassifiedImage(){}
            /**
             * Provides some simple routines for handling colours.
             *
             * @author Alex Biddulph
             */
              //Image variables:
            SegmentedRegion horizontalFilteredSegments;       //! @variable The filtered segmented horizontal scanlines.
            SegmentedRegion verticalFilteredSegments;         //! @variable The filtered segmented vertical scanlines.

            //! Transitions
            std::map<COLOUR_CLASS, std::vector<ColourSegment>> matchedHorizontalSegments;
            std::map<COLOUR_CLASS, std::vector<ColourSegment>> matchedVerticalSegments;

            std::vector<arma::vec2> greenHorizonInterpolatedPoints;

            std::shared_ptr<const messages::input::Image> image;         //@! The image from which the segments are derived.

            std::shared_ptr<utility::vision::LookUpTable> LUT;
             

            /*!
            Gets the name of the given colour.
            @param colour The colour name desired.
            @return The name of the colour.
            */
            static std::string getColourName(const Colour& colour) {
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
            static Colour getColourFromName(const std::string& name) {
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
            static COLOUR_CLASS getColourClassFromName(const std::string& name) {
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
            static std::string getColourClassName(const COLOUR_CLASS& id) {
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
            static COLOUR_CLASS getClassOfColour(const Colour& c) {
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

                    case green: {
                        return FIELD_COLOUR;
                    }

                    default: {
                        return UNKNOWN_COLOUR;
                    }
                }
            }
            
           
        };
        
    }  // vision
}  // messages

#endif  // MESSAGES_VISION_CLASSIFIEDIMAGE_H
