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

#ifndef UTILITY_VISION_COLOURSEGMENT_H
#define UTILITY_VISION_COLOURSEGMENT_H

#include <string>
#include <vector>
#include <armadillo>
#include <ostream>

#include "ColourClassification.h"

namespace utility {

    /**
     * Provides some simple routines for the generation of classified images.
     *
     * @author Alex Biddulph
     */
    namespace vision {

        /**
         * The possible alignment for segments in a region.
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
            std::vector<std::vector<ColourSegment>> m_segmentedScans;       //! @variable The segments in this region.
            ScanDirection m_direction;                                      //! @variable The alignment of the scans in this region.
        } SegmentedRegion;

        void setColourSegment(ColourSegment& colourSegment, const arma::vec2& start, const arma::vec2& end, const Colour& colour) {
            colourSegment.m_colour = colour;

            colourSegment.m_start = start;
            colourSegment.m_end = end;

            colourSegment.m_lengthPixels = arma::norm((colourSegment.m_start - colourSegment.m_end), 2);            // Length of the vector between the two points.
            colourSegment.m_centre = (colourSegment.m_start + colourSegment.m_end) * 0.5;                           // Midpoint of the vector.
        }

        bool joinColourSegment(ColourSegment& colourSegment, const ColourSegment& other) {
            // Colours don't match - segments cannot be joined
            if (colourSegment.m_colour != other.m_colour) {
                return false;
            }

            if ((colourSegment.m_start[0] == other.m_end[0]) && (colourSegment.m_start[1] == other.m_end[1])) {
                colourSegment.m_start = other.m_start;
            }

            else if ((colourSegment.m_end[0] == other.m_start[0]) && (colourSegment.m_end[1] == other.m_start[1])) { 
                colourSegment.m_end = other.m_end;
            }

            //there are no matching endpoints
            else {
                return false;
            }

            colourSegment.m_lengthPixels = arma::norm((colourSegment.m_start - colourSegment.m_end), 2);            // Length of the vector between the two points.
            colourSegment.m_centre = (colourSegment.m_start + colourSegment.m_end) * 0.5;                           // Midpoint of the vector.

            return true;
        }

        /*! @brief Stream insertion operator for a single ColourSegment.
         *      The segment is terminated by a newline.
         */
        std::ostream& operator<< (std::ostream& output, const ColourSegment& c) {
            output << c.m_start << " - " << c.m_end << " length(pixels): " << c.m_lengthPixels << " colour: " << getColourName(c.m_colour) << std::endl;

            return output;
        }

        /*! @brief Stream insertion operator for a vector of ColourSegments.
         *      Each segment is terminated by a newline.
         *  @relates ColourSegment
         */
        std::ostream& operator<< (std::ostream& output, const std::vector<ColourSegment>& c) {
            for (auto it : c)
                output << it;

            return output;
        }

        //! only used for ransac - segments cannot overlap and thus cannot have equal centres
        bool operator== (const ColourSegment& lhs, const ColourSegment& rhs) {
            // Compare both elements in both centre points for equality.
            return ((lhs.m_centre[0] == rhs.m_centre[0]) && (lhs.m_centre[1] == rhs.m_centre[1]));
        } 
    }
}

#endif // UTILITY_VISION_COLOURSEGMENT_H