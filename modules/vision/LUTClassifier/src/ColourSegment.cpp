/*
 * This file is part of ColourSegment.
 *
 * ColourSegment is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ColourSegment is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ColourSegment.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ColourSegment.h"
 
namespace modules{
    namespace vision{
        using messages::vision::ClassifiedImage::Colour;

        void ColourSegment::set(const arma::vec2& start, const arma::vec2& end, const Colour& colour) {
            m_colour = colour;

            m_start = start;
            m_end = end;

            m_length_pixels = arma::norm((start - end), 2);     // Length of the vector between the two points.
        //    m_length_pixels = (m_start - m_end).abs();        // original code.

            m_centre = (m_start + m_end) * 0.5;
        }

        void ColourSegment::setColour(const Colour& colour) {
            m_colour = colour;
        }

        bool ColourSegment::join(const ColourSegment &other) {
            //colours don't match - segments cannot be joined
            if (m_colour != other.m_colour)
                return false;

            if ((m_start[0] == other.m_end[0]) && (m_start[1] == other.m_end[1])) {
                m_start = other.m_start;
            }

            else if ((m_end[0] == other.m_start[0]) && (m_end[1] == other.m_start[1])) { 
                m_end = other.m_end;
            }

            //there are no matching endpoints
            else {
                return false;
            }

            m_length_pixels = arma::norm((m_start - m_end), 2);     // Length of the vector between the two points.
        //    m_length_pixels = (m_start - m_end).abs();        // original code.

            m_centre = (m_start + m_end) * 0.5;

            return true;
        }

        /*! @brief Stream insertion operator for a single ColourSegment.
         *      The segment is terminated by a newline.
         */
        std::ostream& operator<< (std::ostream& output, const ColourSegment& c) {
            output << c.m_start << " - " << c.m_end << " length(pixels): " << c.m_length_pixels << " colour: " << messages::vision::ClassifiedImage::getColourName(c.m_colour) << std::endl;

            return output;
        }

        /*! @brief Stream insertion operator for a vector of ColourSegments.
         *      Each segment is terminated by a newline.
         *  @relates ColourSegment
         */
        std::ostream& operator<< (std::ostream& output, const std::vector<ColourSegment>& c) {
            for (size_t i = 0; i < c.size(); i++)
                output << c[i];

            return output;
        }
    }
}
