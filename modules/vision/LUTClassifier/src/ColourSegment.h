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

#ifndef MODULES_VISION_COLOURSEGMENT_H
#define MODULES_VISION_COLOURSEGMENT_H

#include <nuclear> 
#include <string>
#include <armadillo>

#include "messages/input/Image.h"
 
namespace modules {
    namespace vision {

        /**
         * Describes a segment of continuous colour in an image.
         *
         * @author Alex Biddulph
         */
        class ColourSegment  {
        private:
            //! @variable The colour of the segment.
            Colour m_colour;

            //! @variable The length of the segment in pixels.
            unsigned int m_length_pixels;

            //! @variable The start pixel location.
            arma::vec2 m_start, 

            //! @variable The end  pixellocation.
            m_end, 

            //! @variable The centre pixellocation.
            m_centre;
           
        public:
            ColourSegment() {
                set(arma::zeros<arma::vec>(2), arma::zeros<arma::vec>(2), invalid);
            }

            ColourSegment(const arma::vec2& start, const arma::vec2& end, const Colour& colour) {
                set(start, end, colour);
            }

            //! Returns the length of the segment in pixels.
            unsigned int getLength() const {
                return m_length_pixels;
            }

            //! Returns the colour of the segment.
            Colour getColour() const {
                return m_colour;
            }

            //! Returns the start location of the segment in pixel coordinates.
            const arma::vec2& getStart() const {
                return m_start;
            }

            //! Returns the end location of the segment in pixel coordinates.
            const arma::vec2& getEnd() const {
                return m_end;
            }

            //! Returns the end location of the segment in pixel coordinates.
            const arma::vec2& getCentre() const {
                return m_centre;
            }

            /**
             * Sets the parameters for the segment.
             * @param start The start location of the segment.
             * @param end The end location of the segment.
             * @param colour The colour of the segment.
             */
            void set(const arma::vec2& start, const arma::vec2& end, const Colour& colour);

            //! Set the colour of the segment.
            void setColour(const Colour& colour);

            /**
             * Joins the given segment to this one. This segment will now be as long as the total
             * length of the two provided the following are true:
             *     - The start of this segment matches the end of the provided segment, or vice versa.
             *     - The colours of the two segments are the same.
             * @param other The segment to join onto this one.
             * @return Whether the segments were joined - will be false if the conditions are not met.
             */
            bool join(const ColourSegment& other);

            //! output stream operator.
            friend std::ostream& operator<<(std::ostream& output, const ColourSegment& c);

            //! output stream operator for a vector of segments.
            friend std::ostream& operator<<(std::ostream& output, const std::vector<ColourSegment>& c);

            //! only used for ransac - segments cannot overlap and thus cannot have equal centres
            friend bool operator==(const ColourSegment& lhs, const ColourSegment& rhs) {
                // Compare both elements in both centre points for equality.
                return ((lhs.m_centre[0] == rhs.m_centre[0]) && (lhs.m_centre[1] == rhs.m_centre[1]));
            } 
    };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_COLOURSEGMENT_H

