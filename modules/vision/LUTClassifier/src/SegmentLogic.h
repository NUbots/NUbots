/*
 * This file is part of SementLogic.
 *
 * SementLogic is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SementLogic is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SementLogic.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_SEGMENTLOGIC_H
#define MODULES_VISION_SEGMENTLOGIC_H

#include <nuclear>
#include <vector>
#include <armadillo>

#include "messages/vision/ClassifiedImage.h"

namespace modules{
	namespace vision{
		std::ostream& operator<< (std::ostream& output, const messages::vision::ColourSegment& c);
        std::ostream& operator<< (std::ostream& output, const std::vector<messages::vision::ColourSegment>& c);
		bool operator== (const messages::vision::ColourSegment& lhs, const messages::vision::ColourSegment& rhs);
		class SegmentLogic{
		public:
			SegmentLogic();

        	static void setColourSegment(messages::vision::ColourSegment& colourSegment, const arma::vec2& start, const arma::vec2& end, const messages::vision::Colour& colour);
	        static bool joinColourSegment(messages::vision::ColourSegment& colourSegment, const messages::vision::ColourSegment& other);
		};
	}
}
#endif