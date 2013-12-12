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

#include "utility/vision/ColourClassification.h"
#include "utility/vision/ColourSegment.h"

namespace messages {
    namespace vision {

        /**
         * This class contains all information about a classified image.
         *
         * @author Jake Fountain
         */
        class ClassifiedImage {
        public:
            ClassifiedImage();

/*
            void setFilteredSegments(const  SegmentedRegion& hor, const  SegmentedRegion& vert);
            void setTransitionsMaps(const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& hor, 
                                    const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& vert);

            const SegmentedRegion& getHorizontalFilteredSegments();
            const SegmentedRegion& getVerticalFilteredSegments();
            const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& getHorizontalTransitionsMap();
            const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& getVerticalTransitionsMap();
*/
        
            //Image variables:
            utility::vision::SegmentedRegion horizontal_filtered_segments;       //! @variable The filtered segmented horizontal scanlines.
            utility::vision::SegmentedRegion vertical_filtered_segments;         //! @variable The filtered segmented vertical scanlines.

            //! Transitions
            std::map<utility::vision::COLOUR_CLASS, std::vector<utility::vision::ColourSegment>> matched_horizontal_segments;
            std::map<utility::vision::COLOUR_CLASS, std::vector<utility::vision::ColourSegment>> matched_vertical_segments;

//            GreenHorizon green_horizon;
        };
        
    }  // vision
}  // messages

#endif  // MESSAGES_VISION_CLASSIFIEDIMAGE_H
