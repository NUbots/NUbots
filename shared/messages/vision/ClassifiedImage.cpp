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

#include "ClassifiedImage.h"

namespace messages {
    namespace vision {

        ClassifiedImage::ClassifiedImage() {
		}      
	
	/*	void ClassifiedImage::setFilteredSegments(const  SegmentedRegion& hor, const  SegmentedRegion& vert){
			horizontal_filtered_segments = hor;
			vertical_filtered_segments = vert;
		}
        void ClassifiedImage::setTransitionsMaps(const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& hor, 
                                const std::map<COLOUR_CLASS, std::vector<modules::vision::ColourSegment> >& vert){
        	matched_horizontal_segments = hor;
			matched_vertical_segments = vert;
        }

        const SegmentedRegion& ClassifiedImage::getHorizontalFilteredSegments(){
        	return horizontal_filtered_segments;
        }
        const SegmentedRegion& ClassifiedImage::getVerticalFilteredSegments(){
	    	return vertical_filtered_segments;
        }
        const std::map<COLOUR_CLASS, std::vector<ColourSegment> >& ClassifiedImage::getHorizontalTransitionsMap(){
        	return matched_horizontal_segments;
        }
        const std::map<COLOUR_CLASS, std::vector<ColourSegment> >& ClassifiedImage::getVerticalTransitionsMap(){
        	return matched_vertical_segments;
        }*/

    }  // vision
}  // messages
