/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LUTClassifier.h"
#include "QuexClassifier.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        void LUTClassifier::findGoalBases(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {
            
            // Get some local references to class variables to make text shorter
            auto& horizon = classifiedImage.horizon;
            
        	std::vector<int> points;

        	// Loop through all of our goal segments
        	auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    // Push back our midpoints x position
                    points.push_back(it->second.midpoint[0]);
                }
            }
            
            // Sort our points
            std::sort(points.begin(), points.end());
            
            // If we have some points
            if(!points.empty()) {
                
                // Loop through our points removing close points
                for(auto it = points.begin(); it < points.end() - 1; ++it) {
                    
                    // If our next point is not the minimum distance away, remove it
                    if(*(it + 1) - *it < GOAL_FINDER_MINIMUM_VERTICAL_SPACING) {
                        points.erase(it + 1);
                    }
                    // Otherwise, cast a line from the horizon down
                    else {
                        
                        int x = *it;
                        
                        // Find our point to classify from (slightly above the horizon)
                        int top = std::max(int(x * horizon[0] + horizon[1]), 0);
                        top = std::min(top, int(image.height() - 1));
                        
                        
                        // Classify our segments
                        auto segments = quex->classify(image, lut, { x, top }, { x, int(image.height() - 1) }, 1);
                        insertSegments(classifiedImage, segments, true);
                    }
                }
            }

        }

    }  // vision
}  // modules