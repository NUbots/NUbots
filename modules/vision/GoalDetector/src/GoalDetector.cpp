/*
 * This file is part of NUbots Codebase.
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

#include "GoalDetector.h"

#include "messages/vision/ClassifiedImage.h"

namespace modules {
namespace vision {

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

        	// Get our left and right points
        	std::vector<arma::uvec2> leftPoints;
        	std::vector<arma::uvec2> rightPoints;

            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            auto vSegments = image.verticalSegments.equal_range(ObjectClass::GOAL);

        	for(auto it = hSegments.first; it != hSegments.second; ++it) {
                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    leftPoints.push_back(it->second.start);
                    rightPoints.push_back(it->second.end);
                }
        	}

            // Fit lines to each set of points using ransac
            // auto lines = ransac<LineModel>(leftPoints);


            //                                                                                 CONSENSUS_THRESHOLD,
            //                                                                                 MINIMUM_POINTS,
            //                                                                                 MAX_ITERATIONS_PER_FITTING,
            //                                                                                 MAX_FITTING_ATTEMPTS,
            //                                                                                 SELECTION_METHOD

            // Build quads using the sets of lines

            // Throwout/merge quads

            // Assign a "Left" and "Right" if we can

            // The quads are the goal posts?




        	// Use ransac to find left goalposts (left segment ends)

        	// Use ransac to find right goalposts (right segment ends)
        });

    }

}
}

