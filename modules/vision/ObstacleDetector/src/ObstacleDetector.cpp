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

#include "ObstacleDetector.h"

#include <armadillo>
#include "messages/vision/ClassifiedImage.h"

namespace modules {
namespace vision {

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;

    ObstacleDetector::ObstacleDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

            std::vector<arma::ivec2> points;

            // Get all the segments that are relevant to finding an obstacle
            for(int i = 0; i < 6; ++i) {

                auto segments = i == 0 ? image.horizontalSegments.equal_range(ObjectClass::UNKNOWN)
                              : i == 1 ? image.verticalSegments.equal_range(ObjectClass::UNKNOWN)
                              : i == 2 ? image.horizontalSegments.equal_range(ObjectClass::CYAN_TEAM)
                              : i == 3 ? image.verticalSegments.equal_range(ObjectClass::CYAN_TEAM)
                              : i == 4 ? image.horizontalSegments.equal_range(ObjectClass::MAGENTA_TEAM)
                              : image.verticalSegments.equal_range(ObjectClass::MAGENTA_TEAM);

                for(auto it = segments.first; it != segments.second; ++it) {

                    auto& start = it->second.start;
                    auto& end = it->second.end;

                    // Check if we have a subsequent segment
                    if(it->second.previous && image.visualHorizonAtPoint(start[0]) < start[1]) {
                        points.push_back(start);
                    }

                    // Check if we have a subsequent segment
                    if(it->second.next && image.visualHorizonAtPoint(end[0]) < end[1]) {
                        points.push_back(end);
                    }
                }
            }


            // ALGORITHM

            // we seperate into 3 sections, starts ends and other
            // All horizontal segments that are 100% below the GH go to start/end
            // end of vertical segments go to other

            // We sort all of them by x

            // We have a counter for "depth"
            // We increment it for every start we se and decrement for every end
            // We keep a record of the point with the largest Y (start end or other)
            // If any of the segments are either cyan or magenta, we flag it as a team
            // If we see both cyan and magenta then it goes back to being stuck at unknown
            // When we hit the transition to 0 depth we make the previous points into an obstacle
            // Once you have all the points, do a lower convex hull on them to try to determine it's size
            // Do d2p at these points or something to try to determine the physical size of the obstacle on the field



            // Sort our points
            std::sort(points.begin(), points.end(), [] (const arma::ivec2& a, const arma::ivec2& b) {
                return a[0] < b[0];
            });

            // Now build a locally convex hull or something

            // Or possibly do a threshold check (for every start there is an end, if all segments end we have reached the end of the obstacle)
        });

    }

}
}

