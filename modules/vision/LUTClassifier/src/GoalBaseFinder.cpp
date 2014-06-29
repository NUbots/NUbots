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

            std::vector<arma::ivec2> points;

            // Loop through all of our goal segments
            auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    // Push back our midpoint
                    points.push_back(it->second.midpoint);
                }
            }

            // Sort our points
            std::sort(points.begin(), points.end(), [] (const arma::ivec2& a, const arma::ivec2& b) {
                return a[0] < b[0];
            });

            // If we have some points draw lines from them
            if(!points.empty()) {

                arma::running_stat_vec<arma::vec2> stats;
                for(auto it = points.begin(); it < points.end(); ++it) {

                    auto p1 = it;
                    auto p2 = it + 1;

                    // Add our point to the stats
                    stats(arma::vec2({ double(p2->at(0)), double(p2->at(1)) }));

                    // If the next point is too far away to be considered in this cluster
                    if(p2 == points.end() || p2->at(0) - p1->at(0) > GOAL_MAXIMUM_VERTICAL_CLUSTER_SPACING) {

                        // Get our relevant values
                        int top = lround(stats.min()[1]);
                        int base = lround(stats.max()[1]);
                        int x = lround(stats.mean()[0]);
                        arma::vec sd = stats.stddev();
                        int jump = sd.is_empty() ? 1 : std::max(1, int(lround(sd[0] * 1)));

                        arma::ivec2 start = { x, top - GOAL_VERTICAL_CLUSTER_UPPER_BUFFER };
                        arma::ivec2 end   = { x, base + GOAL_VERTICAL_CLUSTER_LOWER_BUFFER };

                        // Adjust our Y to stay on the screen
                        start[1] = std::max(start[1], 0);
                        end[1]   = std::min(end[1], int(image.height() - 1));

                        // Classify our point based on these
                        auto segments = quex->classify(image, lut, start, end, 1);
                        insertSegments(classifiedImage, segments, true);

                        // Shift our line right
                        start = { x + jump, start[1] };
                        end   = { x + jump, end[1] };

                        segments = quex->classify(image, lut, start, end, 1);
                        insertSegments(classifiedImage, segments, true);

                        // Shift our line left
                        start = { x - jump, start[1] };
                        end   = { x - jump, end[1] };

                        segments = quex->classify(image, lut, start, end, 1);
                        insertSegments(classifiedImage, segments, true);

                        stats.reset();
                    }
                }
            }
        }

    }  // vision
}  // modules