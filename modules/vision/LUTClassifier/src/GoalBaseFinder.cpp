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
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        void LUTClassifier::findGoalBases(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

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
                for(auto it = points.begin(); it != points.end(); ++it) {

                    auto p1 = it;
                    auto p2 = it + 1;

                    // Add our point to the stats
                    stats(arma::vec2({ double(p1->at(0)), double(p1->at(1)) }));

                    // If the next point is too far away to be considered in this cluster
                    if(p2 == points.end() || p2->at(0) - p1->at(0) > GOAL_MAXIMUM_VERTICAL_CLUSTER_SPACING) {

                        arma::imat search(2, 6);

                        // Set our x to the mean x point
                        search.row(0).fill(int(lround(stats.mean()[0])));

                        // Set our start points y to the top line found - some buffer
                        search(arma::umat({ 1 }), arma::umat({ 0, 2, 4 })).fill(lround(stats.min()[1]) - GOAL_VERTICAL_CLUSTER_UPPER_BUFFER);

                        // Set our base points to the bottom line found + some buffer
                        search(arma::umat({ 1 }), arma::umat({ 1, 3, 5 })).fill(lround(stats.max()[1]) + GOAL_VERTICAL_CLUSTER_UPPER_BUFFER);

                        auto sd = stats.stddev();
                        int jump = sd.is_empty() ? 1 : std::max(1, int(lround(sd[0] * GOAL_VERTICAL_SD_JUMP)));

                        // Offset our x by some constant of standard deviations
                        search(arma::umat({ 0 }), arma::umat({ 0, 1 })) -= jump;
                        search(arma::umat({ 0 }), arma::umat({ 4, 5 })) += jump;

                        // Our top must be at least 0
                        search(arma::uvec({ 1 }), arma::find(search.row(1) < 0)).fill(0);

                        // Our base must be at most image height
                        search(arma::uvec({ 1 }), arma::find(search.row(1) > int(image.height() - 1))).fill(int(image.height() - 1));

                        // Only draw lines if they are in range
                        if(search(0, 0) >= 0 && search(0, 0) < int(image.width())) {

                            auto segments = quex->classify(image, lut, search.col(0), search.col(1));
                            insertSegments(classifiedImage, segments, true);
                        }
                        if(search(0, 2) >= 0 && search(0, 2) < int(image.width())) {

                            auto segments = quex->classify(image, lut, search.col(2), search.col(3));
                            insertSegments(classifiedImage, segments, true);
                        }
                        if(search(0, 4) >= 0 && search(0, 4) < int(image.width())) {

                            auto segments = quex->classify(image, lut, search.col(4), search.col(5));
                            insertSegments(classifiedImage, segments, true);
                        }

                        stats.reset();
                    }
                }
            }
        }

    }  // vision
}  // modules