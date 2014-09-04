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

#include "utility/math/vision.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::imageToScreen;

        void LUTClassifier::enhanceBall(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

        	/*
                This section improves the classification of the ball.
                We first find all of the orange transitions that are below the visual horizon.
                We then take the norm of these points to attempt to find a very rough "centre" for the ball.
                Using the expected size of the ball at this position on the screen, we then crosshatch 2x the
                size needed to ensure that the ball is totally covered.
             */

            std::vector<arma::ivec2> points;
            auto& sensors = *classifiedImage.sensors;

            // Loop through all of our goal segments
            auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::BALL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                auto& pt = it->second;

                // We throw out points if they are:
                // Have both edges above the green horizon
                // Do not have a transition on either side (are on an edge)
                if(classifiedImage.visualHorizonAtPoint(pt.start[0]) <= pt.start[1] || classifiedImage.visualHorizonAtPoint(pt.end[0]) <= pt.end[1]) {

                    // Push back our midpoint offset to be in the middle of the subsample
                    points.push_back(it->second.midpoint - arma::ivec2{ int(it->second.subsample) / 2, 0 });
                }
            }


            // Sort our points
            std::sort(points.begin(), points.end(), [] (const arma::ivec2& a, const arma::ivec2& b) {
                return a[0] < b[0];
            });

            // If we have some then do our ball enhancer
            if(!points.empty()) {

                arma::running_stat_vec<arma::vec2> stats;
                for(auto it = points.begin(); it != points.end(); ++it) {

                    auto p1 = it;
                    auto p2 = it + 1;

                    // Add our point to the statistics
                    stats(arma::vec2({ double(p1->at(0)), double(p1->at(1)) }));

                    // If the next point is too far away to be considered in this cluster
                    if(p2 == points.end() || p2->at(0) - p1->at(0) > BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING) {

                        // Get the centre point to use and translate it into the kinematics form
                        arma::vec2 centre = stats.mean();
                        arma::vec2 kinematicsCentre = imageToScreen(stats.mean(), classifiedImage.dimensions);

                        // Shift the camera by BALL_RADIUS in order to move it to the correct position
                        auto cameraMatrix = sensors.orientationCamToGround;
                        cameraMatrix(2, 3) -= BALL_RADIUS;

                        // Get our two points
                        auto groundPoint = getGroundPointFromScreen(kinematicsCentre, cameraMatrix, FOCAL_LENGTH_PIXELS);
                        arma::vec4 edgePoint = arma::ones(4);
                        edgePoint.rows(0, 2) = groundPoint + (BALL_RADIUS * cameraMatrix.submat(0, 2, 2, 2));

                        auto screenEdge = projectWorldPointToScreen(edgePoint, cameraMatrix, FOCAL_LENGTH_PIXELS);

                        double radius = arma::norm(screenEdge - kinematicsCentre);

                        auto getX = [] (double r, double x0, double y0, double y) {

                            double a = y - y0;
                            double b = sqrt(r * r - a * a);

                            return std::make_pair(int(lround(x0 - b)), int(lround(x0 + b)));
                        };

                        int jumpSize = std::max(1, int(lround((2 * radius) / double(BALL_MINIMUM_INTERSECTIONS_FINE + 2))));

                        int xStart = std::max(int(lround(centre[0] - radius * BALL_SEARCH_CIRCLE_SCALE + jumpSize)), 0);
                        int xEnd   = std::min(int(lround(centre[0] + radius * BALL_SEARCH_CIRCLE_SCALE - jumpSize)), int(image.width() - 1));
                        int yStart = std::max(int(lround(centre[1] - radius * BALL_SEARCH_CIRCLE_SCALE + jumpSize)), 0);
                        int yEnd   = std::min(int(lround(centre[1] + radius * BALL_SEARCH_CIRCLE_SCALE - jumpSize)), int(image.height() - 1));

                        for(int x = xStart; x <= xEnd; x += jumpSize) {

                            auto ends = getX(BALL_SEARCH_CIRCLE_SCALE * radius, centre[1], centre[0], x);

                            arma::ivec2 start = { x, ends.first };
                            arma::ivec2 end = { x, ends.second };

                            start[1] = std::max(start[1], 0);
                            end[1] = std::min(end[1], int(image.height() - 1));

                            auto segments = quex->classify(image, lut, start, end);
                            insertSegments(classifiedImage, segments, true);
                        }

                        for(int y = yStart; y <= yEnd; y += jumpSize) {

                            auto ends = getX(BALL_SEARCH_CIRCLE_SCALE * radius, centre[0], centre[1], y);

                            arma::ivec2 start = { ends.first, y };
                            arma::ivec2 end = { ends.second, y };

                            start[0] = std::max(start[0], 0);
                            end[0] = std::min(end[0], int(image.width() - 1));

                            auto segments = quex->classify(image, lut, start, end);
                            insertSegments(classifiedImage, segments, false);
                        }

                        stats.reset();
                    }
                }
            }
        }

    }  // vision
}  // modules