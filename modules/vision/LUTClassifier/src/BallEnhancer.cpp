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
#include "utility/nubugger/NUhelpers.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::imageToScreen;
        using utility::nubugger::drawVisionLines;

        std::pair<float, arma::ivec2> fieldEdgeDirection(const arma::ivec2& base, const Image& image, const arma::fvec3 greenCentroid) {


            // Get our centre and vector to the green centroid
            // auto centrePixel = image(base[0], base[1]);
            // arma::fvec3 c({ float(centrePixel.y), float(centrePixel.cb), float(centrePixel.cr) });
            // arma::fvec3 g = c - greenCentroid;


            // Get our relevant pixels
            std::array<Image::Pixel, 24> pixels {
                image(base[0] - 2, base[1] - 2),
                image(base[0] - 2, base[1] - 1),
                image(base[0] - 2, base[1] + 0),
                image(base[0] - 2, base[1] + 1),
                image(base[0] - 2, base[1] + 2),

                image(base[0] - 1, base[1] - 2),
                image(base[0] - 1, base[1] - 1),
                image(base[0] - 1, base[1] + 0),
                image(base[0] - 1, base[1] + 1),
                image(base[0] - 1, base[1] + 2),

                image(base[0] + 0, base[1] - 2),
                image(base[0] + 0, base[1] - 1),
                image(base[0] + 0, base[1] + 1),
                image(base[0] + 0, base[1] + 2),

                image(base[0] + 1, base[1] - 2),
                image(base[0] + 1, base[1] - 1),
                image(base[0] + 1, base[1] + 0),
                image(base[0] + 1, base[1] + 1),
                image(base[0] + 1, base[1] + 2),

                image(base[0] + 2, base[1] - 2),
                image(base[0] + 2, base[1] - 1),
                image(base[0] + 2, base[1] + 0),
                image(base[0] + 2, base[1] + 1),
                image(base[0] + 2, base[1] + 2)
            };

            // Find out how green each pixel is!
            std::array<float, 24> greenness;
            for(int i = 0; i < int(greenness.size()); ++i) {
                greenness[i] = arma::norm(greenCentroid - arma::fvec3({ float(pixels[i].y), float(pixels[i].cb), float(pixels[i].cr) }));
            }

            constexpr float M_1_SQRT5 = 0.4472135955;
            constexpr float M_2_SQRT5 = 0.894427191;
            constexpr float M_SQRT2_2 = M_SQRT2 * 0.5;

            // 0 5 10 14 19
            // 1 6 11 15 20
            // 2 7    16 21
            // 3 8 12 17 22
            // 4 9 13 18 23

            arma::fvec2 greenDirection = (arma::fvec2({-M_SQRT2_2, -M_SQRT2_2}) * greenness[0])
                                       + (arma::fvec2({-M_2_SQRT5, -M_1_SQRT5}) * greenness[1])
                                       + (arma::fvec2({-1        , +0})         * greenness[2])
                                       + (arma::fvec2({-M_2_SQRT5, +M_1_SQRT5}) * greenness[3])
                                       + (arma::fvec2({-M_SQRT2_2, +M_SQRT2_2}) * greenness[4])
                                       + (arma::fvec2({-M_1_SQRT5, -M_2_SQRT5}) * greenness[5])
                                       + (arma::fvec2({-M_SQRT2_2, -M_SQRT2_2}) * greenness[6])
                                       + (arma::fvec2({-1        , +0})         * greenness[7])
                                       + (arma::fvec2({-M_SQRT2_2, +M_SQRT2_2}) * greenness[8])
                                       + (arma::fvec2({-M_1_SQRT5, +M_2_SQRT5}) * greenness[9])
                                       + (arma::fvec2({+0        , -1})         * greenness[10])
                                       + (arma::fvec2({+0        , -1})         * greenness[11])
                                       + (arma::fvec2({+0        , +1})         * greenness[12])
                                       + (arma::fvec2({+0        , +1})         * greenness[13])
                                       + (arma::fvec2({+M_1_SQRT5, -M_2_SQRT5}) * greenness[14])
                                       + (arma::fvec2({+M_SQRT2_2, -M_SQRT2_2}) * greenness[15])
                                       + (arma::fvec2({+1        , +0})         * greenness[16])
                                       + (arma::fvec2({+M_SQRT2_2, +M_SQRT2_2}) * greenness[17])
                                       + (arma::fvec2({+M_1_SQRT5, +M_2_SQRT5}) * greenness[14])
                                       + (arma::fvec2({+M_SQRT2_2, -M_SQRT2_2}) * greenness[19])
                                       + (arma::fvec2({+M_2_SQRT5, -M_1_SQRT5}) * greenness[20])
                                       + (arma::fvec2({+1        , +0})         * greenness[21])
                                       + (arma::fvec2({+M_2_SQRT5, +M_1_SQRT5}) * greenness[22])
                                       + (arma::fvec2({+M_SQRT2_2, +M_SQRT2_2}) * greenness[23]);

            // 1/sqrt(5);
            // 2/sqrt(5);



            // Work out our greenest direction!
            // arma::fvec2 greenDirection = (arma::fvec2({-M_SQRT2_2, -M_SQRT2_2}) * greenness[6])
            //                            + (arma::fvec2({-1        , +0})         * greenness[7])
            //                            + (arma::fvec2({-M_SQRT2_2, +M_SQRT2_2}) * greenness[8])
            //                            + (arma::fvec2({+0        , -1})         * greenness[11])
            //                            + (arma::fvec2({+0        , +1})         * greenness[12])
            //                            + (arma::fvec2({+M_SQRT2_2, -M_SQRT2_2}) * greenness[15])
            //                            + (arma::fvec2({+1        , +0})         * greenness[16])
            //                            + (arma::fvec2({+M_SQRT2_2, +M_SQRT2_2}) * greenness[17]);

            // How strong is our greenness movement?
            double strength = arma::norm(greenDirection);

            // Normalise our direction
            greenDirection /= strength;

            strength /= greenness.size();
            arma::ivec2 greenNormal({ -int(std::round(greenDirection[1])), int(std::round(greenDirection[0])) });

            return std::make_pair(strength, greenNormal);
        }

        void LUTClassifier::enhanceBall(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            // Loop through all of our possible ball segments
            std::vector<arma::ivec2> points;
            auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                auto& pt = it->second;

                // We throw out points if they:
                // Have both edges above the green horizon
                // Are too small
                if((classifiedImage.visualHorizonAtPoint(pt.start[0]) <= pt.start[1]
                || classifiedImage.visualHorizonAtPoint(pt.end[0]) <= pt.end[1])
                && pt.length > 20) {
                    points.push_back(pt.midpoint);
                }
            }

            std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> debug;
            std::vector<arma::ivec2> edges;

            // For each of these points move upward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                for(int y = point[1]; y > classifiedImage.horizon.y(point[0]); --y) {

                    char c = lut(image(point[0], y));

                    if(c == 'g') {
                        edges.push_back(arma::ivec2({ point[0], y - 1 }));
                        debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1})));
                        break;
                    }
                }
            }

            // For each of these points move leftward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                for(int x = point[0]; x > 0; --x) {

                    char c = lut(image(x, point[1]));

                    if(c == 'g') {
                        edges.push_back(arma::ivec2({ x + 1, point[1] }));
                        debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1})));
                        break;
                    }
                }
            }

            // For each of these points move rightward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                for(int x = point[0]; x < image.width - 1; ++x) {

                    char c = lut(image(x, point[1]));

                    if(c == 'g') {
                        edges.push_back(arma::ivec2({ x - 1, point[1] }));
                        debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1})));
                        break;
                    }
                }
            }

            // While we still have edges
            auto setComparator = [] (const arma::ivec2& a, const arma::ivec2& b) {
                return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0];
            };
            std::set<arma::ivec2, decltype(setComparator)> pSet(setComparator);

            for(auto& edge : edges) {

                // Go clockwise
                arma::ivec2 point = edge;
                for(int i = 0; i < 100; ++i) {

                    std::tuple<arma::ivec2, arma::ivec2, arma::vec4> d;
                    std::get<0>(d) = point;

                    float strength;
                    arma::ivec2 direction;
                    std::tie(strength, direction) = fieldEdgeDirection(point, image, greenCentroid);

                    // If our strength get's too low then stop
                    if(strength < 2) {
                        break;
                    }

                    point += direction;


                    pSet.insert(point);

                    std::get<1>(d)  = point;

                    float r = (strength / 30);
                    float b = 1 - (strength / 30);
                    std::get<2>(d)  = arma::vec4({r,0,b,1});
                    debug.push_back(d);
                }

                // Go Anticlockwise
                point = edge;
                for(int i = 0; i < 100; ++i) {

                    std::tuple<arma::ivec2, arma::ivec2, arma::vec4> d;
                    std::get<0>(d) = point;

                    float strength;
                    arma::ivec2 direction;
                    std::tie(strength, direction) = fieldEdgeDirection(point, image, greenCentroid);

                    // If our strength get's too low then stop
                    if(strength < 2) {
                        break;
                    }

                    point -= direction;

                    pSet.insert(point);

                    std::get<1>(d)  = point;

                    float r = (strength / 30);
                    float b = 1 - (strength / 30);
                    std::get<2>(d)  = arma::vec4({r,0,b,1});
                    debug.push_back(d);
                }
            }

            // Put our set into the object
            classifiedImage.ballPoints.insert(classifiedImage.ballPoints.begin(), pSet.begin(), pSet.end());

            emit(drawVisionLines(debug));

            // for(auto& edge : edges) {

            //     fieldEdgeDirection(edge);
            // }

            // Now do we look for a cluster?

            // For each of the pixel starting points move up until we hit green (the top of the ball is the most likely to be well defined)

        	/*
                This section improves the classification of the ball.
                We first find all of the orange transitions that are below the visual horizon.
                We then take the norm of these points to attempt to find a very rough "centre" for the ball.
                Using the expected size of the ball at this position on the screen, we then crosshatch 2x the
                size needed to ensure that the ball is totally covered.
             */


            // std::vector<arma::ivec2> points;
            // auto& sensors = *classifiedImage.sensors;

            // // Loop through all of our goal segments
            // auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::BALL);
            // for(auto it = hSegments.first; it != hSegments.second; ++it) {

            //     auto& pt = it->second;

            //     // We throw out points if they are:
            //     // Have both edges above the green horizon
            //     // Do not have a transition on either side (are on an edge)
            //     if(classifiedImage.visualHorizonAtPoint(pt.start[0]) <= pt.start[1] || classifiedImage.visualHorizonAtPoint(pt.end[0]) <= pt.end[1]) {

            //         // Push back our midpoint offset to be in the middle of the subsample
            //         points.push_back(it->second.midpoint - arma::ivec2{ int(it->second.subsample) / 2, 0 });
            //     }
            // }


            // // Sort our points
            // std::sort(points.begin(), points.end(), [] (const arma::ivec2& a, const arma::ivec2& b) {
            //     return a[0] < b[0];
            // });

            // // If we have some then do our ball enhancer
            // if(!points.empty()) {

            //     arma::running_stat_vec<arma::vec2> stats;
            //     for(auto it = points.begin(); it != points.end(); ++it) {

            //         auto p1 = it;
            //         auto p2 = it + 1;

            //         // Add our point to the statistics
            //         stats(arma::vec2({ double(p1->at(0)), double(p1->at(1)) }));

            //         // If the next point is too far away to be considered in this cluster
            //         if(p2 == points.end() || p2->at(0) - p1->at(0) > BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING) {

            //             // Get the centre point to use and translate it into the kinematics form
            //             arma::vec2 centre = stats.mean();
            //             arma::vec2 kinematicsCentre = imageToScreen(stats.mean(), classifiedImage.dimensions);

            //             // Shift the camera by BALL_RADIUS in order to move it to the correct position
            //             auto cameraMatrix = sensors.orientationCamToGround;
            //             cameraMatrix(2, 3) -= BALL_RADIUS;

            //             // Get our two points
            //             auto groundPoint = getGroundPointFromScreen(kinematicsCentre, cameraMatrix, FOCAL_LENGTH_PIXELS);
            //             arma::vec4 edgePoint = arma::ones(4);
            //             edgePoint.rows(0, 2) = groundPoint + (BALL_RADIUS * cameraMatrix.submat(0, 2, 2, 2));

            //             auto screenEdge = projectWorldPointToScreen(edgePoint, cameraMatrix, FOCAL_LENGTH_PIXELS);

            //             double radius = arma::norm(screenEdge - kinematicsCentre);

            //             // solve for the two solutions of x given the equation of a circle (x - x0)^2 + (y - y0)^2 = r^2
            //             // the circle is centered on the point (x0, y0) and the points are those which intersect the horizontal line given by y
            //             auto getX = [] (double r, double x0, double y0, double y) {

            //                 double a = y - y0;
            //                 double b = sqrt(r * r - a * a);

            //                 return std::make_pair(int(lround(x0 - b)), int(lround(x0 + b)));
            //             };

            //             int jumpSize = std::max(1, int(lround((2 * radius) / double(BALL_MINIMUM_INTERSECTIONS_FINE + 2))));

            //             int xStart = std::max(int(lround(centre[0] - radius * BALL_SEARCH_CIRCLE_SCALE + jumpSize)), 0);
            //             int xEnd   = std::min(int(lround(centre[0] + radius * BALL_SEARCH_CIRCLE_SCALE - jumpSize)), int(image.width - 1));
            //             int yStart = std::max(int(lround(centre[1] - radius * BALL_SEARCH_CIRCLE_SCALE + jumpSize)), 0);
            //             int yEnd   = std::min(int(lround(centre[1] + radius * BALL_SEARCH_CIRCLE_SCALE - jumpSize)), int(image.height - 1));

            //             for(int x = xStart; x <= xEnd; x += jumpSize) {

            //                 auto ends = getX(BALL_SEARCH_CIRCLE_SCALE * radius, centre[1], centre[0], x);

            //                 arma::ivec2 start = { x, ends.first };
            //                 arma::ivec2 end = { x, ends.second };

            //                 start[1] = std::max(start[1], 0);
            //                 end[1] = std::min(end[1], int(image.height - 1));

            //                 auto segments = quex->classify(image, lut, start, end);
            //                 insertSegments(classifiedImage, segments, true);
            //             }

            //             for(int y = yStart; y <= yEnd; y += jumpSize) {

            //                 auto ends = getX(BALL_SEARCH_CIRCLE_SCALE * radius, centre[0], centre[1], y);

            //                 arma::ivec2 start = { ends.first, y };
            //                 arma::ivec2 end = { ends.second, y };

            //                 start[0] = std::max(start[0], 0);
            //                 end[0] = std::min(end[0], int(image.width - 1));

            //                 auto segments = quex->classify(image, lut, start, end);
            //                 insertSegments(classifiedImage, segments, false);
            //             }

            //             stats.reset();
            //         }
            //     }
            // }
        }

    }  // vision
}  // modules