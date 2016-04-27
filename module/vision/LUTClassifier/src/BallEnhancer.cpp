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

namespace module {
    namespace vision {

        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ObjectClass;
        using message::vision::ClassifiedImage;

        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::imageToScreen;
        using utility::nubugger::drawVisionLines;

        std::pair<float, arma::ivec2> fieldEdgeDirection(const arma::ivec2& base, const Image& image, const arma::fvec3 greenCentroid) {

            // Get our relevant pixels
            //TODO:bounds check
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
            // NUClear::log("hSegments size = ", std::distance(hSegments.first,hSegments.second));
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                auto& pt = it->second;

                // We throw out points if they:
                // Have both edges above the green horizon
                // Are too small
                if((classifiedImage.visualHorizonAtPoint(pt.start[0]) <= pt.start[1]
                || classifiedImage.visualHorizonAtPoint(pt.end[0]) <= pt.end[1])
                && pt.length > 3) {
                    points.push_back(pt.midpoint);
                }
            }

            std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> debug;
            std::vector<arma::ivec2> edges;

            // For each of these points move upward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                int minY = int(std::max(3.0, classifiedImage.horizon.y(point[0])));
                for(int y = point[1]; y > minY; --y) {

                    char c = lut(image(point[0], y));

                    if(c == 'g') {
                        auto p = arma::ivec2({ point[0], y - 1 });
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[0].push_back(p);
                        debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1})));
                        break;
                    }
                }
            }

            // For each of these points move leftward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                for(int x = point[0]; x > 3; --x) {

                    char c = lut(image(x, point[1]));

                    if(c == 'g') {
                        auto p = arma::ivec2({ x + 1, point[1] });
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[1].push_back(p);
                        debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1})));
                        break;
                    }
                }
            }

            // For each of these points move rightward until we find a strong transition to green
            for(auto& point : points) {

                // The last pixel we looked at
                // auto lastPixel = image(point[0], point[1]);

                for(int x = point[0]; x < int(image.width) - 3; ++x) {

                    char c = lut(image(x, point[1]));

                    if(c == 'g') {
                        auto p = arma::ivec2({ x - 1, point[1] });
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[2].push_back(p);
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

                // The seed points are also edges
                pSet.insert(edge);

                // Go clockwise
                arma::ivec2 point = edge;
                for(int i = 0; i < 100; ++i) {

                    // Break if we hit the edge of the screen
                    if(point[0] < 4 || point[0] > (int(image.width) - 4) || point[1] < 4 || point[1] > (int(image.height) - 4)) {
                        break;
                    }

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

                    // Break if we hit the edge of the screen
                    if(point[0] < 4 || point[0] > (int(image.width) - 4) || point[1] < 4 || point[1] > (int(image.height) - 4)) {
                        break;
                    }

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
            // NUClear::log("Finished enhancing ball regions: number of points = ", pSet.size());
            emit(drawVisionLines(debug));
        }

    }  // vision
}  // modules
