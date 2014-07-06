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

#include "utility/math/geometry/Line.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::geometry::Line;

        void LUTClassifier::findGoals(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
               Here we cast classification lines to attempt to locate the general area of the goals.
               We cast lines only above the visual horizon (with some buffer) so that we do not over.
               classify the mostly empty green below.
             */

            auto& visualHorizon = classifiedImage.visualHorizon;
            auto& maxPoint = classifiedImage.maxVisualHorizon;
            auto& minPoint = classifiedImage.minVisualHorizon;

            auto hLeft = visualHorizon.begin();
            auto hRight = visualHorizon.end() - 1;

            // Cast lines upward to find the goals
            for(int y = maxPoint->at(1); y >= 0; y -= GOAL_LINE_SPACING) {

                // If our left hand side is in range, or we are over the top
                if(hLeft->at(1) >= y) {

                    arma::ivec2 start = { 0, y };
                    arma::ivec2 end = { int(image.width() - 1), y };

                    // Clip the point to be outside the visual horizon
                    while(hLeft < minPoint) {

                        auto p1 = hLeft;
                        auto p2 = hLeft + 1;

                        if(y <= p1->at(1) && y >= p2->at(1)) {

                            // Make a line from the two points and find our x
                            Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                            if(l.isHorizontal()) {
                                end[0] = p1->at(0);
                            }
                            else {
                                end[0] = round(l.x(y));
                            }

                            break;
                        }
                        // Try our previous point
                        else {
                            ++hLeft;
                        }
                    }

                    // Insert our segments
                    auto segments = quex->classify(image, lut, start, end, GOAL_SUBSAMPLING);
                    insertSegments(classifiedImage, segments, false);
                }

                // If our right hand side is in range and has not gone out of scope
                if(hRight->at(1) >= y && hRight > minPoint) {

                    arma::ivec2 start = { 0, y };
                    arma::ivec2 end = { int(image.width() - 1), y };

                    // Clip the point to be outside the visual horizon
                    while(hRight > minPoint) {

                        auto p1 = hRight - 1;
                        auto p2 = hRight;

                        if(y >= p1->at(1) && y <= p2->at(1)) {

                            // Make a line from the two points and find our x
                            Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                            if(l.isHorizontal()) {
                                start[0] = p2->at(0);
                            }
                            else {
                                start[0] = round(l.x(y));
                            }

                            break;
                        }
                        // Try our previous point
                        else {
                            --hRight;
                        }
                    }

                    // Insert our segments
                    auto segments = quex->classify(image, lut, start, end, GOAL_SUBSAMPLING);
                    insertSegments(classifiedImage, segments, false);

                }
            }

        }

    }  // vision
}  // modules