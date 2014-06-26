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

        void LUTClassifier::findGoals(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
               Here we cast classification lines to attempt to locate the general area of the goals.
               We cast lines only above the visual horizon (with some buffer) so that we do not over.
               classify the mostly empty green below.
             */

            auto& horizonPoints = classifiedImage.visualHorizonPoints;
            auto& maxPoint = classifiedImage.maxVisualHorizonPoint;

            auto hLeft = horizonPoints.begin();
            auto hRight = horizonPoints.end() - 1;

            // Cast lines upward to find the goals
            for(int y = classifiedImage.maxVisualHorizonPoint->at(1); y >= 0; y -= GOAL_FINDER_LINE_SPACING) {

                // If our left hand side is in range, or we are over the top
                if(hLeft->at(1) >= uint(y)) {

                    arma::uvec2 start = { uint(0), uint(y) };
                    arma::uvec2 end = { image.width() - 1, uint(y) };

                    while(hLeft < maxPoint) {

                        auto& eq = classifiedImage.visualHorizon[std::distance(horizonPoints.begin(), hLeft)];

                        int y1 = hLeft->at(1);
                        int y2 = (hLeft + 1)->at(1);

                        if(y <= y1 && y >= y2 && eq[1] != 0) {

                            // Solve the equation for x
                            end[0] = std::round((double(y) - eq[2]) / eq[1]);
                            break;
                        }
                        // Try our previous point
                        else {
                            ++hLeft;
                        }
                    }

                    // Insert our segments
                    auto segments = quex->classify(image, lut, start, end, GOAL_FINDER_SUBSAMPLING);
                    insertSegments(classifiedImage, segments, false);
                }

                // If our right hand side is in range and has not gone out of scope
                if(hRight->at(1) >= uint(y) && hRight > maxPoint) {

                    arma::uvec2 start = { uint(0), uint(y) };
                    arma::uvec2 end = { image.width() - 1, uint(y) };

                    while(hRight > maxPoint) {

                        auto& eq = classifiedImage.visualHorizon[std::distance(horizonPoints.begin(), hRight) - 1];

                        int y1 = (hRight - 1)->at(1);
                        int y2 = hRight->at(1);

                        if(y >= y1 && y <= y2 && eq[1] != 0) {

                            // Solve the equation for x
                            start[0] = std::round((double(y) - eq[2]) / eq[1]);
                            break;
                        }
                        // Try our previous point
                        else {
                            --hRight;
                        }
                    }

                    // Insert our segments
                    auto segments = quex->classify(image, lut, start, end, GOAL_FINDER_SUBSAMPLING);
                    insertSegments(classifiedImage, segments, false);

                }
            }

        }

    }  // vision
}  // modules