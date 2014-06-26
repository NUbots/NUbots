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

        void LUTClassifier::enhanceGoals(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
                Here we improve the classification of goals.
                We do this by taking our course classification of the whole image
                and generating new segments where yellow was detected.
                We first generate segments above and below that are 2x the width of the segment
             */


            // TODO improve this, it does not work effectilly
            // It does not find the highest and lowest points of the goals well
            // Also the ball finder lines screw up the goal detection

            for (uint i = 0; i < GOAL_FINDER_DETECTOR_LEVELS.size(); ++i) {

                std::vector<ClassifiedImage<ObjectClass>::Segment> newSegments;
                auto goalSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);

                for(auto it = goalSegments.first; it != goalSegments.second; ++it) {

                    auto& elem = it->second;
                    arma::vec2 midpoint = arma::conv_to<arma::vec>::from(elem.midpoint);

                    arma::vec upperBegin = midpoint + arma::vec({ -double(elem.length) * GOAL_FINDER_DETECTOR_LEVELS[i],  double(GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                    arma::vec upperEnd   = midpoint + arma::vec({  double(elem.length) * GOAL_FINDER_DETECTOR_LEVELS[i],  double(GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                    arma::vec lowerBegin = midpoint + arma::vec({ -double(elem.length) * GOAL_FINDER_DETECTOR_LEVELS[i], -double(GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                    arma::vec lowerEnd   = midpoint + arma::vec({  double(elem.length) * GOAL_FINDER_DETECTOR_LEVELS[i], -double(GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });

                    upperBegin[0] = std::max(upperBegin[0], double(0));
                    upperBegin[0] = std::min(upperBegin[0], double(image.width() - 1));

                    upperEnd[0] = std::max(upperEnd[0], double(0));
                    upperEnd[0] = std::min(upperEnd[0], double(image.width() - 1));

                    lowerBegin[0] = std::max(lowerBegin[0], double(0));
                    lowerBegin[0] = std::min(lowerBegin[0], double(image.width() - 1));

                    lowerEnd[0] = std::max(lowerEnd[0], double(0));
                    lowerEnd[0] = std::min(lowerEnd[0], double(image.width() - 1));

                    // If the upper segment is valid
                    if(upperBegin[0] != upperEnd[0]
                      && (upperBegin[1] < image.height() && upperBegin[1] >= 0)
                      && (upperEnd[1] < image.height() && upperEnd[1] >= 0)) {

                        auto segments = quex->classify(image, lut, arma::conv_to<arma::uvec>::from(upperBegin), arma::conv_to<arma::uvec>::from(upperEnd));

                        newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                    }

                    // If the lower segment is valid and not the same as the upper segment
                    if(lowerBegin[0] != lowerEnd[0]
                      && (lowerBegin[1] < image.height() && lowerBegin[1] >= 0)
                      && (lowerEnd[1] < image.height() && lowerEnd[1] >= 0)) {

                        auto segments = quex->classify(image, lut, arma::conv_to<arma::uvec>::from(lowerBegin), arma::conv_to<arma::uvec>::from(lowerEnd));

                        newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                    }
                }

                insertSegments(classifiedImage, newSegments, false);
            }

        }

    }  // vision
}  // modules