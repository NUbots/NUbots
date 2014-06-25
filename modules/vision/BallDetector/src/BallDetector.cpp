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

#include "BallDetector.h"

#include "messages/vision/ClassifiedImage.h"

#include "utility/math/ransac/ransac.h"
#include "utility/math/ransac/RansacCircleModel.h"

#include "messages/vision/VisionObjects.h"

namespace modules {
namespace vision {

    using utility::math::ransac::RansacCircleModel;
    using utility::math::ransac::findMultipleModels;
    using utility::math::ransac::RansacSelectionMethod;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Ball;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

            std::vector<arma::vec2> ballPoints;

            std::function<bool (const arma::uvec2&, const arma::vec3&)> cFunc = [] (const arma::uvec2& point, const arma::vec3& horizon) {
                // Check if the X coordinate of the point is less then the x of the horizon
                return point[0] < horizon[0];
            };

            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::BALL);
            auto vSegments = image.verticalSegments.equal_range(ObjectClass::BALL);

            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    // Get the line segment before the point greater then it
                    auto startHorizon = --std::upper_bound(image.visualHorizon.begin(), image.visualHorizon.end(), it->second.start, cFunc);
                    auto endHorizon = --std::upper_bound(image.visualHorizon.begin(), image.visualHorizon.end(), it->second.end, cFunc);

                    if((it->second.start[0] * startHorizon->at(1) + startHorizon->at(2)) < it->second.start[1]
                    && (it->second.end[0] * endHorizon->at(1) + endHorizon->at(2)) < it->second.end[1]) {
                        ballPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                        ballPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                    }
                }
            }

            for(auto it = vSegments.first; it != vSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    ballPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                }
            }

            log("BallPoints", ballPoints.size());

        });
    }

}
}
