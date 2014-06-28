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

        void LUTClassifier::enhanceBall(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

        	/*
                This section improves the classification of the ball.
                We first find all of the orange transitions that are below the visual horizon.
                We then take the norm of these points to attempt to find a very rough "centre" for the ball.
                Using the expected size of the ball at this position on the screen, we then crosshatch 2x the
                size needed to ensure that the ball is totally covered.
             */

            std::vector<arma::ivec2> points;

            auto ballSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::BALL);

            for(auto it = ballSegments.first; it != ballSegments.second; ++it) {

                auto& pt = it->second;

                // We throw out points if they are:
                // Have both edges above the green horizon
                // Do not have a transition on either side (are on an edge)
                if(classifiedImage.visualHorizonAtPoint(pt.start[0]) >= pt.start[1] || classifiedImage.visualHorizonAtPoint(pt.end[0]) >= pt.end[1]) {

                    // Push back our midpoints x position
                    points.push_back(pt.midpoint);
                }
            }

            //arma::ivec2 centre = arma::mean(points);

            // get a running stat vec of the ball points below the green horizon

            // calculate the mean and SD of the points

            // Iterate through the points again and remove any that are outside the SD

            // Recalculate the mean

            // Get the estimated ball width for this distance

            // Create a field that is CONFIG * larger then this

            // Within a circle of that config diameter, draw n lines, where n ensures that at least SOMECONFIG lines go through the ball

        }

    }  // vision
}  // modules