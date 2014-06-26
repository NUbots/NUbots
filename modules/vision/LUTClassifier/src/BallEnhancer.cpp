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
            arma::running_stat_vec<arma::uvec> centre;
            auto ballSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::BALL);
            for(auto it = ballSegments.first; it != ballSegments.second; ++it) {

                auto& elem = it->second;

                centre(elem.midpoint);

                // Get the expected size of the ball at the
            }

            // Throw out any outliers (bigger then x sd)
            centre.stddev();

            // Find the size of a ball at the position
            auto ballSize = centre.mean();

            // Distance to point to centre ( n below horizon = h/alphax )

            // Get the width of the imaginary ball

            // Find the angular width of the ball at this distance

            // Multiply tan of that angle by the angle->pixels constant (alpha?)

        }

    }  // vision
}  // modules