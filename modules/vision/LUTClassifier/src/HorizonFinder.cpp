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

        void LUTClassifier::findHorizon(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

                // Element 0 is gradient, element 1 is intercept (confirmed by Jake's Implementation)
                // Coordinate system: 0,0 is the centre of the screen. pos[0] is along the y axis of the
                // camera transform, pos[1] is along the z axis (x points out of the camera)
                classifiedImage.horizon = sensors.orientationHorizon;

                // Move the intercept to be at 0,0
                classifiedImage.horizon[1] += (image.height() * 0.5) + classifiedImage.horizon[0] * -(image.width() * 0.5);
        }

    }  // vision
}  // modules