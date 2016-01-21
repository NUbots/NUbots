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
#include "utility/motion/ForwardKinematics.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::input::Sensors;
        using message::vision::LookUpTable;
        using message::vision::ObjectClass;
        using message::vision::ClassifiedImage;

        void LUTClassifier::findHorizon(const Image& image, const LookUpTable&, ClassifiedImage<ObjectClass>& classifiedImage) {

                auto& sensors = *classifiedImage.sensors;

                // Coordinate system: 0,0 is the centre of the screen. pos[0] is along the y axis of the
                // camera transform, pos[1] is along the z axis (x points out of the camera)
                classifiedImage.horizon = utility::motion::kinematics::calculateHorizon(sensors.orientationCamToGround.submat(0,0,2,2).t(), FOCAL_LENGTH_PIXELS);


                // Move our axis to be at the top left of the screen
                classifiedImage.horizon.distance = -classifiedImage.horizon.distanceToPoint({ -double(image.width) * 0.5, -double(image.height) * 0.5 });
        }

    }  // vision
}  // modules