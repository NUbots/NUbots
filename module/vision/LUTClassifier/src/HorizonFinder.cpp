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

#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ClassifiedImage;

        using ServoID = utility::input::ServoID;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;

        void LUTClassifier::findHorizon(const Image& image, const LookUpTable&, ClassifiedImage& classifiedImage) {

                auto& sensors = *classifiedImage.sensors;

                // Get our transform to world coordinates
                const Rotation3D& Rtw = Transform3D(convert<double, 4, 4>(sensors.world)).rotation();
                const Rotation3D& Rtc = Transform3D(convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH))).rotation();
                Rotation3D Rcw =  Rtc.i() * Rtw;
                classifiedImage.horizon_normal = convert<double, 3>(Rcw.z());
        }

    }  // vision
}  // modules
