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

#ifndef MESSAGES_INPUT_CAMERA_PARAMETERS_H
#define MESSAGES_INPUT_CAMERA_PARAMETERS_H

#include <armadillo>

namespace messages {
    namespace input{

        struct CameraParameters{
           arma::uvec2 imageSizePixels;
           arma::vec2 FOV;     //Anglular Field of view
           arma::vec2 screenToAngularFactor;    //(x,y screen *-> thetax = x*screenAngularFactor[0], thetay = y*screenAngularFactor[1])
           double effectiveScreenDistancePixels;    //Distance to the virtual screen in pixels

           double distortionFactor; //see RADIAL_CORRECTION_COEFFICIENT in VisionKinematics.h (may not be used yet)
        };

    }
}

#endif // MESSAGES_INPUT_GLOBAL_CONFIG_CAMERA_PARAMETERS_H
