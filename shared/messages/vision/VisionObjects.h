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

#ifndef MESSAGES_VISION_VISIONOBJECTS_H
#define MESSAGES_VISION_VISIONOBJECTS_H

#include <armadillo>

#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/geometry/Polygon.h"
#include "messages/input/Sensors.h"

namespace messages {
    namespace vision {

        struct VisionObject {

            /**
             * Measurements are taken from the ground plane below the robot
             * It is measured in spherical coodinates and the error is a covariance
             * matrix representing the uncertinaty.
             */
            struct Measurement {
                arma::vec3 position;
                arma::mat33 error;
            };

            // Time the image was taken
            NUClear::clock::time_point timestamp;

            // Position of object relative to ground to centre of object in spherical coordinates
            std::vector<Measurement> measurements;

            // The angular position and size from the perspective of the camera
            arma::vec2 screenAngular;
            arma::vec2 angularSize;
            
            std::shared_ptr<const messages::input::Sensors> sensors;
        };

        struct Ball : public VisionObject {
            utility::math::geometry::Circle circle;
        };

        struct Goal : public VisionObject {
            enum class Side {
                LEFT,
                RIGHT,
                UNKNOWN
            } side;

            utility::math::geometry::Quad quad;
        };

        struct Obstacle : public VisionObject {

            enum class Team {
                NONE,
                MAGENTA,
                CYAN
            } team;

            utility::math::geometry::Polygon polygon;
        };

    }
}

#endif // MESSAGES_VISION_VISIONOBJECTS_H
