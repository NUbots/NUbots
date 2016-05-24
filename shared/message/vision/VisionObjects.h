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

#ifndef MESSAGE_VISION_VISIONOBJECTS_H
#define MESSAGE_VISION_VISIONOBJECTS_H

#include <armadillo>

#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/geometry/Polygon.h"
#include "message/input/Sensors.h"
#include "message/vision/ClassifiedImage.h"

namespace message {
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
                //Optional vel measurement, currently used for ball only\
                //XXX: THIS IS A TURRIBLE IDEA
                arma::vec3 velocity;
                arma::mat33 velCov;
            };

            // Time the image was taken
            NUClear::clock::time_point timestamp;


            // Position of object relative to ground to centre of object in spherical coordinates
            std::vector<Measurement> measurements;

            // The angular position and size from the perspective of the camera
            arma::vec2 screenAngular;
            arma::vec2 angularSize;

            // The sensor frame that was used to detect this object
            std::shared_ptr<const message::input::Sensors> sensors;

            // The classified image that was used to detect this object
            // TODO: Why is this here?
            std::shared_ptr<const message::vision::ClassifiedImage<message::vision::ObjectClass>> classifiedImage;
        };

        struct Ball : public VisionObject {
            utility::math::geometry::Circle circle;
        };

        struct Goal : public VisionObject {
            enum class Side {
                UNKNOWN,
                LEFT,
                RIGHT,
            } side = Side::UNKNOWN;

            enum class Team {
                UNKNOWN,
                OWN,
                OPPONENT,
            } team = Team::UNKNOWN;

            utility::math::geometry::Quad quad;
        };

        struct Obstacle : public VisionObject {

            enum class Team { // TODO: Rename to TeamColour?
                NONE,
                MAGENTA,
                CYAN
            } team = Team::NONE;

            utility::math::geometry::Polygon polygon;
        };

    }
}

#endif // MESSAGE_VISION_VISIONOBJECTS_H
