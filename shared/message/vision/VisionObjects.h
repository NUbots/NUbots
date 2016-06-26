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

            // Time the image was taken
            NUClear::clock::time_point timestamp;

            // The angular position and size from the perspective of the camera
            // Use these values to move the camera around to see this object
            arma::vec2 screenAngular;
            arma::vec2 angularSize;

            // The sensor frame that was used to detect this object
            std::shared_ptr<const message::input::Sensors> sensors;

            // The classified image that was used to detect this object
            // TODO: Why is this here?
            std::shared_ptr<const message::vision::ClassifiedImage<message::vision::ObjectClass>> classifiedImage;
        };

        struct Ball : public VisionObject {

            std::vector<arma::vec3> edgePoints;
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

            enum class MeasurementType {
                LEFT_NORMAL,
                RIGHT_NORMAL,
                TOP_NORMAL,
                BASE_NORMAL
            };

            std::vector<std::pair<MeasurementType, arma::vec3>> measurements;
            utility::math::geometry::Quad quad;
        };

        struct Obstacle : public VisionObject {

            enum class Team {
                NONE,
                MAGENTA,
                CYAN
            } team = Team::NONE;

            utility::math::geometry::Polygon polygon;
        };

    }
}

#endif // MESSAGE_VISION_VISIONOBJECTS_H
