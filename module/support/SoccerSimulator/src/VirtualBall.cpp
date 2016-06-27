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

#include "VirtualBall.h"

#include <armadillo>

#include "message/vision/VisionObjects.h"
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "utility/math/coordinates.h"
#include "utility/math/vision.h"

namespace module {
namespace support {

    using message::vision::Ball;
    using message::input::Sensors;
    using utility::math::matrix::Transform2D;
    using message::input::CameraParameters;

    VirtualBall::VirtualBall()
    : position(arma::fill::zeros)
    , velocity(arma::fill::zeros)
    , diameter(0.1) {
    }

    VirtualBall::VirtualBall(arma::vec2 position, float diameter)
    : position(position)
    , velocity(arma::fill::zeros)
    , diameter(diameter) {
    }

    // utility::math::matrix::Transform2D ballPose;
    arma::vec3 position;
    arma::vec3 velocity;

    // arma::vec2 position;
    float diameter;

    Ball VirtualBall::detect(const CameraParameters& /*camParams*/, Transform2D /*robotPose*/, std::shared_ptr<const Sensors> sensors, arma::vec4 /*error*/){
        Ball result;

        //auto visibleMeasurements = computeVisible(position,camParams,robotPose,sensors,error);

        // TODO: set timestamp, sensors, classifiedImage?
        /*for (auto& m : visibleMeasurements.measurements){
            m.velocity.rows(0,1) = robotPose.rotation().i() * velocity.rows(0,1);
            m.velCov = 0.1 * arma::eye(3,3);
            result.measurements.push_back(m);
        }*/


        //result.screenAngular = visibleMeasurements.screenAngular;
        //result.angularSize = arma::vec2({0, 0});
        result.sensors = sensors;
        result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.


        //If no measurements are in the Ball, then there it was not observed
        return result;
    }

}
}
