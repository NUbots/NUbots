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

#include "KFBallLocalisationEngine.h"
#include <chrono>
#include "utility/time/time.h"
#include "utility/math/coordinates.h"
#include "utility/motion/ForwardKinematics.h"
#include "message/vision/VisionObjects.h"
#include "message/localisation/FieldObject.h"

using utility::math::coordinates::cartesianToSpherical;
using utility::math::coordinates::sphericalToCartesian;
using extension::Configuration;

using message::vision::VisionObject;
// using message::localisation::FakeOdometry;
using utility::time::TimeDifferenceSeconds;

namespace module {
namespace localisation {

/// Integrate time-dependent observations on all objects
void KFBallLocalisationEngine::TimeUpdate(NUClear::clock::time_point current_time) {
    double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
    last_time_update_time_ = current_time;
    ball_filter_.timeUpdate(seconds);
}


// void KFBallLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time,
//                                           const FakeOdometry&) {
//     double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
//     last_time_update_time_ = current_time;
//     ball_filter_.timeUpdate(seconds); // TODO odometry was removed from here odom
// }

double KFBallLocalisationEngine::MeasurementUpdate(const VisionObject& observed_object) {
    double quality = 1.0;

    for (auto& measurement : observed_object.measurements) {
        // Spherical from ground:
        auto currentState = ball_filter_.get();

        double ballAngle = 0;
        if (0 != currentState(1) || 0 != currentState(0)) {
            ballAngle = std::atan2(currentState(1), currentState(0));
        }

        arma::vec3 measuredPosCartesian = sphericalToCartesian(measurement.position);
        arma::vec2 cartesianImuObservation2d = observed_object.sensors->robotToIMU * measuredPosCartesian.rows(0,1);
        arma::vec3 cartesianImuObservation = arma::vec3({cartesianImuObservation2d(0),
                                                         cartesianImuObservation2d(1),
                                                         measuredPosCartesian(2)});
        arma::vec3 sphericalImuObservation = cartesianToSpherical(cartesianImuObservation);
        sphericalImuObservation(1) -= ballAngle;
        arma::mat33 cov = measurement.error;

        //Old measurement
        // quality *= ball_filter_.measurementUpdate(sphericalImuObservation, cov, ballAngle);

        //new measurement
        //add velocity before measurement:
        arma::vec posVel = arma::join_cols(sphericalImuObservation, measurement.velocity.rows(0,1));
        arma::mat posVelCov = arma::eye(5,5);
        posVelCov.submat(0,0,2,2) = cov;
        posVelCov.submat(3,3,4,4) = measurement.velCov.submat(0,0,1,1);
        quality *= ball_filter_.measurementUpdate(posVel, posVelCov, ballAngle);
    }

    return quality;
}

void KFBallLocalisationEngine::UpdateConfiguration(const Configuration& config) {
    cfg_.emitBallFieldobjects = config["EmitBallFieldobjects"].as<bool>();

    ball::BallModel::Config ball_cfg;
    ball_cfg.ballDragCoefficient = config["BallDragCoefficient"].as<double>();
    ball_cfg.processNoisePositionFactor = config["ProcessNoisePositionFactor"].as<double>();
    ball_cfg.processNoiseVelocityFactor = config["ProcessNoiseVelocityFactor"].as<double>();
    ball_cfg.ballHeight = config["BallHeight"].as<double>();
    ball_filter_.model.cfg_ = ball_cfg;
}

bool KFBallLocalisationEngine::CanEmitFieldObjects() {
    return cfg_.emitBallFieldobjects;
}

}
}
