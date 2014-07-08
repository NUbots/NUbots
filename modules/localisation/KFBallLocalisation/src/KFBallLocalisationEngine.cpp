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
#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"

using messages::vision::VisionObject;
using messages::localisation::FakeOdometry;
using utility::time::TimeDifferenceSeconds;

namespace modules {
namespace localisation {

/// Integrate time-dependent observations on all objects
void KFBallLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time) {
    double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
    last_time_update_time_ = current_time;
    ball_filter_.timeUpdate(seconds);
}

void KFBallLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time,
                                          const FakeOdometry&) {
    double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
    last_time_update_time_ = current_time;
    ball_filter_.timeUpdate(seconds); // TODO odometry was removed from here odom
}

double KFBallLocalisationEngine::MeasurementUpdate(
    const messages::vision::VisionObject& observed_object) {

    // // Radial coordinates
    // arma::vec2 measurement = { observed_object.sphericalFromNeck[0],
    //                            observed_object.sphericalFromNeck[1] };
    // arma::mat22 cov = { observed_object.sphericalError[0], 0,
    //                     0, observed_object.sphericalError[1] };

    // Distance and unit vector heading
    // arma::vec3 measurement = { groundDist,
    //                            std::cos(observed_object.sphericalFromNeck[1]),
    //                            std::sin(observed_object.sphericalFromNeck[1]) };
    // arma::mat33 cov = { observed_object.sphericalError[0], 0, 0,
    //                     0, observed_object.sphericalError[1], 0,
    //                     0, 0, observed_object.sphericalError[1] };

    // // Robot relative cartesian coordinates
    arma::vec2 measurement = observed_object.measurements[0].position.rows(0, 1);
    arma::mat22 cov = observed_object.measurements[0].error.submat(0, 0, 1, 1);

    double quality = ball_filter_.measurementUpdate(measurement, cov);

    return quality;
}

void KFBallLocalisationEngine::UpdateConfiguration(
    const messages::support::Configuration<KFBallLocalisationEngineConfig>& config) {
    ball_filter_.model.ballDragCoefficient = config["BallDragCoefficient"].as<double>();
    cfg_.emit_ball_fieldobjects = config["EmitBallFieldobjects"].as<bool>();
}

bool KFBallLocalisationEngine::CanEmitFieldObjects() {
    return cfg_.emit_ball_fieldobjects;
}

}
}
