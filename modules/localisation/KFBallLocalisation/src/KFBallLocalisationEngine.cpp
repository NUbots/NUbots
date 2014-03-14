/*
 * This file is part of KFBallLocalisation.
 *
 * KFBallLocalisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KFBallLocalisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with KFBallLocalisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KFBallLocalisationEngine.h"

#include "messages/vision/VisionObjects.h"

using messages::vision::VisionObject;

namespace modules {
namespace localisation {
	
/// Integrate time-dependent observations on all objects
void KFBallLocalisationEngine::TimeUpdate(time_t current_time) {
    arma::vec3 tmp = { 0, 0, 0 };
    ball_filter_.timeUpdate(0.1, tmp);
}

double KFBallLocalisationEngine::MeasurementUpdate(
    const messages::vision::VisionObject& observed_object) {

    // // Radial coordinates
    // arma::vec2 measurement = { observed_object.sphericalFromNeck[0],
    //                            observed_object.sphericalFromNeck[1] };
    // arma::mat22 cov = { observed_object.sphericalError[0], 0,
    //                     0, observed_object.sphericalError[1] };

    // Distance and unit vector heading
    arma::vec3 measurement = { observed_object.sphericalFromNeck[0],
                               std::cos(observed_object.sphericalFromNeck[1]),
                               std::sin(observed_object.sphericalFromNeck[1]) };
    arma::mat33 cov = { observed_object.sphericalError[0], 0, 0,
                        0, observed_object.sphericalError[1], 0,
                        0, 0, observed_object.sphericalError[1] };

    // // Robot relative cartesian coordinates
    // auto dist = observed_object.sphericalFromNeck[0];
    // auto heading = observed_object.sphericalFromNeck[1];
    // arma::vec2 measurement = { dist * std::cos(heading),
    //                            dist * std::sin(heading) };
    // auto dist_error = observed_object.sphericalError[0];
    // auto heading_error = observed_object.sphericalError[1];
    // arma::mat22 cov = { dist_error * heading_error, 0,
    //                     0, dist_error * heading_error };

    double quality = ball_filter_.measurementUpdate(measurement, cov);

    return quality;
}

}
}
