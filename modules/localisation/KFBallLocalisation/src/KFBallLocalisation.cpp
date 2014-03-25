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

#include "KFBallLocalisation.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/NUbugger/NUgraph.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "BallModel.h"

using utility::nubugger::graph;
using messages::support::Configuration;

namespace modules {
namespace localisation {

    KFBallLocalisation::KFBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           Options<Sync<KFBallLocalisation>>>("NUbugger Output", [this](const time_t&) {

            auto ball_msg = std::make_unique<messages::localisation::Ball>();

            auto model_state = engine_.ball_filter_.get();
            auto model_cov = engine_.ball_filter_.getCovariance();

            ball_msg->position = model_state.rows(0, 1);
            ball_msg->velocity = model_state.rows(2, 3);
            ball_msg->sr_xx = model_cov(0, 0);
            ball_msg->sr_xy = model_cov(0, 1);
            ball_msg->sr_yy = model_cov(1, 1);

            emit(std::move(ball_msg));
        });


        // // Simulate Vision
        // on<Trigger<Every<500, std::chrono::milliseconds>>,
        //    Options<Sync<KFBallLocalisation>>>("Vision Simulation - Ball", [this](const time_t&) {

        //     auto camera_pos = arma::vec3 { 0.0, 0.0, 0.0 };
        //     double camera_heading = 0.0; // 3.1415926535;

        //     auto ball_pos = arma::vec3 { marker_[0], marker_[1], 0.0 };

        //     auto ball = std::make_unique<messages::vision::Ball>();

        //     // (dist, bearing, declination)
        //     ball->sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(ball_pos - camera_pos);
        //     ball->sphericalFromNeck[1] = utility::math::angle::normalizeAngle(ball->sphericalFromNeck[1] - camera_heading);
        //     ball->sphericalError = { 0.0001, 0.0001, 0.000001 };

        //     emit(std::move(ball));
        // });


        // on<Trigger<Every<250, std::chrono::milliseconds>>,
        //    With<messages::vision::Ball>,
        //    Options<Sync<KFBallLocalisation>>
        //   >("KFBallLocalisation Step",
        //     [this](const time_t&,
        //            const messages::vision::Ball& ball) {

        //     // engine_.TimeUpdate(0.5);
        //     engine_.MeasurementUpdate(ball);
        // });

       on<Trigger<messages::vision::Ball>,
           Options<Sync<KFBallLocalisation>>
          >("KFBallLocalisation Step",
            [this](const messages::vision::Ball& ball) {

            // engine_.TimeUpdate(0.5);
            engine_.MeasurementUpdate(ball);
        });


        on<Trigger<Every<100, std::chrono::milliseconds>>,
           Options<Sync<KFBallLocalisation>>
           >([this](const time_t&) {
            engine_.TimeUpdate(0.1);
        });
    }
}
}
