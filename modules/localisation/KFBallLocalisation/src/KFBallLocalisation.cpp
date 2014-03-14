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

    double triangle_wave(double t, double period) {
        auto a = period; // / 2.0;
        auto k = t / a;
        return 2.0 * std::abs(2.0 * (k - std::floor(k + 0.5))) - 1.0;
    }
    double sawtooth_wave(double t, double period) {
        return 2.0 * std::fmod(t / period, 1.0) - 1.0;
    }
    double square_wave(double t, double period) {
        return std::copysign(1.0, sawtooth_wave(t, period));
    }

    KFBallLocalisation::KFBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           Options<Sync<KFBallLocalisation>>>("NUbugger Output", [this](const time_t&) {
            // emit(std::make_unique<messages::LMissile>());
            // std::cout << __PRETTY_FUNCTION__ << ": rand():" << rand() << std::endl;

            arma::vec::fixed<localisation::BallModel::size> state = engine_.ball_filter_.get();
            auto cov = engine_.ball_filter_.getCovariance();

            // // NUClear::log("=====================", "Covariance Matrix\n", cov);
            // NUClear::log("=====================", "Number of models: ",
            //              engine_.robot_models_.hypotheses().size());
            // for (auto& model : engine_.robot_models_.hypotheses()) {
            //     NUClear::log("    ", *model);
            // }

            auto now = NUClear::clock::now();
            auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
            double t = ms / 1000.0;
            double secs = 40;
            // marker_ = { 2 * cos(t / (1000.0 * secs)), 2 * sin(t / (1000.0 * secs)) };
            // marker_ = { 6 * cos(t / (1000.0 * secs)), 0 };


            auto triangle1 = triangle_wave(t, secs);
            auto triangle2 = triangle_wave(t + (secs / 4.0), secs);
            marker_ = { triangle1 * 3, triangle2 * 2 };

            auto velocity_x = -square_wave(t, secs) * ((3 * 4) / secs);
            auto velocity_y = -square_wave(t + (secs / 4.0), secs) * ((2 * 4) / secs);

            arma::vec2 diff = { marker_[0] - state[0], marker_[1] - state[1] };
            float distance = arma::norm(diff, 2);
            emit(graph("Localisation estimate distance error", distance));
            emit(graph("Estimated ball position", state[0], state[1]));
            emit(graph("Actual ball position", marker_[0], marker_[1]));
            emit(graph("Estimated ball velocity", state[2], state[3]));
            emit(graph("Actual ball velocity", velocity_x, velocity_y));

            auto ball_msg = std::make_unique<messages::localisation::FieldObject>();
            std::vector<messages::localisation::FieldObject::Model> ball_msg_models;
            messages::localisation::FieldObject::Model ball_model;
            ball_msg->name = "ball";
            ball_model.wm_x = state[0];
            ball_model.wm_y = state[1];
            ball_model.heading = 0;
            ball_model.sd_x = 0.1;
            ball_model.sd_y = 0.1;
            ball_model.sr_xx = cov(0, 0);
            ball_model.sr_xy = cov(0, 1);
            ball_model.sr_yy = cov(1, 1);
            ball_model.lost = false;
            ball_msg_models.push_back(ball_model);

            messages::localisation::FieldObject::Model ball_marker_model;
            ball_msg->name = "ball";
            ball_marker_model.wm_x = marker_[0];
            ball_marker_model.wm_y = marker_[1];
            ball_marker_model.heading = 0;
            ball_marker_model.sd_x = 0.01;
            ball_marker_model.sd_y = 0.01;
            ball_marker_model.sr_xx = 0.01;
            ball_marker_model.sr_xy = 0;
            ball_marker_model.sr_yy = 0.01;
            ball_marker_model.lost = false;
            ball_msg_models.push_back(ball_marker_model);

            ball_msg->models = ball_msg_models;
            emit(std::move(ball_msg));
        });


        // Simulate Vision
        on<Trigger<Every<500, std::chrono::milliseconds>>,
           Options<Sync<KFBallLocalisation>>>("Vision Simulation - Ball", [this](const time_t&) {

            auto camera_pos = arma::vec3 { 0.0, 0.0, 0.0 };
            double camera_heading = 0.0; // 3.1415926535;

            auto ball_pos = arma::vec3 { marker_[0], marker_[1], 0.0 };

            auto ball = std::make_unique<messages::vision::Ball>();

            // (dist, bearing, declination)
            ball->sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(ball_pos - camera_pos);
            ball->sphericalFromNeck[1] = utility::math::angle::normalizeAngle(ball->sphericalFromNeck[1] - camera_heading);
            ball->sphericalError = { 0.0001, 0.0001, 0.000001 };

            emit(std::move(ball));
        });


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
