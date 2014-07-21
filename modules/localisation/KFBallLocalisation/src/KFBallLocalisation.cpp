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

#include "KFBallLocalisation.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUhelpers.h"
#include "messages/input/Sensors.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "BallModel.h"

using messages::localisation::Self;
using utility::nubugger::graph;
using messages::input::Sensors;
using messages::support::Configuration;
// using messages::localisation::FakeOdometry;
using messages::localisation::Mock;
using messages::localisation::Ball;

namespace modules {
namespace localisation {

    double time_diff() {
            auto now = NUClear::clock::now();
            auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
            double t = ms / 1000.0;
            return t;
    }

    KFBallLocalisation::KFBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

        on<Trigger<Configuration<KFBallLocalisationEngineConfig>>>([this](const Configuration<KFBallLocalisationEngineConfig>& config) {
            engine_.UpdateConfiguration(config);
        });

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Sensors>,
           With<std::vector<Self>>,
           Options<Sync<KFBallLocalisation>>>("NUbugger Output",
                [this](const time_t&, const Sensors& sensors, const std::vector<Self>& robots) {
            
            auto model_state = engine_.ball_filter_.get();
            auto model_cov = engine_.ball_filter_.getCovariance();

            arma::mat22 imu_to_robot = sensors.robotToIMU.t();
            arma::vec2 robot_space_ball_pos = imu_to_robot * model_state.rows(0, 1);

            messages::localisation::Ball ball;
            ball.position = robot_space_ball_pos;
            ball.velocity = imu_to_robot * model_state.rows(2, 3) + robots[0].velocity;
            ball.position_cov = model_cov.submat(0,0,1,1);
            ball.world_space = false;

            if (engine_.CanEmitFieldObjects()) {
                auto ball_msg = std::make_unique<Ball>(ball);
                auto ball_vec_msg = std::make_unique<std::vector<Ball>>();
                ball_vec_msg->push_back(ball);
                emit(std::move(ball_msg));
                emit(std::move(ball_vec_msg));
            } else {
                Mock<Ball> mock_ball = Mock<Ball>(ball);
                auto mock_ball_msg = std::make_unique<Mock<Ball>>(mock_ball);
                emit(std::move(mock_ball_msg));
            }

            emit(graph("Localisation Ball", model_state(0), model_state(1)));
            emit(graph("Localisation Ball Velocity", model_state(2), model_state(3)));
        });

        // on<Trigger<FakeOdometry>,
        //     Options<Sync<KFBallLocalisation>>
        //     >("KFBallLocalisation Odometry", [this](const FakeOdometry& odom) {
        //      auto curr_time = NUClear::clock::now();
        //      engine_.TimeUpdate(curr_time, odom);
        //  });

        on<Trigger<Every<100, Per<std::chrono::seconds>>>,
             Options<Sync<KFBallLocalisation>>
            >("KFBallLocalisation Time", [this](const time_t&) {
            auto curr_time = NUClear::clock::now();
            engine_.TimeUpdate(curr_time);
        });

        on<Trigger<std::vector<messages::vision::Ball>>,
             Options<Sync<KFBallLocalisation>>
             >("KFBallLocalisation Step",
                [this](const std::vector<messages::vision::Ball>& balls) {

            if(balls.size() > 0) {
                auto curr_time = NUClear::clock::now();
                engine_.TimeUpdate(curr_time);

                engine_.MeasurementUpdate(balls[0]);
            }
        });
    }
}
}
