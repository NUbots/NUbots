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
#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/support/Configuration.h"
#include "message/localisation/FieldObject.h"
#include "BallModel.h"
#include "utility/localisation/transform.h"
#include "utility/nubugger/NUhelpers.h"

using message::localisation::Self;
using utility::nubugger::graph;
using message::input::Sensors;
using message::support::Configuration;
// using message::localisation::FakeOdometry;
using message::localisation::Ball;
using utility::nubugger::drawArrow;


namespace module {
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


        on<Configuration>("KFBallLocalisationEngine.yaml").then([this](const Configuration& config) {
            engine_.UpdateConfiguration(config);
        });

        // Emit to NUbugger
       emit_data_handle = on<Every<100, std::chrono::milliseconds>,
           With<Sensors>,
           With<std::vector<Self>>,
           Sync<KFBallLocalisation>>().then("NUbugger Output", [this](const Sensors& sensors, const std::vector<Self>& robots) {

            arma::vec model_state = engine_.ball_filter_.get();
            arma::mat model_cov = engine_.ball_filter_.getCovariance();

            arma::mat22 imu_to_robot = sensors.robotToIMU.t();
            arma::vec2 robot_space_ball_pos = imu_to_robot * model_state.rows(0, 1);
            arma::vec2 robot_space_ball_vel = imu_to_robot * model_state.rows(2, 3);

            message::localisation::Ball ball;
            ball.position = robot_space_ball_pos;
            ball.velocity = robot_space_ball_vel + robots[0].velocity;
            ball.position_cov = model_cov.submat(0,0,1,1);
            ball.world_space = false;

            ball.last_measurement_time = last_measurement_time;

            auto ball_msg = std::make_unique<Ball>(ball);
            auto ball_vec_msg = std::make_unique<std::vector<Ball>>();
            ball_vec_msg->push_back(ball);
            emit(std::move(ball_msg));
            emit(std::move(ball_vec_msg));

            arma::vec3 worldSpaceBallPos = arma::zeros(3);
            worldSpaceBallPos.rows(0,1) = utility::localisation::transform::RobotToWorldTransform(robots[0].position, robots[0].heading, robot_space_ball_pos);
            arma::vec3 worldSpaceBallVel = arma::zeros(3);
            worldSpaceBallVel.rows(0,1) = utility::localisation::transform::RobotToWorldTransform(arma::zeros(2), robots[0].heading, robot_space_ball_vel);

            emit(drawArrow("ballvel", worldSpaceBallPos, worldSpaceBallVel, arma::norm(worldSpaceBallVel)));

            emit(graph("Localisation Ball", model_state(0), model_state(1)));
            emit(graph("Localisation Ball Velocity", model_state(2), model_state(3)));
        });

        //Disable until first data
        emit_data_handle.disable();

        // on<Trigger<FakeOdometry>,
        //     Sync<KFBallLocalisation>
        //     >("KFBallLocalisation Odometry", [this](const FakeOdometry& odom) {
        //      auto curr_time = NUClear::clock::now();
        //      engine_.TimeUpdate(curr_time, odom);
        //  });

        on<Every<100, Per<std::chrono::seconds>>, Sync<KFBallLocalisation>>().then("KFBallLocalisation Time", [this] {
            auto curr_time = NUClear::clock::now();
            engine_.TimeUpdate(curr_time);
        });

        on<Trigger<std::vector<message::vision::Ball>>,
             Sync<KFBallLocalisation>
             >().then("KFBallLocalisation Step",
                [this](const std::vector<message::vision::Ball>& balls) {

            //Is this check necessary?
            if(!emit_data_handle.enabled()){
                emit_data_handle.enable();
            }

            if(balls.size() > 0) {
                auto curr_time = NUClear::clock::now();
                last_measurement_time = curr_time;
                engine_.TimeUpdate(curr_time);

                engine_.MeasurementUpdate(balls[0]);
                //DEBUG
                // for(auto& m : balls[0].measurements){
                //     log("ball measurement:", m.position, m.error, m.velocity, m.velCov);
                // }

            }
        });
    }
}
}
