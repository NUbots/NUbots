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

#include "MockRobot.h"
#include <nuclear>
#include "utility/math/angle.h"
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/localisation/transform.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
// #include "BallModel.h"
#include "utility/localisation/transform.h"

using utility::math::matrix::rotationMatrix;
using utility::math::angle::normalizeAngle;
using utility::math::angle::vectorToBearing;
using utility::math::angle::bearingToUnitVector;
using utility::math::coordinates::cartesianToSpherical;
using utility::localisation::transform::WorldToRobotTransform;
using utility::localisation::transform::RobotToWorldTransform;
using utility::nubugger::graph;
using messages::support::Configuration;
using messages::support::FieldDescription;
using modules::localisation::MockRobotConfig;
using messages::localisation::Mock;

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
    double sine_wave(double t, double period) {
        return std::sin((2.0 * M_PI * t) / period);
    }
    double absolute_time() {
        auto now = NUClear::clock::now();
        auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
        double t = ms / 1000.0;
        return t;
    }

    void MockRobot::UpdateConfiguration(
        const messages::support::Configuration<MockRobotConfig>& config) {
        cfg_.simulate_vision = config["SimulateVision"].as<bool>();
        cfg_.simulate_goal_observations = config["SimulateGoalObservations"].as<bool>();
        cfg_.simulate_ball_observations = config["SimulateBallObservations"].as<bool>();
        cfg_.simulate_odometry = config["SimulateOdometry"].as<bool>();
        cfg_.simulate_robot_movement = config["SimulateRobotMovement"].as<bool>();
        cfg_.simulate_ball_movement = config["SimulateBallMovement"].as<bool>();
        cfg_.emit_robot_fieldobjects = config["EmitRobotFieldobjects"].as<bool>();
        cfg_.emit_ball_fieldobjects = config["EmitBallFieldobjects"].as<bool>();

        NUClear::log(__func__, "cfg_.simulate_vision = ", cfg_.simulate_vision);
        NUClear::log(__func__, "cfg_.simulate_goal_observations = ", cfg_.simulate_goal_observations);
        NUClear::log(__func__, "cfg_.simulate_ball_observations = ", cfg_.simulate_ball_observations);
        NUClear::log(__func__, "cfg_.simulate_odometry = ", cfg_.simulate_odometry);
        NUClear::log(__func__, "cfg_.simulate_robot_movement = ", cfg_.simulate_robot_movement);
        NUClear::log(__func__, "cfg_.simulate_ball_movement = ", cfg_.simulate_ball_movement);
        NUClear::log(__func__, "cfg_.emit_robot_fieldobjects = ", cfg_.emit_robot_fieldobjects);
        NUClear::log(__func__, "cfg_.emit_ball_fieldobjects = ", cfg_.emit_ball_fieldobjects);
    }

    MockRobot::MockRobot(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {
            field_description_ = std::make_shared<FieldDescription>(desc);
        });

        on<Trigger<Configuration<MockRobotConfig>>>(
            "MockRobotConfig Update",
            [this](const Configuration<MockRobotConfig>& config) {
            UpdateConfiguration(config);
        });

        // Update robot position
        on<Trigger<Every<10, std::chrono::milliseconds>>>("Robot motion", [this](const time_t&){
            if (!cfg_.simulate_robot_movement) {
                robot_velocity_ = { 0, 0 };
                return;
            }

            auto t = absolute_time();
            double period = 100;
            double x_amp = 3;
            double y_amp = 2;

            arma::vec old_pos = robot_position_;

            auto wave1 = triangle_wave(t, period);
            auto wave2 = triangle_wave(t + (period / 4.0), period);
            // auto wave1 = sine_wave(t, period);
            // auto wave2 = sine_wave(t + (period / 4.0), period);
            robot_position_ = { wave1 * x_amp, wave2 * y_amp };

            arma::vec diff = robot_position_ - old_pos;

            robot_heading_ = vectorToBearing(diff);
            robot_velocity_ = robot_heading_ / 100.0;
        });

        // Update ball position
        on<Trigger<Every<10, std::chrono::milliseconds>>>("Ball motion", [this](const time_t&){

            if (!cfg_.simulate_ball_movement) {
                ball_velocity_ = { 0, 0 };
                return;
            }


            auto t = absolute_time();
            double period = 40;
            double x_amp = 3;
            double y_amp = 2;

            auto triangle1 = triangle_wave(t, period);
            auto triangle2 = triangle_wave(t + (period / 4.0), period);
            ball_position_ = { triangle1 * x_amp, triangle2 * y_amp };

            auto velocity_x = -square_wave(t, period) * ((x_amp * 4) / period);
            auto velocity_y = -square_wave(t + (period / 4.0), period) * ((y_amp * 4) / period);
            ball_velocity_ = { velocity_x, velocity_y };
        });

//         // Simulate orientation matrix
//         on<Trigger<Every<10, std::chrono::milliseconds>>>(
//             "Orientation Matrix Simulation", [this](const time_t&){
// // orient =   M: W -> R
// //          M^T: R -> W

// //          a, x_w,  M*R_a*x_w = x_r

// // M is an orthonormal basis for world coords expressed in robot coords
// // i.e. M contains unit vectors pointing along each of the world axes
// // Note: M can only attempt to track the robot's orientation - not its position.
// //       i.e. The origin of the world coords resulting from M is still the
// //            robot's torso, but the axes are parallel to the field axes.
//             arma::mat33 M =


//         });

        // Simulate Odometry
        on<Trigger<Every<100, std::chrono::milliseconds>>>("Odometry Simulation",
            [this](const time_t&) {
            if (!cfg_.simulate_odometry)
                return;

            auto odom = std::make_unique<messages::localisation::FakeOdometry>();

            double heading_diff = robot_heading_ - odom_old_robot_heading_;
            odom->torso_rotation = normalizeAngle(heading_diff);

            // Calculate torso displacement in robot-space:
            arma::vec2 position_diff = robot_position_ - odom_old_robot_position_;
            arma::mat22 rot = rotationMatrix(robot_heading_);
            odom->torso_displacement = rot * position_diff;

            odom_old_robot_position_ = robot_position_;
            odom_old_robot_heading_ = robot_heading_;

            emit(graph("Odometry torso_displacement",
                odom->torso_displacement[0],
                odom->torso_displacement[1]));
            emit(graph("Odometry torso_rotation", odom->torso_rotation));

            emit(std::move(odom));
        });

        // Simulate Vision
        on<Trigger<Every<30, Per<std::chrono::seconds>>>,
           Options<Sync<MockRobot>>>("Vision Simulation", [this](const time_t&) {
            if (!cfg_.simulate_vision)
                return;

            if (field_description_ == nullptr) {
                NUClear::log(__FILE__, __LINE__, ": field_description_ == nullptr");
                return;
            }

            // Camera setup
            auto camera_pos = arma::vec3 { robot_position_[0], robot_position_[1], 0.0 };
            // double camera_heading = std::atan2(robot_heading_[1], robot_heading_[0]);

            // Goal observation
            if (cfg_.simulate_goal_observations) {
                auto& fd = field_description_;
                // auto goal1_pos = arma::vec3 { fd->goalpost_br[0], fd->goalpost_br[1], 0.0 };
                // auto goal2_pos = arma::vec3 { fd->goalpost_bl[0], fd->goalpost_bl[1], 0.0 };

                auto goal1 = messages::vision::Goal();
                auto goal2 = messages::vision::Goal();
                goal1.side = messages::vision::Goal::Side::RIGHT;
                goal2.side = messages::vision::Goal::Side::LEFT;

                messages::vision::VisionObject::Measurement g1_m;
                auto g1_pos_2d = WorldToRobotTransform(robot_position_, robot_heading_, fd->goalpost_br);
                auto g1_pos_cartesian = arma::vec3({ g1_pos_2d(0), g1_pos_2d(1), 0 });
                g1_m.position = cartesianToSpherical(g1_pos_cartesian);

                messages::vision::VisionObject::Measurement g2_m;
                auto g2_pos_2d = WorldToRobotTransform(robot_position_, robot_heading_, fd->goalpost_bl);
                auto g2_pos_cartesian = arma::vec3({ g2_pos_2d(0), g2_pos_2d(1), 0 });
                g2_m.position = cartesianToSpherical(g2_pos_cartesian);

                g1_m.error = arma::eye(3, 3) * 0.1;
                g2_m.error = arma::eye(3, 3) * 0.1;
                goal1.measurements.push_back(g1_m);
                goal2.measurements.push_back(g2_m);

                auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

                goals->push_back(goal1);
                goals->push_back(goal2);

                emit(std::move(goals));
            }

            // Ball observation
            if (cfg_.simulate_ball_observations) {

                auto ball_vec = std::make_unique<std::vector<messages::vision::Ball>>();

                // Observations in spherical from ground reference: (dist, bearing, declination)
                messages::vision::Ball ball;
                messages::vision::VisionObject::Measurement b_m;
                auto ball_pos_2d = WorldToRobotTransform(robot_position_, robot_heading_, ball_position_);
                auto ball_pos_cartesian = arma::vec3({ ball_pos_2d(0), ball_pos_2d(1), field_description_->ball_radius });
                b_m.position = cartesianToSpherical(ball_pos_cartesian);
                b_m.error = arma::eye(3, 3) * 0.1;

                ball.measurements.push_back(b_m);
                ball_vec->push_back(ball);

                emit(std::move(ball_vec));
            }
        });

        // Emit robot to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Mock<std::vector<messages::localisation::Self>>>,
           Options<Sync<MockRobot>>>("NUbugger Output",
            [this](const time_t&,
                   const Mock<std::vector<messages::localisation::Self>>& mock_robots) {

            auto& robots = mock_robots.data;

            emit(graph("Actual robot position", robot_position_[0], robot_position_[1]));
            // emit(graph("Actual robot heading", robot_heading_[0], robot_heading_[1]));
            emit(graph("Actual robot heading", robot_heading_));
            emit(graph("Actual robot velocity", robot_velocity_[0], robot_velocity_[1]));

            if (robots.size() >= 1) {
                emit(graph("Estimated robot position", robots[0].position[0], robots[0].position[1]));
                emit(graph("Estimated robot heading", robots[0].heading[0], robots[0].heading[1]));
            }

            // Robot message
            if (!cfg_.emit_robot_fieldobjects)
                return;

            auto robots_msg = std::make_unique<std::vector<messages::localisation::Self>>();
            
            for (auto& model : robots) {
                robots_msg->push_back(model);
            }

            messages::localisation::Self self_marker;
            self_marker.position = robot_position_;
            self_marker.heading = bearingToUnitVector(robot_heading_);
            self_marker.sr_xx = 0.01;
            self_marker.sr_xy = 0;
            self_marker.sr_yy = 0.01;
            robots_msg->push_back(self_marker);

            emit(std::move(robots_msg));
        });

        // Emit ball to Nubugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Mock<messages::localisation::Ball>>,
           With<Mock<std::vector<messages::localisation::Self>>>,
           Options<Sync<MockRobot>>>("NUbugger Output",
            [this](const time_t&,
                   const Mock<messages::localisation::Ball>& mock_ball,
                   const Mock<std::vector<messages::localisation::Self>>& mock_robots) {


            auto& ball = mock_ball.data;
            auto& robots = mock_robots.data;



            if (robots.empty())
                return;

            arma::vec2 robot_ball_pos = RobotToWorldTransform(
                robots[0].position, robots[0].heading, ball.position);
            arma::vec2 ball_pos = RobotToWorldTransform(
                robot_position_, robot_heading_, ball.position);
            emit(graph("Estimated ball position", ball_pos[0], ball_pos[1]));
            // emit(graph("Estimated ball velocity", state[2], state[3]));
            emit(graph("Actual ball position", ball_position_[0], ball_position_[1]));
            emit(graph("Actual ball velocity", ball_velocity_[0], ball_velocity_[1]));

            // Ball message
            if (!cfg_.emit_ball_fieldobjects)
                return;
            
            auto balls_msg = std::make_unique<std::vector<messages::localisation::Ball>>();

            messages::localisation::Ball ball_model;
            ball_model.position = ball_pos;
            ball_model.velocity = ball_velocity_;
            ball_model.sr_xx = ball.sr_xx;
            ball_model.sr_xy = ball.sr_xy;
            ball_model.sr_yy = ball.sr_yy;
            ball_model.world_space = true;
            balls_msg->push_back(ball_model);

            messages::localisation::Ball ball_marker;
            ball_marker.position = ball_position_;
            ball_marker.velocity = ball_velocity_;
            ball_marker.sr_xx = 0.01;
            ball_marker.sr_xy = 0;
            ball_marker.sr_yy = 0.01;
            ball_marker.world_space = true;
            balls_msg->push_back(ball_marker);

            messages::localisation::Ball robot_ball;
            robot_ball.position = robot_ball_pos;
            robot_ball.velocity = ball_velocity_;
            robot_ball.sr_xx = 0.01;
            robot_ball.sr_xy = 0;
            robot_ball.sr_yy = 0.01;
            robot_ball.world_space = true;
            balls_msg->push_back(robot_ball);

            emit(std::move(balls_msg));
        });
    }
}
}

