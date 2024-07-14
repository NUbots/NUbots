/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "WalkToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::WalkTo;
    using message::strategy::WalkToFieldPosition;
    using WalkToBallTask     = message::strategy::WalkToBall;
    using WalkToKickBallTask = message::strategy::WalkToKickBall;
    using FieldDescription   = message::support::FieldDescription;

    using utility::math::angle::angle_between;
    using utility::math::euler::pos_rpy_to_transform;
    using utility::nusight::graph;
    using utility::support::Expression;

    WalkToBall::WalkToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToBall.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.ball_y_offset          = config["ball_y_offset"].as<double>();
            cfg.ball_kick_distance     = config["ball_kick_distance"].as<double>();
            cfg.ball_approach_distance = config["ball_approach_distance"].as<double>();
            cfg.goal_target_offset     = config["goal_target_offset"].as<double>();
            cfg.max_angle_error        = config["max_angle_error"].as<Expression>();
            cfg.avoid_ball_offset      = Eigen::Vector3d(config["avoid_ball_offset"].as<Expression>());
            cfg.infront_of_ball_radius = config["infront_of_ball_radius"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Goal Position", [this](const FieldDescription& fd) {
            // Update the goal position
            rGFf = Eigen::Vector3d(-fd.dimensions.field_length / 2 - cfg.goal_target_offset, 0, 0);
        });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToBallTask>, With<Ball>, With<Sensors>>().then([this](const Ball& ball, const Sensors& sensors) {
            // If we have a ball, walk to it
            if (NUClear::clock::now() - ball.time_of_measurement < cfg.ball_search_timeout) {
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                // Add an offset to account for walking with the foot in front of the ball
                rBRr.y() += cfg.ball_y_offset;
                const double heading = std::atan2(rBRr.y(), rBRr.x());
                auto Hrb             = pos_rpy_to_transform(rBRr, Eigen::Vector3d(0, 0, heading));
                emit<Task>(std::make_unique<WalkTo>(Hrb));
            }
        });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToKickBallTask>, Optional<With<Robots>>, With<Ball>, With<Sensors>, With<Field>>().then(
            [this](const std::shared_ptr<const Robots>& robots,
                   const Ball& ball,
                   const Sensors& sensors,
                   const Field& field) {
                // If we have a ball, walk to it
                if (NUClear::clock::now() - ball.time_of_measurement < cfg.ball_search_timeout) {
                    // Position of the ball relative to the robot in the robot space
                    Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                    // Add an offset to account for walking with the foot in front of the ball
                    rBRr.y() += cfg.ball_y_offset;

                    // Position of the ball relative to the field in the field space
                    Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                    // Position of the goal relative to the ball in the field space
                    Eigen::Vector3d rGBf = rGFf - rBFf;

                    // Normalize the vector
                    Eigen::Vector3d uGBf = rGBf.normalized();

                    // Get position of robot on field
                    Eigen::Isometry3d Hfr        = field.Hfw * sensors.Hrw.inverse();
                    Eigen::Vector3d rRFf         = Hfr.translation();
                    Eigen::Vector3d robot_unit_x = Hfr.linear().col(0);

                    // Compute the heading (angle between the x-axis and the vector from the kick position to the goal)
                    double desired_heading = std::atan2(rGBf.y(), rGBf.x());

                    // Compute the error between the robot unit x and desired heading
                    double angle_error =
                        std::atan2(uGBf.y(), uGBf.x()) - std::atan2(robot_unit_x.y(), robot_unit_x.x());
                    // Normalize the angle error to be within the range [-pi, pi]
                    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

                    // Scale the distance to walk to based on the angle error
                    double angle_error_scaling_factor =
                        std::clamp(std::abs(angle_error) / cfg.max_angle_error, 0.0, 1.0);

                    // Compute position to kick
                    Eigen::Vector3d rKFf = Eigen::Vector3d::Zero();

                    // If the robot is approaching ball from in front, move to the side of the ball
                    auto Hfk = Eigen::Isometry3d::Identity();
                    if (rBFf.x() > rRFf.x()) {
                        Eigen::Vector3d offset_left  = cfg.avoid_ball_offset;
                        Eigen::Vector3d offset_right = cfg.avoid_ball_offset.cwiseProduct(Eigen::Vector3d(1, -1, 1));
                        Eigen::Vector3d rArFf        = rBFf + offset_right;
                        Eigen::Vector3d rAlFf        = rBFf + offset_left;

                        // Select point closest to robot to avoid ball
                        if ((rRFf - rArFf).norm() < (rRFf - rAlFf).norm()) {
                            // Compute desired heading to face the final approach point
                            rKFf = rBFf - uGBf * cfg.ball_kick_distance - uGBf * cfg.ball_approach_distance;
                            Eigen::Vector3d rKRf = rKFf - rArFf;
                            double heading       = std::atan2(rKRf.y(), rKRf.x());
                            Hfk                  = pos_rpy_to_transform(rArFf, Eigen::Vector3d(0, 0, heading));
                        }
                        else {
                            // Compute desired heading to face the final approach point
                            rKFf = rBFf - uGBf * cfg.ball_kick_distance - uGBf * cfg.ball_approach_distance;
                            Eigen::Vector3d rKLf = rKFf - rAlFf;
                            double heading       = std::atan2(rKLf.y(), rKLf.x());
                            Hfk                  = pos_rpy_to_transform(rAlFf, Eigen::Vector3d(0, 0, heading));
                        }
                    }
                    else if (std::abs(angle_error) > cfg.max_angle_error) {
                        rKFf = rBFf - uGBf * cfg.ball_kick_distance - uGBf * cfg.ball_approach_distance;
                        Hfk  = pos_rpy_to_transform(rKFf, Eigen::Vector3d(0, 0, desired_heading));
                    }
                    else {
                        // Move towards kick distance, where ball_approach_distance is scaled by angle error
                        // to ensure robot is facing the desired heading before reaching kick distance
                        rKFf = rBFf - uGBf * cfg.ball_kick_distance
                               - uGBf * cfg.ball_approach_distance * angle_error_scaling_factor;
                        Hfk = pos_rpy_to_transform(rKFf, Eigen::Vector3d(0, 0, desired_heading));
                        // If there are robots, check if there are obstacles in the way
                        if (robots != nullptr) {
                            // Get the positions of all robots in the world
                            std::vector<Eigen::Vector2d> all_obstacles{};
                            for (const auto& robot : robots->robots) {
                                all_obstacles.emplace_back((field.Hfw * robot.rRWw).head(2));
                            }
                            auto robot_infront = robot_infront_of_ball(all_obstacles, rBFf.head(2));
                            if (robot_infront.has_value()) {
                                this->log<NUClear::INFO>("Robot in front of ball", robot_infront.value());
                                // Move to the side of the ball
                                if (robot_infront.value().y() > rBFf.y()) {
                                    Hfk = pos_rpy_to_transform(rKFf, Eigen::Vector3d(0, 0, desired_heading + M_PI_4));
                                }
                                else {
                                    Hfk = pos_rpy_to_transform(rKFf, Eigen::Vector3d(0, 0, desired_heading - M_PI_4));
                                }
                            }
                        }
                    }


                    emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk));
                }
            });
    }

}  // namespace module::strategy
