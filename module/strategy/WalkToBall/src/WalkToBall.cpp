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
    using WalkToBallTask = message::strategy::WalkToBall;
    using message::strategy::TackleBall;
    using message::strategy::WalkToKickBall;
    using FieldDescription = message::support::FieldDescription;

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
            cfg.avoid_opponent_offset  = config["avoid_opponent_offset"].as<double>();
            cfg.approach_offset        = config["approach_offset"].as<double>();
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
        on<Provide<WalkToKickBall>, With<Ball>, With<Sensors>, With<Field>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field) {
                // Skip execution if the ball hasn't been seen recently
                if (NUClear::clock::now() - ball.time_of_measurement > cfg.ball_search_timeout) {
                    return;
                }

                // Ball position relative to robot in robot frame (rBRr)
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                rBRr.y() += cfg.ball_y_offset;  // Offset for ball-walking alignment

                // Ball and goal positions in field frame
                const Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                const Eigen::Vector3d rGBf = rGFf - rBFf;        // Vector from ball to goal
                const Eigen::Vector3d uGBf = rGBf.normalized();  // Unit vector toward goal

                // Robot position and orientation in field frame
                Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf  = Hfr.translation();  // Robot position in field

                // Compute desired heading angle (towards goal from ball)
                double desired_heading  = std::atan2(rGBf.y(), rGBf.x());
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Heading error (difference between robot and desired heading)
                double angle_error =
                    std::atan2(std::sin(desired_heading - robot_heading), std::cos(desired_heading - robot_heading));

                // Compute kick target position (directly behind the ball)
                const Eigen::Vector3d kick_target = rBFf - uGBf * cfg.ball_kick_distance;

                // Final walking target pose
                Eigen::Isometry3d Hfk = Eigen::Isometry3d::Identity();

                // If robot is approaching from behind the ball (x-axis direction)
                if (rBFf.x() > rRFf.x()) {
                    // Compute left and right side positions to walk around the ball
                    Eigen::Vector3d rAlFf = rBFf + cfg.avoid_ball_offset;
                    Eigen::Vector3d rArFf = rBFf + cfg.avoid_ball_offset.cwiseProduct(Eigen::Vector3d(1, -1, 1));

                    // Choose the closer avoidance side
                    const Eigen::Vector3d& avoid_pos = (rRFf - rArFf).norm() < (rRFf - rAlFf).norm() ? rArFf : rAlFf;

                    // Walk to avoid point and face the final approach direction
                    Eigen::Vector3d approach = kick_target - uGBf * cfg.ball_approach_distance;
                    double heading           = std::atan2((approach - avoid_pos).y(), (approach - avoid_pos).x());
                    Hfk                      = pos_rpy_to_transform(avoid_pos, Eigen::Vector3d(0, 0, heading));
                }
                // If heading error is large, back off further to face goal first
                else if (std::abs(angle_error) > cfg.max_angle_error) {
                    Eigen::Vector3d approach = kick_target - uGBf * cfg.ball_approach_distance;
                    Hfk                      = pos_rpy_to_transform(approach, Eigen::Vector3d(0, 0, desired_heading));
                }
                // Otherwise, walk directly to scaled approach position
                else {
                    double angle_scale     = std::clamp(std::abs(angle_error) / cfg.max_angle_error, 0.0, 1.0);
                    Eigen::Vector3d target = kick_target - uGBf * cfg.ball_approach_distance * angle_scale;
                    Hfk                    = pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, desired_heading));
                }

                // Issue walking task toward final position and orientation
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, false));
            });


        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<TackleBall>, With<Ball>, With<Sensors>, With<Field>, With<Robots>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field, const Robots& robots) {
                // Do nothing if we can't see the ball
                if (NUClear::clock::now() - ball.time_of_measurement > cfg.ball_search_timeout) {
                    return;
                }

                // Position of the ball relative to the robot in the robot space
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                // Add an offset to account for walking with the foot in front of the ball
                rBRr.y() += cfg.ball_y_offset;

                // Position of the ball relative to the field in the field space
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                // If there are no robots, we cannot tackle the ball
                if (robots.robots.size() == 0) {
                    return;
                }

                // Find the robot in possession
                Eigen::Vector3d rOBf = (field.Hfw * robots.robots[0].rRWw) - rBFf;
                // Loop over all opponent robots and find the one closest to the ball
                for (const auto& robot : robots.robots) {
                    if (robot.teammate_id == 0) {
                        // field.Hfw * robot.rRWw -> rRFf
                        Eigen::Vector3d candidate_rOBf = (field.Hfw * robot.rRWw) - rBFf;
                        if (candidate_rOBf.norm() < rOBf.norm()) {
                            rOBf = candidate_rOBf;
                        }
                    }
                }

                // Normalise the vector
                Eigen::Vector3d uOBf = rOBf.normalized();

                // Get position of robot on field
                Eigen::Isometry3d Hfr   = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf    = Hfr.translation();
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Create left/right perpendicular unit vectors to opponent direction
                Eigen::Vector3d uLeft  = Eigen::Vector3d(-uOBf.y(), uOBf.x(), 0);  // 90 deg left
                Eigen::Vector3d uRight = Eigen::Vector3d(uOBf.y(), -uOBf.x(), 0);  // 90 deg right

                // Offset positions from the ball on both sides
                Eigen::Vector3d rLeftFf  = rBFf + uLeft * cfg.approach_offset;
                Eigen::Vector3d rRightFf = rBFf + uRight * cfg.approach_offset;

                // Choose the side to approach based on which is closer to the robot
                // Unit vector of the heading in field space
                Eigen::Vector3d uHf = (rRFf - rLeftFf).norm() < (rRFf - rRightFf).norm() ? uRight : uLeft;
                if ((rRFf - rLeftFf).norm() < (rRFf - rRightFf).norm()) {
                    log<INFO>("Tackling ball, approaching from left side");
                }
                else {
                    log<INFO>("Tackling ball, approaching from right side");
                }
                double heading = std::atan2(uHf.y(), uHf.x());
                // Error between the robot's heading and the desired heading perpendicular to the opponent
                double angle_error = std::atan2(std::sin(heading - robot_heading), std::cos(heading - robot_heading));

                // if not robot is not yet aligned, add the offset to approach to the side of the opponent
                Eigen::Vector3d rApproachFf = rBFf;
                if (std::abs(angle_error) > 0.2) {
                    // Add y offset in the direction of the perpendicular vector
                    rApproachFf -= uHf * cfg.approach_offset;
                }
                // Always add some distance backwards from opponent along opponent vector
                rApproachFf -= uOBf * cfg.avoid_opponent_offset;

                // If the distance to the ball is very small, ie we are actively tackling, the heading should be equal
                // to the robot's current heading Otherwise small changed causes the robot to recalibrate its heading by
                // walking backwards
                log<INFO>("Distance", rBRr.head<2>().norm());
                if (rBRr.head<2>().norm() < 0.5) {
                    log<INFO>("Close! So using robot's own heading");
                    heading = robot_heading;
                }

                Eigen::Isometry3d Hfk = pos_rpy_to_transform(rApproachFf, Eigen::Vector3d(0, 0, heading));
                log<INFO>("Tackling ball, walking to approach point", rApproachFf, "with heading", heading);
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, false));
            });

        // Walk and stop behind the ball, facing the goal. Avoids walking into the ball.
        on<Provide<PositionBehindBall>, With<Ball>, With<Sensors>, With<Field>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field) {
                // Skip execution if the ball hasn't been seen recently
                if (NUClear::clock::now() - ball.time_of_measurement > cfg.ball_search_timeout) {
                    return;
                }

                // Ball position relative to robot in robot frame (rBRr)
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                rBRr.y() += cfg.ball_y_offset;  // Offset for ball-walking alignment

                // Ball and goal positions in field frame
                const Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                const Eigen::Vector3d rGBf = rGFf - rBFf;        // Vector from ball to goal
                const Eigen::Vector3d uGBf = rGBf.normalized();  // Unit vector toward goal

                // Robot position and orientation in field frame
                Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d rRFf  = Hfr.translation();  // Robot position in field

                // Compute desired heading angle (towards goal from ball)
                double desired_heading  = std::atan2(rGBf.y(), rGBf.x());
                Eigen::Vector3d robot_x = Hfr.linear().col(0);  // Robot forward direction in field
                double robot_heading    = std::atan2(robot_x.y(), robot_x.x());

                // Heading error (difference between robot and desired heading)
                double angle_error =
                    std::atan2(std::sin(desired_heading - robot_heading), std::cos(desired_heading - robot_heading));

                // Compute kick target position (directly behind the ball)
                const Eigen::Vector3d kick_target = rBFf - uGBf * cfg.distance_behind_ball;

                // Final walking target pose
                Eigen::Isometry3d Hfk = Eigen::Isometry3d::Identity();

                // If robot is approaching from behind the ball (x-axis direction)
                if (rBFf.x() > rRFf.x()) {
                    // Compute left and right side positions to walk around the ball
                    Eigen::Vector3d rAlFf = rBFf + cfg.avoid_ball_offset;
                    Eigen::Vector3d rArFf = rBFf + cfg.avoid_ball_offset.cwiseProduct(Eigen::Vector3d(1, -1, 1));

                    // Choose the closer avoidance side
                    const Eigen::Vector3d& avoid_pos = (rRFf - rArFf).norm() < (rRFf - rAlFf).norm() ? rArFf : rAlFf;

                    // Walk to avoid point and face the final approach direction
                    Eigen::Vector3d approach = kick_target - uGBf * cfg.ball_approach_distance;
                    double heading           = std::atan2((approach - avoid_pos).y(), (approach - avoid_pos).x());
                    Hfk                      = pos_rpy_to_transform(avoid_pos, Eigen::Vector3d(0, 0, heading));
                }
                // Otherwise, walk directly to scaled approach position
                else {
                    double angle_scale     = std::clamp(std::abs(angle_error) / cfg.max_angle_error, 0.0, 1.0);
                    Eigen::Vector3d target = kick_target - uGBf * cfg.ball_approach_distance * angle_scale;
                    Hfk                    = pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, desired_heading));
                }

                // Issue walking task toward final position and orientation
                // Stop when at the target position
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk, true));
            });
    }

}  // namespace module::strategy
