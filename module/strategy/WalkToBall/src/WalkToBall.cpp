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
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::WalkDirect;
    using message::planning::WalkTo;
    using message::strategy::WalkToFieldPosition;
    using WalkToBallTask     = message::strategy::WalkToBall;
    using WalkToKickBallTask = message::strategy::WalkToKickBall;
    using FieldDescription   = message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    WalkToBall::WalkToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToBall.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.ball_y_offset        = config["ball_y_offset"].as<double>();
            cfg.ball_kick_distance   = config["ball_kick_distance"].as<double>();
            cfg.goal_target_offset   = config["goal_target_offset"].as<double>();
            cfg.approach_ball_radius = config["approach_ball_radius"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Goal Position", [this](const FieldDescription& fd) {
            // Update the goal position
            rGFf = Eigen::Vector3d(-fd.dimensions.field_length / 2 - cfg.goal_target_offset, 0, 0);
        });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToBallTask>, With<Ball>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const Ball& ball, const Sensors& sensors) {
                // If we have a ball, walk to it
                if (NUClear::clock::now() - ball.time_of_measurement < cfg.ball_search_timeout) {
                    Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                    // Add an offset to account for walking with the foot in front of the ball
                    rBRr.y() += cfg.ball_y_offset;
                    const double heading = std::atan2(rBRr.y(), rBRr.x());
                    auto Hrb             = pos_rpy_to_transform(rBRr, Eigen::Vector3d(0, 0, heading));
                    emit<Task>(std::make_unique<WalkDirect>(Hrb, cfg.approach_ball_radius));
                }
            });

        // If the Provider updates on Every and the last Ball was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToKickBallTask>, With<Ball>, With<Sensors>, With<Field>, Every<30, Per<std::chrono::seconds>>>()
            .then([this](const Ball& ball, const Sensors& sensors, const Field& field) {
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

                    Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();

                    // Get position of robot on field
                    Eigen::Vector3d rRFf = Hfr.translation();
                    Eigen::Vector3d uRFf = Hfr.linear().col(0);

                    // Compute the heading (angle between the x-axis and the vector from the kick position to the goal)
                    double heading = std::atan2(rGBf.y(), rGBf.x());

                    // Compute the angle between the robot x-axis and the desired kick direction
                    double angle_error = heading - std::atan2(uRFf.y(), uRFf.x());

                    // Compute position to kick
                    Eigen::Vector3d rKFf = Eigen::Vector3d::Zero();

                    // If the ball is closer to the goal than the robot and robot is not facing the goal, walk to point
                    // behind the ball
                    if (rBFf.x() > rRFf.x() || std::abs(angle_error) > M_PI_2) {
                        log<NUClear::INFO>("Walking to point behind ball");
                        rKFf = rBFf - uGBf * cfg.ball_kick_distance;
                    }
                    else {
                        log<NUClear::INFO>("Walking to ball directly");
                        // Walk to the ball
                        rKFf = rBFf;
                    }
                    log<NUClear::INFO>("Angle error: ", angle_error);

                    auto Hfk = pos_rpy_to_transform(rKFf, Eigen::Vector3d(0, 0, heading));

                    emit<Task>(std::make_unique<WalkToFieldPosition>(Hfk));
                }
            });
    }

}  // namespace module::strategy
