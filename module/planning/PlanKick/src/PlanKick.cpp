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
#include "PlanKick.hpp"

#include <fmt/format.h>
#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::KickTo;
    using message::skill::Kick;
    using message::skill::Walk;
    using message::support::FieldDescription;
    using utility::support::Expression;

    PlanKick::PlanKick(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_timeout_threshold  = config["ball_timeout_threshold"].as<double>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<double>();
            cfg.ball_angle_threshold    = config["ball_angle_threshold"].as<double>();
            cfg.target_angle_threshold  = config["target_angle_threshold"].as<Expression>();
            cfg.kick_power_far          = config["kick_power_far"].as<double>();
            cfg.kick_power_mid          = config["kick_power_mid"].as<double>();
            cfg.kick_power_near         = config["kick_power_near"].as<double>();
        });

        // Shared with WalkToBall so both modules target the same point behind the goal line
        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            cfg.goal_target_offset = config["goal_target_offset"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Goal Position", [this](const FieldDescription& fd) {
            // Update the goal position
            rGFf        = Eigen::Vector3d(-fd.dimensions.field_length / 2 - cfg.goal_target_offset, 0, 0);
            field_length = fd.dimensions.field_length;
        });

        on<Provide<KickTo>, Uses<Kick>, Trigger<Ball>, With<Sensors>, With<Field>>().then(
            [this](const KickTo& kick_to,
                   const Uses<Kick>& kick,
                   const Ball& ball,
                   const Sensors& sensors,
                   const Field& field) {
                log<DEBUG>(fmt::format("PlanKick tick: run_state={} done={}",
                                       kick.run_state == RunState::RUNNING   ? "RUNNING"
                                       : kick.run_state == RunState::QUEUED  ? "QUEUED"
                                                                              : "NO_TASK",
                                       kick.done));
                // If the kick is running, don't interrupt or the robot may fall
                if (kick.run_state == RunState::RUNNING && !kick.done) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                // CHECK IF BALL IS BALL MEASUREMENT IS RECENT ENOUGH
                // If the ball measurement is old, then don't do anything
                auto time_difference = std::chrono::duration_cast<std::chrono::milliseconds>(
                    NUClear::clock::now() - ball.time_of_measurement);
                if (time_difference.count() >= cfg.ball_timeout_threshold) {
                    return;
                }

                // CHECK IF CLOSE TO BALL
                // Get the angle and distance to the ball
                const Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                double ball_angle          = std::abs(std::atan2(rBRr.y(), rBRr.x()));
                double ball_distance       = rBRr.head(2).norm();

                // Need to be near the ball to consider kicking it
                if (ball_distance > cfg.ball_distance_threshold || ball_angle > cfg.ball_angle_threshold) {
                    log<DEBUG>(fmt::format("Ball not close enough, distance {}m and angle {} radians.",
                                           ball_distance,
                                           ball_angle));
                    return;
                }

                // CHECK IF FACING POINT TO KICK TO
                double align_angle = std::abs(std::atan2(kick_to.rPRr.y(), kick_to.rPRr.x()));

                // Don't kick if we should align but we're not aligned to the target
                if (align_angle > cfg.target_angle_threshold) {
                    log<DEBUG>("Robot is not aligned to the kick target.");
                    return;
                }

                // If the kick conditions are not met, the function will have returned with no Tasks, ending the kick
                // Otherwise, the kick conditions are met and we need to check if we are already kicking
                // If we are already queued to kick, then only emit Idle to keep the previous Kick Task running
                if (kick.run_state == RunState::QUEUED) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                // COMPUTE KICK TARGET (field space) AND DIRECTION (robot-relative vector toward it)
                const Eigen::Isometry3d Hrf = sensors.Hrw * field.Hfw.inverse();
                // Transform the goal into robot space, then difference against the already robot-relative
                // ball position so the translation cancels correctly (both points are in the same frame)
                const Eigen::Vector3d rGRr           = Hrf * rGFf;
                const Eigen::Vector3d direction_unit = (rGRr - rBRr).normalized();

                // Determine power based on distance to target
                const double distance_to_target = rGRr.norm();
                const double power = distance_to_target > 2.0 * field_length / 3.0 ? cfg.kick_power_far
                                     : distance_to_target > field_length / 3.0     ? cfg.kick_power_mid
                                                                                    : cfg.kick_power_near;

                log<INFO>("KICK!");
                auto kick_task        = std::make_unique<Kick>();
                kick_task->target     = rGFf;
                kick_task->direction  = direction_unit * power;
                emit<Task>(std::move(kick_task));
            });
    }


}  // namespace module::planning
