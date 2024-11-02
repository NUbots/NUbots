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

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/planning/KickTo.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::planning::KickTo;
    using message::skill::Kick;
    using message::skill::Walk;
    using utility::input::LimbID;
    using utility::support::Expression;

    PlanKick::PlanKick(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_timeout_threshold  = config["ball_timeout_threshold"].as<double>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<double>();
            cfg.ball_angle_threshold    = config["ball_angle_threshold"].as<double>();
            cfg.target_angle_threshold  = config["target_angle_threshold"].as<Expression>();
            cfg.kick_leg                = config["kick_leg"].as<std::string>();
        });

        on<Provide<KickTo>, Uses<Kick>, Trigger<Ball>, Trigger<Stability>, With<Sensors>>().then(
            [this](const KickTo& kick_to,
                   const Uses<Kick>& kick,
                   const Ball& ball,
                   const Stability& stability,
                   const Sensors& sensors) {
                // If the kick is running, don't interrupt or the robot may fall
                if (kick.run_state == RunState::RUNNING && !kick.done) {
                    emit<Task>(std::make_unique<Idle>());
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
                    log<NUClear::DEBUG>(fmt::format("Ball not close enough, distance {}m and angle {} radians.",
                                                    ball_distance,
                                                    ball_angle));
                    return;
                }

                // CHECK IF FACING POINT TO KICK TO
                double align_angle = std::abs(std::atan2(kick_to.rPRr.y(), kick_to.rPRr.x()));

                // Don't kick if we should align but we're not aligned to the target
                if (align_angle > cfg.target_angle_threshold) {
                    log<NUClear::DEBUG>("Robot is not aligned to the kick target.");
                    return;
                }

                // If the kick conditions are not met, the function will have returned with no Tasks, ending the kick
                // Otherwise, the kick conditions are met and we need to check if we are already kicking
                // If we are already queued to kick, then only emit Idle to keep the previous Kick Task running
                if (kick.run_state == RunState::QUEUED) {
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // If the robot is not standing, make it stand before kicking
                if (stability != Stability::STANDING) {
                    log<NUClear::DEBUG>("Standing to kick!");
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                    return;
                }

                // If the kick leg is forced left, kick left. If the kick leg is auto,
                // kick with left leg if ball is more to the left
                if (cfg.kick_leg == LimbID::LEFT_LEG || (cfg.kick_leg == LimbID::UNKNOWN && rBRr.y() > 0.0)) {
                    log<NUClear::INFO>("LEFT KICK!");
                    emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG));
                }
                else {  // kick leg is forced right or ball is more to the right and kick leg is auto
                    log<NUClear::INFO>("RIGHT KICK!");
                    emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG));
                }
            });
    }


}  // namespace module::planning
