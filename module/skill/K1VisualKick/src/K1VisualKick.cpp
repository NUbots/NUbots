/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "K1VisualKick.hpp"

#include <cmath>
#include <fmt/format.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/booster/BoosterKick.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterVisualKick.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/skill/Kick.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::booster::BoosterKick;
    using message::booster::BoosterMode;
    using message::booster::K1Mode;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::skill::Kick;
    using VisualKick = message::booster::BoosterVisualKick;
    using KickVer    = message::booster::VisualKickVer;

    K1VisualKick::K1VisualKick(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1VisualKick.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.version     = config["version"].as<int>() == 1 ? KickVer::V1 : KickVer::V2;
            cfg.kick_duration = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick_duration"].as<double>()));
            cfg.power_scale = config["power_scale"].as<double>();
            cfg.min_ball_move_distance = config["min_ball_move_distance"].as<double>();
            cfg.ball_reference_fraction = config["ball_reference_fraction"].as<double>();
        });

        // Every lets the task re-run to check whether kick_duration has elapsed
        on<Provide<Kick>, Every<10, Per<std::chrono::seconds>>, Trigger<Sensors>, With<Field>, With<Ball>>().then(
            [this](const Kick& kick,
                   const RunReason& run_reason,
                   const Sensors& sensors,
                   const Field& field,
                   const Ball& ball) {
                if (run_reason == RunReason::NEW_TASK) {
                    log<INFO>("Starting visual kick");
                    // Kicking requires movement, so request soccer mode from the Booster hardware.
                    auto mode  = std::make_unique<BoosterMode>();
                    mode->mode = K1Mode::SOCCER;
                    emit(std::move(mode));
                    // Starting the visual kick switches the robot into the mode that accepts kick
                    // references on rt/kick_ball, so the reference must be sent after the start.
                    auto vk     = std::make_unique<VisualKick>();
                    vk->start   = true;
                    vk->version = cfg.version;
                    emit(std::move(vk));

                    initial_ball_position = ball.rBWw;
                    const Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                    const Eigen::Isometry3d Hrf = Hfr.inverse();
                    const Eigen::Vector3d rBRr  = sensors.Hrw * ball.rBWw;
                    // Reference point a fraction of the way from the robot (origin in robot frame) to the ball
                    const Eigen::Vector3d rRefRr = rBRr * cfg.ball_reference_fraction;
                    const Eigen::Vector3d rGRr   = Hrf * kick.target;

                    auto ref                  = std::make_unique<BoosterKick>();
                    ref->x                    = rRefRr.x();
                    ref->y                    = rRefRr.y();
                    ref->dir                  = std::atan2(kick.direction.y(), kick.direction.x());
                    ref->power                = kick.direction.norm() * cfg.power_scale;
                    ref->goal_x               = rGRr.x();
                    ref->goal_y               = rGRr.y();
                    ref->robot_theta_to_field = std::atan2(Hfr.linear().col(0).y(), Hfr.linear().col(0).x());
                    emit(std::move(ref));

                    kick_start_time = NUClear::clock::now();
                }

                if (kick_start_time == NUClear::clock::time_point{}) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                double current_ball_distance = (ball.rBWw - initial_ball_position).norm();

                // Stop kick after set duration or ball moving past threshold distance
                if (NUClear::clock::now() - kick_start_time > cfg.kick_duration || current_ball_distance > cfg.min_ball_move_distance) {
                    log<INFO>(fmt::format(
                        "K1VisualKick done: elapsed={}s configured_duration={}s",
                        std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                    - kick_start_time)
                            .count(),
                        std::chrono::duration_cast<std::chrono::duration<double>>(cfg.kick_duration).count()));
                    emit<Task>(std::make_unique<Done>());
                }
                else {
                    emit<Task>(std::make_unique<Continue>());
                }
            });

        // Stop the SDK behaviour when the Kick task ends
        on<Stop<Kick>>().then([this] {
            log<INFO>("Stopping visual kick");
            auto vk     = std::make_unique<VisualKick>();
            vk->start   = false;
            vk->version = cfg.version;
            emit(std::move(vk));
            kick_start_time = NUClear::clock::time_point{};
        });
    }

}  // namespace module::skill
