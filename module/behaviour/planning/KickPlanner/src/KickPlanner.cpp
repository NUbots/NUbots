/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include "KickPlanner.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/KickPlan.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/LimbID.hpp"


namespace module::behaviour::planning {

    using extension::Configuration;

    using message::behaviour::KickPlan;
    using message::behaviour::WantsToKick;
    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::motion::KickCommand;
    using message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using message::platform::ButtonMiddleDown;
    using message::support::FieldDescription;

    using KickType      = message::behaviour::KickPlan::KickType;
    using PenaltyReason = message::input::GameState::Data::PenaltyReason;
    using Phase         = message::input::GameState::Data::Phase;
    using VisionBalls   = message::vision::Balls;

    using utility::input::LimbID;

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("KickPlanner.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.max_ball_distance        = config["max_ball_distance"].as<float>();
            cfg.kick_corridor_width      = config["kick_corridor_width"].as<float>();
            cfg.seconds_not_seen_limit   = config["seconds_not_seen_limit"].as<float>();
            cfg.kick_forward_angle_limit = config["kick_forward_angle_limit"].as<float>();
            emit(std::make_unique<WantsToKick>(false));
        });

        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                ball_last_seen = NUClear::clock::now();
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] { force_playing = true; });

        on<Trigger<Ball>,
           With<Field>,
           With<FieldDescription>,
           With<KickPlan>,
           With<Sensors>,
           Optional<With<GameState>>>()
            .then([this](const Ball& ball,
                         const Field& field,
                         const FieldDescription& fd,
                         const KickPlan& kick_plan,
                         const Sensors& sensors,
                         const std::shared_ptr<const GameState>& game_state) {
                // Get time since last seen ball
                auto now = NUClear::clock::now();
                double seconds_since_last_seen =
                    std::chrono::duration_cast<std::chrono::microseconds>(now - ball_last_seen).count() * 1e-6;

                // Compute target in robot coords
                Eigen::Isometry3f Hfw(field.Hfw);
                Eigen::Isometry3f Htw(sensors.Htw);
                Eigen::Vector3f ball_position =
                    Htw * Eigen::Vector3f(ball.position.x(), ball.position.y(), fd.ball_radius);

                // Transform target from field to torso space
                Eigen::Isometry3f Htf       = Htw * Hfw.inverse();
                Eigen::Vector3f kick_target = Htf * Eigen::Vector3f(kick_plan.target.x(), kick_plan.target.y(), 0.0f);
                float kick_angle            = std::fabs(std::atan2(kick_target.y(), kick_target.x()));

                bool correct_state = true;
                if (game_state) {
                    // If we are playing with GameController, only kick if we are in the playing state, are not
                    // penalised and are not in ready state
                    correct_state = game_state->data.phase == Phase::PLAYING
                                    && game_state->data.self.penalty_reason == PenaltyReason::UNPENALISED
                                    && game_state->data.phase != Phase::READY;
                }
                bool kick_is_valid = kick_valid(ball_position);
                if (kick_is_valid) {
                    last_time_valid = now;
                }
                float time_since_valid = (now - last_time_valid).count() * (1 / double(NUClear::clock::period::den));

                if (seconds_since_last_seen < cfg.seconds_not_seen_limit && kick_is_valid
                    && (correct_state || force_playing) && kick_angle < cfg.kick_forward_angle_limit) {

                    switch (kick_plan.kick_type.value) {
                        case KickType::IK_KICK:
                            if (ball_position.y() > 0.0f) {
                                emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3f(0.1f, 0.04f, 0.0f),
                                                                               Eigen::Vector3f::UnitX(),
                                                                               KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                            }
                            else {
                                emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3f(0.1f, -0.04f, 0.0f),
                                                                               Eigen::Vector3f::UnitX(),
                                                                               KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                            }
                            break;
                        case KickType::SCRIPTED:
                            if (ball_position.y() > 0.0f) {
                                emit(std::make_unique<KickScriptCommand>(
                                    KickScriptCommand(LimbID::LEFT_LEG, KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                                ;
                            }
                            else {
                                emit(std::make_unique<KickScriptCommand>(
                                    KickScriptCommand(LimbID::RIGHT_LEG, KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                                ;
                            }
                            break;
                        default: throw std::runtime_error("KickPlanner: Invalid KickType");
                    }
                }
                else if (seconds_since_last_seen > cfg.seconds_not_seen_limit
                         || time_since_valid > cfg.seconds_not_seen_limit) {
                    emit(std::make_unique<WantsToKick>(WantsToKick(false)));
                }
            });
    }


    [[nodiscard]] bool KickPlanner::kick_valid(const Eigen::Vector3f& ball_pos) const {
        return (ball_pos.x() > 0.0f) && (ball_pos.x() < cfg.max_ball_distance)
               && (std::fabs(ball_pos.y()) < cfg.kick_corridor_width * 0.5f);
    }
}  // namespace module::behaviour::planning
