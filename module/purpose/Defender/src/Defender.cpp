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
#include "Defender.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/UpdateBoundingBox.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase    = message::input::GameState::Phase;
    using GameMode = message::input::GameState::Mode;
    using message::input::GameState;
    using message::purpose::CornerKickDefender;
    using message::purpose::DirectFreeKickDefender;
    using message::purpose::GoalKickDefender;
    using message::purpose::InDirectFreeKickDefender;
    using message::purpose::NormalDefender;
    using message::purpose::PenaltyKickDefender;
    using message::purpose::ThrowInDefender;
    using message::purpose::UpdateBoundingBox;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::StandStill;
    using message::strategy::WalkInsideBoundedBox;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::WalkToKickBall;

    using DefenderTask = message::purpose::Defender;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    Defender::Defender(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {
        on<Configuration>("Defender.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defender.yaml
            this->log_level              = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position           = config["ready_position"].as<Expression>();
            cfg.penalty_defence_position = config["penalty_defence_position"].as<Expression>();
            cfg.Hfr = pos_rpy_to_transform(Eigen::Vector3d(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                                           Eigen::Vector3d(0, 0, cfg.ready_position.z()));

            cfg.bounded_region_x_min = config["bounded_region_x_min"].as<Expression>();
            cfg.bounded_region_x_max = config["bounded_region_x_max"].as<Expression>();
            cfg.bounded_region_y_min = config["bounded_region_y_min"].as<Expression>();
            cfg.bounded_region_y_max = config["bounded_region_y_max"].as<Expression>();
        });

        on<Trigger<UpdateBoundingBox>>().then([this](const UpdateBoundingBox& new_bounding_box) {
            cfg.bounded_region_x_min = new_bounding_box.x_min;
            cfg.bounded_region_x_max = new_bounding_box.x_max;
            cfg.bounded_region_y_min = new_bounding_box.y_min;
            cfg.bounded_region_y_max = new_bounding_box.y_max;
            // Debugging
            emit(std::make_unique<WalkInsideBoundedBox>(cfg.bounded_region_x_min,
                                                        cfg.bounded_region_x_max,
                                                        cfg.bounded_region_y_min,
                                                        cfg.bounded_region_y_max,
                                                        cfg.Hfr));
        });

        on<Provide<DefenderTask>, Optional<Trigger<GameState>>>().then(
            [this](const DefenderTask& defender_task, const std::shared_ptr<const GameState>& game_state) {
                log<DEBUG>("DEFENDER");
                // Do not use GameController information if force playing or force penalty shootout
                if (defender_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->mode.value) {
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalDefender>()); break;
                        case GameMode::DIRECT_FREEKICK: emit<Task>(std::make_unique<DirectFreeKickDefender>()); break;
                        case GameMode::INDIRECT_FREEKICK:
                            emit<Task>(std::make_unique<InDirectFreeKickDefender>());
                            break;
                        case GameMode::PENALTYKICK: emit<Task>(std::make_unique<PenaltyKickDefender>()); break;
                        case GameMode::CORNER_KICK: emit<Task>(std::make_unique<CornerKickDefender>()); break;
                        case GameMode::GOAL_KICK: emit<Task>(std::make_unique<GoalKickDefender>()); break;
                        case GameMode::THROW_IN: emit<Task>(std::make_unique<ThrowInDefender>()); break;
                        default: log<WARN>("Game mode unknown.");
                    }
                }
            });


        // Normal READY state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // If we are stable, walk to the ready field position
            log<DEBUG>("READY");
            emit<Task>(std::make_unique<WalkToFieldPosition>(cfg.Hfr, true));
        });

        // Normal PLAYING state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalDefender>>().then([this] {
            log<DEBUG>("INITIAL");
            emit<Task>(std::make_unique<StandStill>());
        });

        // Direct free kick
        on<Provide<DirectFreeKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Direct free kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Indirect free kick
        on<Provide<InDirectFreeKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Indirect free kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Penalty kick
        on<Provide<PenaltyKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Penalty kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, move to our penalty defence position and look at the ball
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(
                        std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(
                            Eigen::Vector3d(cfg.penalty_defence_position.x(), cfg.penalty_defence_position.y(), 0),
                            Eigen::Vector3d(0, 0, cfg.penalty_defence_position.z()))),
                        1);
                    emit<Task>(std::make_unique<FindBall>(), 2);
                    emit<Task>(std::make_unique<LookAtBall>(), 3);
                    return;
                }
                // Otherwise, play as normal
                play();
            });


        // Corner kick
        on<Provide<CornerKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Corner kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Goal kick
        on<Provide<GoalKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Goal kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Throw in
        on<Provide<DirectFreeKickDefender>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Throw in sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (game_state.secondary_state.team_performing != game_state.team.team_id) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });
    }

    void Defender::play() {
        // Walk to the ball and kick!
        // Second argument is priority - higher number means higher priority
        emit<Task>(std::make_unique<FindBall>(), 1);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 2);  // try to track the ball
        emit<Task>(std::make_unique<WalkToKickBall>(), 3);  // try to walk to the ball and align towards opponents goal
        emit<Task>(std::make_unique<WalkInsideBoundedBox>(cfg.bounded_region_x_min,
                                                          cfg.bounded_region_x_max,
                                                          cfg.bounded_region_y_min,
                                                          cfg.bounded_region_y_max,
                                                          cfg.Hfr),
                   4);  // Patrol bounded box region
    }

}  // namespace module::purpose
