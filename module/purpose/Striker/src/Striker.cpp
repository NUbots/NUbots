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
#include "Striker.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/Striker.hpp"
#include "message/purpose/UpdateBoundingBox.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase          = message::input::GameState::Data::Phase;
    using GameMode       = message::input::GameState::Data::Mode;
    using SecondaryState = message::input::GameState::Data::SecondaryState;
    using message::input::GameState;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::KickTo;
    using message::purpose::CornerKickStriker;
    using message::purpose::DirectFreeKickStriker;
    using message::purpose::GoalKickStriker;
    using message::purpose::InDirectFreeKickStriker;
    using message::purpose::NormalStriker;
    using message::purpose::PenaltyKickStriker;
    using message::purpose::PenaltyShootoutStriker;
    using message::purpose::ThrowInStriker;
    using message::purpose::UpdateBoundingBox;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkInsideBoundedBox;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::WalkToKickBall;

    using StrikerTask = message::purpose::Striker;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    Striker::Striker(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Striker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Striker.yaml
            this->log_level              = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position           = config["ready_position"].as<Expression>();
            cfg.penalty_defence_position = config["penalty_defence_position"].as<Expression>();
            cfg.Hfr = pos_rpy_to_transform(Eigen::Vector3d(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                                           Eigen::Vector3d(0, 0, cfg.ready_position.z()));
            cfg.ball_kickoff_outside_radius = config["ball_kickoff_outside_radius"].as<double>();

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

        on<Provide<StrikerTask>, Optional<Trigger<GameState>>>().then(
            [this](const StrikerTask& striker_task, const std::shared_ptr<const GameState>& game_state) {
                // Do not use GameController information if force playing or force penalty shootout
                if (striker_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->data.mode.value) {
                        case GameMode::PENALTY_SHOOTOUT: emit<Task>(std::make_unique<PenaltyShootoutStriker>()); break;
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalStriker>()); break;
                        case GameMode::DIRECT_FREEKICK: emit<Task>(std::make_unique<DirectFreeKickStriker>()); break;
                        case GameMode::INDIRECT_FREEKICK:
                            emit<Task>(std::make_unique<InDirectFreeKickStriker>());
                            break;
                        case GameMode::PENALTYKICK: emit<Task>(std::make_unique<PenaltyKickStriker>()); break;
                        case GameMode::CORNER_KICK: emit<Task>(std::make_unique<CornerKickStriker>()); break;
                        case GameMode::GOAL_KICK: emit<Task>(std::make_unique<GoalKickStriker>()); break;
                        case GameMode::THROW_IN: emit<Task>(std::make_unique<ThrowInStriker>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        // Normal READY state
        on<Provide<NormalStriker>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // If we are stable, walk to the ready field position
            emit<Task>(std::make_unique<WalkToFieldPosition>(cfg.Hfr, true));
        });

        // Normal PLAYING state
        on<Provide<NormalStriker>,
           When<Phase, std::equal_to, Phase::PLAYING>,
           With<GameState>,
           Optional<With<Ball>>,
           Optional<With<Field>>>()
            .then([this](const GameState& game_state,
                         const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Field>& field) {
                // If it's not our kickoff and timer is going, stand still
                // The secondary timer will only happen for kickoff here
                if (!game_state.data.our_kick_off
                    && (game_state.data.secondary_time - NUClear::clock::now()).count() > 0) {
                    // Check if the ball has moved, if so start playing
                    if (ball != nullptr && field != nullptr
                        && (field->Hfw * ball->rBWw).norm() > cfg.ball_kickoff_outside_radius) {
                        play();
                        return;
                    }
                    // Walk to ready so we are ready to play when kickoff finishes
                    emit<Task>(std::make_unique<WalkToFieldPosition>(cfg.Hfr, true));
                    return;
                }
                play();
            });

        // Normal UNKNOWN state
        on<Provide<NormalStriker>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty shootout PLAYING state
        on<Provide<PenaltyShootoutStriker>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Penalty shootout UNKNOWN state
        on<Provide<PenaltyShootoutStriker>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown penalty shootout game phase."); });

        // Default for INITIAL, READY, SET, FINISHED, TIMEOUT
        on<Provide<PenaltyShootoutStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Direct free kick
        on<Provide<DirectFreeKickStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Direct free kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Indirect free kick
        on<Provide<InDirectFreeKickStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Indirect free kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Penalty kick
        on<Provide<PenaltyKickStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Penalty kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, move to the penalty defence position (or bounded box edge) and look
                // at the ball
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(
                                   Eigen::Vector3d(
                                       std::min(cfg.bounded_region_x_max, (double) cfg.penalty_defence_position.x()),
                                       cfg.penalty_defence_position.y(),
                                       cfg.penalty_defence_position.z()),
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
        on<Provide<CornerKickStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Corner kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Goal kick
        on<Provide<GoalKickStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Goal kick sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });

        // Throw in
        on<Provide<ThrowInStriker>, When<Phase, std::equal_to, Phase::PLAYING>, With<GameState>>().then(
            [this](const GameState& game_state) {
                // Throw in sub mode: 0 for ready, 1 for freeze/ball repositioning by referee
                if (game_state.data.secondary_state.sub_mode) {
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }
                // If the performing team is not us, find the ball and look at it
                if (int(game_state.data.secondary_state.team_performing) != int(game_state.data.team.team_id)) {
                    emit<Task>(std::make_unique<FindBall>(), 1);
                    emit<Task>(std::make_unique<LookAtBall>(), 2);
                    return;
                }
                // Otherwise, play as normal
                play();
            });
    }

    void Striker::play() {
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
