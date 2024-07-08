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
#include "AllRounder.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/AllRounder.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase    = message::input::GameState::Data::Phase;
    using GameMode = message::input::GameState::Data::Mode;
    using message::input::GameState;
    using message::planning::KickTo;
    using message::purpose::CornerKickAllRounder;
    using message::purpose::DirectFreeKickAllRounder;
    using message::purpose::GoalKickAllRounder;
    using message::purpose::InDirectFreeKickAllRounder;
    using message::purpose::NormalAllRounder;
    using message::purpose::PenaltyKickAllRounder;
    using message::purpose::PenaltyShootoutAllRounder;
    using message::purpose::ThrowInAllRounder;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::WalkToKickBall;

    using AllRounderTask = message::purpose::AllRounder;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    AllRounder::AllRounder(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("AllRounder.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AllRounder.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position = config["ready_position"].as<Expression>();
        });

        on<Provide<AllRounderTask>, Optional<Trigger<GameState>>>().then(
            [this](const AllRounderTask& AllRounder_task, const std::shared_ptr<const GameState>& game_state) {
                // Do not use GameController information if force playing or force penalty shootout
                if (AllRounder_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->data.mode.value) {
                        case GameMode::PENALTY_SHOOTOUT:
                            emit<Task>(std::make_unique<PenaltyShootoutAllRounder>());
                            break;
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalAllRounder>()); break;
                        case GameMode::DIRECT_FREEKICK: emit<Task>(std::make_unique<DirectFreeKickAllRounder>()); break;
                        case GameMode::INDIRECT_FREEKICK:
                            emit<Task>(std::make_unique<InDirectFreeKickAllRounder>());
                            break;
                        case GameMode::PENALTYKICK: emit<Task>(std::make_unique<PenaltyKickAllRounder>()); break;
                        case GameMode::CORNER_KICK: emit<Task>(std::make_unique<CornerKickAllRounder>()); break;
                        case GameMode::GOAL_KICK: emit<Task>(std::make_unique<GoalKickAllRounder>()); break;
                        case GameMode::THROW_IN: emit<Task>(std::make_unique<ThrowInAllRounder>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        // Normal READY state
        on<Provide<NormalAllRounder>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // If we are stable, walk to the ready field position
            emit<Task>(std::make_unique<WalkToFieldPosition>(
                pos_rpy_to_transform(Eigen::Vector3d(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                                     Eigen::Vector3d(0, 0, cfg.ready_position.z()))));
        });

        // Normal PLAYING state
        on<Provide<NormalAllRounder>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<NormalAllRounder>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty shootout PLAYING state
        on<Provide<PenaltyShootoutAllRounder>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Penalty shootout UNKNOWN state
        on<Provide<PenaltyShootoutAllRounder>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown penalty shootout game phase."); });

        // Default for INITIAL, READY, SET, FINISHED, TIMEOUT
        on<Provide<PenaltyShootoutAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Direct free kick
        on<Provide<DirectFreeKickAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Indirect free kick
        on<Provide<InDirectFreeKickAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty kick
        on<Provide<PenaltyKickAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Corner kick
        on<Provide<CornerKickAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Goal kick
        on<Provide<GoalKickAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Throw in
        on<Provide<ThrowInAllRounder>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });
    }

    void AllRounder::play() {
        // Walk to the ball and kick!
        // Second argument is priority - higher number means higher priority
        emit<Task>(std::make_unique<FindBall>(), 1);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 2);  // try to track the ball
        emit<Task>(std::make_unique<WalkToKickBall>(), 3);  // try to walk to the ball and align towards opponents goal
        emit<Task>(std::make_unique<KickToGoal>(), 4);      // kick the ball if possible
    }

}  // namespace module::purpose
