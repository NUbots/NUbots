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
#include "Goalie.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/strategy/DiveToBall.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {
    using message::input::GameState;
    using Phase    = message::input::GameState::Data::Phase;
    using GameMode = message::input::GameState::Data::Mode;
    using message::strategy::DiveToBall;
    using message::strategy::LookAtBall;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using GoalieTask = message::purpose::Goalie;
    using message::purpose::CornerKickGoalie;
    using message::purpose::DirectFreeKickGoalie;
    using message::purpose::GoalKickGoalie;
    using message::purpose::InDirectFreeKickGoalie;
    using message::purpose::NormalGoalie;
    using message::purpose::PenaltyKickGoalie;
    using message::purpose::PenaltyShootoutGoalie;
    using message::purpose::ThrowInGoalie;
    using message::strategy::FindBall;
    using message::strategy::WalkInsideBoundedBox;
    using message::strategy::WalkToFieldPosition;

    using extension::Configuration;

    using utility::support::Expression;

    Goalie::Goalie(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Goalie.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Goalie.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position = config["ready_position"].as<Expression>();
        });

        on<Provide<GoalieTask>, Optional<Trigger<GameState>>>().then(
            [this](const GoalieTask& goalie_task, const std::shared_ptr<const GameState>& game_state) {
                // Do not use GameController information if force playing or force penalty shootout
                if (goalie_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->data.mode.value) {
                        case GameMode::PENALTY_SHOOTOUT: emit<Task>(std::make_unique<PenaltyShootoutGoalie>()); break;
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalGoalie>()); break;
                        case GameMode::DIRECT_FREEKICK: emit<Task>(std::make_unique<DirectFreeKickGoalie>()); break;
                        case GameMode::INDIRECT_FREEKICK: emit<Task>(std::make_unique<InDirectFreeKickGoalie>()); break;
                        case GameMode::PENALTYKICK: emit<Task>(std::make_unique<PenaltyKickGoalie>()); break;
                        case GameMode::CORNER_KICK: emit<Task>(std::make_unique<CornerKickGoalie>()); break;
                        case GameMode::GOAL_KICK: emit<Task>(std::make_unique<GoalKickGoalie>()); break;
                        case GameMode::THROW_IN: emit<Task>(std::make_unique<ThrowInGoalie>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        // Normal READY state
        on<Provide<NormalGoalie>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            emit<Task>(std::make_unique<WalkToFieldPosition>(
                Eigen::Vector3f(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                cfg.ready_position.z()));
        });

        // Normal PLAYING state
        on<Provide<NormalGoalie>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<NormalGoalie>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty shootout PLAYING state
        on<Provide<PenaltyShootoutGoalie>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Penalty shootout UNKNOWN state
        on<Provide<PenaltyShootoutGoalie>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown penalty shootout game phase."); });

        // Default for INITIAL, READY, SET, FINISHED, TIMEOUT
        on<Provide<PenaltyShootoutGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Direct free kick
        on<Provide<DirectFreeKickGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Indirect free kick
        on<Provide<InDirectFreeKickGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty kick
        on<Provide<PenaltyKickGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Corner kick
        on<Provide<CornerKickGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Goal kick
        on<Provide<GoalKickGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Throw in
        on<Provide<ThrowInGoalie>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });
    }

    void Goalie::play() {
        // Stop the ball!
        // Second argument is priority - higher number means higher priority
        Eigen::Vector3f position(cfg.ready_position.x(), cfg.ready_position.y(), 0);
        emit<Task>(std::make_unique<WalkToFieldPosition>(position), cfg.ready_position.z(), 1);
        emit<Task>(std::make_unique<FindBall>(), 2);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 3);  // try to track the ball
        emit<Task>(std::make_unique<WalkToBall>(), 4);  // try to walk to the ball
        emit<Task>(std::make_unique<WalkInsideBoundedBox>(), 5);  // Patrol bounded box region
    }

}  // namespace module::purpose
