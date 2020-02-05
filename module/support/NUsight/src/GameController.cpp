/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUsight.h"

namespace module {
namespace support {
    using TeamColour     = message::input::GameEvents::TeamColour;
    using Score          = message::input::GameEvents::Score;
    using GoalScored     = message::input::GameEvents::GoalScored;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using CoachMessage   = message::input::GameEvents::CoachMessage;
    using HalfTime       = message::input::GameEvents::HalfTime;
    using BallKickedOut  = message::input::GameEvents::BallKickedOut;
    using KickOffTeam    = message::input::GameEvents::KickOffTeam;
    using GamePhase      = message::input::GameEvents::GamePhase;
    using GameMode       = message::input::GameEvents::GameMode;
    using GameState      = message::input::GameState;
    using GameStateData  = message::input::GameState::Data;

    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    void NUsight::provideGameController() {

        handles["game_state"].push_back(on<Trigger<TeamColour>, With<GameState>>().then(
            [this](std::shared_ptr<const GameState> gameState) { sendGameState("TeamColour", gameState); }));

        handles["game_state"].push_back(on<Trigger<Score>, With<GameState>>().then(
            [this](std::shared_ptr<const GameState> gameState) { sendGameState("Score", gameState); }));

        handles["game_state"].push_back(on<Trigger<GoalScored>, With<GameState>>().then(
            [this](const GoalScored& goalScored, std::shared_ptr<const GameState> gameState) {
                switch (goalScored.context.value) {
                    case message::input::GameEvents::Context::Value::TEAM: {
                        sendGameState("GoalScored<TEAM>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::OPPONENT: {
                        sendGameState("GoalScored<OPPONENT>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown goal score context:", goalScored.context);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<Penalisation>, With<GameState>>().then(
            [this](const Penalisation& penalisation, std::shared_ptr<const GameState> gameState) {
                switch (penalisation.context.value) {
                    case message::input::GameEvents::Context::Value::SELF: {
                        sendGameState("Penalisation<SELF>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::TEAM: {
                        sendGameState("Penalisation<TEAM>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::OPPONENT: {
                        sendGameState("Penalisation<OPPONENT>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown penalisation context:", penalisation.context);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<Unpenalisation>, With<GameState>>().then(
            [this](const Unpenalisation& unpenalisation, std::shared_ptr<const GameState> gameState) {
                switch (unpenalisation.context.value) {
                    case message::input::GameEvents::Context::Value::SELF: {
                        sendGameState("Unpenalisation<SELF>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::TEAM: {
                        sendGameState("Unpenalisation<TEAM>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::OPPONENT: {
                        sendGameState("Unpenalisation<OPPONENT>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown unpenalisation context:", unpenalisation.context);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<CoachMessage>, With<GameState>>().then(
            [this](const CoachMessage& message, std::shared_ptr<const GameState> gameState) {
                switch (message.context.value) {
                    case message::input::GameEvents::Context::Value::TEAM: {
                        sendGameState("CoachMessage<TEAM>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::OPPONENT: {
                        sendGameState("CoachMessage<OPPONENT>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown coach message context:", message.context);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<HalfTime>, With<GameState>>().then(
            [this](std::shared_ptr<const GameState> gameState) { sendGameState("HalfTime", gameState); }));

        handles["game_state"].push_back(on<Trigger<BallKickedOut>, With<GameState>>().then(
            [this](const BallKickedOut& ballKickedOut, std::shared_ptr<const GameState> gameState) {
                switch (ballKickedOut.context.value) {
                    case message::input::GameEvents::Context::Value::TEAM: {
                        sendGameState("BallKickedOut<TEAM>", gameState);
                        break;
                    }

                    case message::input::GameEvents::Context::Value::OPPONENT: {
                        sendGameState("BallKickedOut<OPPONENT>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown ball kicked out context:", ballKickedOut.context);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<KickOffTeam>, With<GameState>>().then(
            [this](std::shared_ptr<const GameState> gameState) { sendGameState("KickOffTeam", gameState); }));

        handles["game_state"].push_back(on<Trigger<GamePhase>, With<GameState>>().then(
            [this](const GamePhase& gamePhase, std::shared_ptr<const GameState> gameState) {
                switch (gamePhase.phase.value) {
                    case GameStateData::Phase::Value::INITIAL: {
                        sendGameState("GamePhase<INITIAL>", gameState);
                        break;
                    }

                    case GameStateData::Phase::Value::READY: {
                        sendGameState("GamePhase<READY>", gameState);
                        break;
                    }

                    case GameStateData::Phase::Value::SET: {
                        sendGameState("GamePhase<SET>", gameState);
                        break;
                    }

                    case GameStateData::Phase::Value::PLAYING: {
                        sendGameState("GamePhase<PLAYING>", gameState);
                        break;
                    }

                    case GameStateData::Phase::Value::TIMEOUT: {
                        sendGameState("GamePhase<TIMEOUT>", gameState);
                        break;
                    }

                    case GameStateData::Phase::Value::FINISHED: {
                        sendGameState("GamePhase<FINISHED>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown game phase:", gamePhase.phase);
                        break;
                    }
                }
            }));

        handles["game_state"].push_back(on<Trigger<GameMode>, With<GameState>>().then(
            [this](const GameMode& gameMode, std::shared_ptr<const GameState> gameState) {
                switch (gameMode.mode.value) {
                    case GameStateData::Mode::Value::NORMAL: {
                        sendGameState("GameMode<NORMAL>", gameState);
                        break;
                    }

                    case GameStateData::Mode::Value::PENALTY_SHOOTOUT: {
                        sendGameState("GameMode<PENALTY_SHOOTOUT>", gameState);
                        break;
                    }

                    case GameStateData::Mode::Value::OVERTIME: {
                        sendGameState("GameMode<OVERTIME>", gameState);
                        break;
                    }

                    default: {
                        log("Unknown game mode:", gameMode.mode);
                        break;
                    }
                }
            }));
    }

    void NUsight::sendGameState(std::string event, std::shared_ptr<const GameState> gameState) {
        log("GameEvent:", event);

        powerplant.emit_shared<Scope::NETWORK>(std::move(gameState), "nusight", true);
    }
}  // namespace support
}  // namespace module
