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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using TeamColour     = message::input::proto::GameEvents::TeamColour;
    using Score          = message::input::proto::GameEvents::Score;
    using GoalScored     = message::input::proto::GameEvents::GoalScored;
    using Penalisation   = message::input::proto::GameEvents::Penalisation;
    using Unpenalisation = message::input::proto::GameEvents::Unpenalisation;
    using CoachMessage   = message::input::proto::GameEvents::CoachMessage;
    using HalfTime       = message::input::proto::GameEvents::HalfTime;
    using BallKickedOut  = message::input::proto::GameEvents::BallKickedOut;
    using KickOffTeam    = message::input::proto::GameEvents::KickOffTeam;
    using GamePhase      = message::input::proto::GameEvents::GamePhase;
    using GameMode       = message::input::proto::GameEvents::GameMode;
    using GameState      = message::input::proto::GameState;
    using GameStateData  = message::input::proto::GameState::Data;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    void NUbugger::provideGameController() {

        // HALP X_X

        handles["game_state"].push_back(on<Trigger<TeamColour>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("TeamColour", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Score>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Score", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GoalScored>, With<GameState>>().then([this](const GoalScored& goalScored, const GameState& gameState) {
            switch (goalScored.context.value)
            {
                case message::input::proto::GameEvents::Context::Value::TEAM:
                {
                    sendGameState("GoalScored<TEAM>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::OPPONENT:
                {
                    sendGameState("GoalScored<OPPONENT>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown goal score context:", std::string(goalScored.context));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<Penalisation>, With<GameState>>().then([this](const Penalisation& penalisation, const GameState& gameState) {
            switch (penalisation.context.value)
            {
                case message::input::proto::GameEvents::Context::Value::SELF:
                {
                    sendGameState("Penalisation<SELF>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::TEAM:
                {
                    sendGameState("Penalisation<TEAM>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::OPPONENT:
                {
                    sendGameState("Penalisation<OPPONENT>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown penalisation context:", std::string(penalisation.context));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<Unpenalisation>, With<GameState>>().then([this](const Unpenalisation& unpenalisation, const GameState& gameState) {
            switch (unpenalisation.context.value)
            {
                case message::input::proto::GameEvents::Context::Value::SELF:
                {
                    sendGameState("Unpenalisation<SELF>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::TEAM:
                {
                    sendGameState("Unpenalisation<TEAM>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::OPPONENT:
                {
                    sendGameState("Unpenalisation<OPPONENT>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown unpenalisation context:", std::string(unpenalisation.context));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<CoachMessage>, With<GameState>>().then([this](const CoachMessage& message, const GameState& gameState) {
            switch (message.context.value)
            {
                case message::input::proto::GameEvents::Context::Value::TEAM:
                {
                    sendGameState("CoachMessage<TEAM>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::OPPONENT:
                {
                    sendGameState("CoachMessage<OPPONENT>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown coach message context:", std::string(message.context));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<HalfTime>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("HalfTime", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<BallKickedOut>, With<GameState>>().then([this](const BallKickedOut& ballKickedOut, const GameState& gameState) {
            switch (ballKickedOut.context.value)
            {
                case message::input::proto::GameEvents::Context::Value::TEAM:
                {
                    sendGameState("BallKickedOut<TEAM>", gameState);
                    break;
                }

                case message::input::proto::GameEvents::Context::Value::OPPONENT:
                {
                    sendGameState("BallKickedOut<OPPONENT>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown ball kicked out context:", std::string(ballKickedOut.context));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<KickOffTeam>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("KickOffTeam", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase>, With<GameState>>().then([this](const GamePhase& gamePhase, const GameState& gameState) {
            switch (gamePhase.phase.value)
            {
                case GameStateData::Phase::Value::INITIAL:
                {
                    sendGameState("GamePhase<INITIAL>", gameState);
                    break;
                }

                case GameStateData::Phase::Value::READY:
                {
                    sendGameState("GamePhase<READY>", gameState);
                    break;
                }

                case GameStateData::Phase::Value::SET:
                {
                    sendGameState("GamePhase<SET>", gameState);
                    break;
                }

                case GameStateData::Phase::Value::PLAYING:
                {
                    sendGameState("GamePhase<PLAYING>", gameState);
                    break;
                }

                case GameStateData::Phase::Value::TIMEOUT:
                {
                    sendGameState("GamePhase<TIMEOUT>", gameState);
                    break;
                }

                case GameStateData::Phase::Value::FINISHED:
                {
                    sendGameState("GamePhase<FINISHED>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown game phase:", std::string(gamePhase.phase));
                    break;
                }
            }
        }));

        handles["game_state"].push_back(on<Trigger<GameMode>, With<GameState>>().then([this](const GameMode& gameMode, const GameState& gameState) {
            switch (gameMode.mode.value)
            {
                case GameStateData::Mode::Value::NORMAL:
                {
                    sendGameState("GameMode<NORMAL>", gameState);
                    break;
                }

                case GameStateData::Mode::Value::PENALTY_SHOOTOUT:
                {
                    sendGameState("GameMode<PENALTY_SHOOTOUT>", gameState);
                    break;
                }

                case GameStateData::Mode::Value::OVERTIME:
                {
                    sendGameState("GameMode<OVERTIME>", gameState);
                    break;
                }

                default:
                {
                    log("Unknown game mode:", std::string(gameMode.mode));
                    break;
                }
            }
        }));
    }

    void NUbugger::sendGameState(std::string event, const GameState& gameState) {
        log("GameEvent:", event);

        send(gameState, 0, true);

        /*
        GameState gameController;
        GameStateData data;

        gameController.event  = event;

        data.phase            = gameState.data.phase;
        data.mode             = gameState.data.mode;
        data.first_half       = gameState.data.firstHalf;
        data.kicked_out_by_us = gameState.data.kickedOutByUs;
        data.kicked_out_time  = gameState.data.kickedOutTime;
        data.our_kick_off     = gameState.data.ourKickOff;
        data.primary_time     = gameState.data.primaryTime;
        data.secondary_time   = gameState.data.secondaryTime;

        GameStateData::Team team;
        auto& gameStateTeam = gameState.data.team;
        team.team_id        = gameStateTeam.teamId;
        team.score          = gameStateTeam.score;
        team.coach_message  = gameStateTeam.coachMessage;

        for (auto& gameStatePlayer : gameStateTeam.players) {
            GameStateData::Robot player;
            player.id             = gameStatePlayer.id;
            player.penalty_reason = gameStatePlayer.penaltyReason;
            player.unpenalised    = gameStatePlayer.unpenalised;
            team.players.push_back(player);
        }

        GameStateData::Team opponent;
        auto& gameStateOpponent = gameState.data.opponent;
        opponent.team_id        = gameStateOpponent.teamId;
        opponent.score          = gameStateOpponent.score;
        opponent.coach_message  = gameStateOpponent.coachMessage;

        for (auto& gameStatePlayer : gameStateOpponent.players) {
            GameStateData::Robot player;
            player.id             = gameStatePlayer.id;
            player.penalty_reason = gameStatePlayer.penaltyReason;
            player.unpenalised    = gameStatePlayer.unpenalised;
            opponent.players.push_back(player);
        }

        gameController.data = data;
        send(gameController, 0, true);
        */
    }
}
}
