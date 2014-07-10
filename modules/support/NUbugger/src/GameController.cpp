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

#include "messages/input/gameevents/GameEvents.h"

namespace modules {
namespace support {

    using namespace messages::input::gameevents;

    void NUbugger::provideGameController() {

        // HALP X_X

        handles["game_controller"].push_back(on<Trigger<TeamColour>, With<GameState>>([this](const TeamColour&, const GameState& gameState) {
            sendGameState("TeamColour", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Score>, With<GameState>>([this](const Score&, const GameState& gameState) {
            sendGameState("Score", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GoalScored<TEAM>>, With<GameState>>([this](const GoalScored<TEAM>&, const GameState& gameState) {
            sendGameState("GoalScored<TEAM>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GoalScored<OPPONENT>>, With<GameState>>([this](const GoalScored<OPPONENT>&, const GameState& gameState) {
            sendGameState("GoalScored<OPPONENT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Penalisation<SELF>>, With<GameState>>([this](const Penalisation<SELF>&, const GameState& gameState) {
            sendGameState("Penalisation<SELF>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Penalisation<TEAM>>, With<GameState>>([this](const Penalisation<TEAM>&, const GameState& gameState) {
            sendGameState("Penalisation<TEAM>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Penalisation<OPPONENT>>, With<GameState>>([this](const Penalisation<OPPONENT>&, const GameState& gameState) {
            sendGameState("Penalisation<OPPONENT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Unpenalisation<SELF>>, With<GameState>>([this](const Unpenalisation<SELF>&, const GameState& gameState) {
            sendGameState("Unpenalisation<SELF>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Unpenalisation<TEAM>>, With<GameState>>([this](const Unpenalisation<TEAM>&, const GameState& gameState) {
            sendGameState("Unpenalisation<TEAM>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<Unpenalisation<OPPONENT>>, With<GameState>>([this](const Unpenalisation<OPPONENT>&, const GameState& gameState) {
            sendGameState("Unpenalisation<OPPONENT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<CoachMessage<TEAM>>, With<GameState>>([this](const CoachMessage<TEAM>&, const GameState& gameState) {
            sendGameState("CoachMessage<TEAM>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<CoachMessage<OPPONENT>>, With<GameState>>([this](const CoachMessage<OPPONENT>&, const GameState& gameState) {
            sendGameState("CoachMessage<OPPONENT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<HalfTime>, With<GameState>>([this](const HalfTime&, const GameState& gameState) {
            sendGameState("HalfTime", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<BallKickedOut<TEAM>>, With<GameState>>([this](const BallKickedOut<TEAM>&, const GameState& gameState) {
            sendGameState("BallKickedOut<TEAM>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<BallKickedOut<OPPONENT>>, With<GameState>>([this](const BallKickedOut<OPPONENT>&, const GameState& gameState) {
            sendGameState("BallKickedOut<OPPONENT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<KickOffTeam>, With<GameState>>([this](const KickOffTeam&, const GameState& gameState) {
            sendGameState("KickOffTeam", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::INITIAL>>, With<GameState>>([this](const GamePhase<Phase::INITIAL>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::INITIAL>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::READY>>, With<GameState>>([this](const GamePhase<Phase::READY>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::READY>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::SET>>, With<GameState>>([this](const GamePhase<Phase::SET>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::SET>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::PLAYING>>, With<GameState>>([this](const GamePhase<Phase::PLAYING>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::PLAYING>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::TIMEOUT>>, With<GameState>>([this](const GamePhase<Phase::TIMEOUT>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::TIMEOUT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::FINISHED>>, With<GameState>>([this](const GamePhase<Phase::FINISHED>&, const GameState& gameState) {
            sendGameState("GamePhase<Phase::FINISHED>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::NORMAL>>, With<GameState>>([this](const GameMode<Mode::NORMAL>&, const GameState& gameState) {
            sendGameState("GameMode<Mode::NORMAL>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::PENALTY_SHOOTOUT>>, With<GameState>>([this](const GameMode<Mode::PENALTY_SHOOTOUT>&, const GameState& gameState) {
            sendGameState("GameMode<Mode::PENALTY_SHOOTOUT>", gameState);
        }));

        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::OVERTIME>>, With<GameState>>([this](const GameMode<Mode::OVERTIME>&, const GameState& gameState) {
            sendGameState("GameMode<Mode::OVERTIME>", gameState);
        }));
    }

    void NUbugger::sendGameState(std::string event, const GameState&) {
        log(event);
    }
}
}