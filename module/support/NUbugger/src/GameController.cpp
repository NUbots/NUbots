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

#include "message/input/gameevents/GameEvents.h"

#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using namespace message::input::gameevents;
    using GameStateProto = message::input::proto::GameState;
    using GameStateData = message::input::proto::GameState::Data;
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

        handles["game_state"].push_back(on<Trigger<GoalScored<TEAM>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GoalScored<TEAM>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GoalScored<OPPONENT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GoalScored<OPPONENT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Penalisation<SELF>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Penalisation<SELF>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Penalisation<TEAM>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Penalisation<TEAM>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Penalisation<OPPONENT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Penalisation<OPPONENT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Unpenalisation<SELF>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Unpenalisation<SELF>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Unpenalisation<TEAM>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Unpenalisation<TEAM>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<Unpenalisation<OPPONENT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("Unpenalisation<OPPONENT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<CoachMessage<TEAM>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("CoachMessage<TEAM>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<CoachMessage<OPPONENT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("CoachMessage<OPPONENT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<HalfTime>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("HalfTime", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<BallKickedOut<TEAM>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("BallKickedOut<TEAM>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<BallKickedOut<OPPONENT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("BallKickedOut<OPPONENT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<KickOffTeam>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("KickOffTeam", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::INITIAL>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::INITIAL>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::READY>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::READY>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::SET>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::SET>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::PLAYING>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::PLAYING>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::TIMEOUT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::TIMEOUT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GamePhase<Phase::FINISHED>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GamePhase<Phase::FINISHED>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GameMode<Mode::NORMAL>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GameMode<Mode::NORMAL>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GameMode<Mode::PENALTY_SHOOTOUT>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GameMode<Mode::PENALTY_SHOOTOUT>", gameState);
        }));

        handles["game_state"].push_back(on<Trigger<GameMode<Mode::OVERTIME>>, With<GameState>>().then([this](const GameState& gameState) {
            sendGameState("GameMode<Mode::OVERTIME>", gameState);
        }));
    }

    void NUbugger::sendGameState(std::string event, const GameState& gameState) {
        log("GameEvent:", event);

        GameStateProto gameController;

        gameController.set_event(event);
        auto* data = gameController.mutable_data();

        data->set_phase(getPhase(gameState.phase));
        data->set_mode(getMode(gameState.mode));
        data->set_first_half(gameState.firstHalf);
        data->set_kicked_out_by_us(gameState.kickedOutByUs);
        data->set_kicked_out_time(getUtcTimestamp(gameState.kickedOutTime));
        data->set_our_kick_off(gameState.ourKickOff);
        data->set_primary_time(getUtcTimestamp(gameState.primaryTime));
        data->set_secondary_time(getUtcTimestamp(gameState.secondaryTime));

        auto* team = data->mutable_team();
        auto& gameStateTeam = gameState.team;
        team->set_team_id(gameStateTeam.teamId);
        team->set_score(gameStateTeam.score);
        team->set_coach_message(gameStateTeam.coachMessage);

        for (auto& gameStatePlayer : gameStateTeam.players) {
            auto* player = team->add_players();
            player->set_id(gameStatePlayer.id);
            player->set_penalty_reason(getPenaltyReason(gameStatePlayer.penaltyReason));
            player->set_unpenalised(getUtcTimestamp(gameStatePlayer.unpenalised));
        }

        auto* opponent = data->mutable_opponent();
        auto& gameStateOpponent = gameState.opponent;
        opponent->set_team_id(gameStateOpponent.teamId);
        opponent->set_score(gameStateOpponent.score);
        opponent->set_coach_message(gameStateOpponent.coachMessage);
        for (auto& gameStatePlayer : gameStateOpponent.players) {
            auto* player = opponent->add_players();
            player->set_id(gameStatePlayer.id);
            player->set_penalty_reason(getPenaltyReason(gameStatePlayer.penaltyReason));
            player->set_unpenalised(getUtcTimestamp(gameStatePlayer.unpenalised));
        }

        send(gameController, 0, true);
    }

    GameStateData::Phase NUbugger::getPhase(const Phase& phase) {
        switch (phase) {
            case Phase::INITIAL:
                return GameStateData::INITIAL;
            case Phase::READY:
                return GameStateData::READY;
            case Phase::SET:
                return GameStateData::SET;
            case Phase::PLAYING:
                return GameStateData::PLAYING;
            case Phase::TIMEOUT:
                return GameStateData::TIMEOUT;
            case Phase::FINISHED:
                return GameStateData::FINISHED;
            default:
                throw std::runtime_error("Invalid Phase");
        }
    }

    GameStateData::Mode NUbugger::getMode(const Mode& mode) {
        switch (mode) {
            case Mode::NORMAL:
                return GameStateData::NORMAL;
            case Mode::PENALTY_SHOOTOUT:
                return GameStateData::PENALTY_SHOOTOUT;
            case Mode::OVERTIME:
                return GameStateData::OVERTIME;
            default:
                throw std::runtime_error("Invalid Mode");
        }
    }

    GameStateData::PenaltyReason NUbugger::getPenaltyReason(const PenaltyReason& penaltyReason) {
        switch (penaltyReason) {
            case PenaltyReason::UNPENALISED:
                return GameStateData::UNPENALISED;
            case PenaltyReason::BALL_MANIPULATION:
                return GameStateData::BALL_MANIPULATION;
            case PenaltyReason::PHYSICAL_CONTACT:
                return GameStateData::PHYSICAL_CONTACT;
            case PenaltyReason::ILLEGAL_ATTACK:
                return GameStateData::ILLEGAL_ATTACK;
            case PenaltyReason::ILLEGAL_DEFENSE:
                return GameStateData::ILLEGAL_DEFENSE;
            case PenaltyReason::REQUEST_FOR_PICKUP:
                return GameStateData::REQUEST_FOR_PICKUP;
            case PenaltyReason::REQUEST_FOR_SERVICE:
                return GameStateData::REQUEST_FOR_SERVICE;
            case PenaltyReason::REQUEST_FOR_PICKUP_TO_SERVICE:
                return GameStateData::REQUEST_FOR_PICKUP_TO_SERVICE;
            case PenaltyReason::SUBSTITUTE:
                return GameStateData::SUBSTITUTE;
            case PenaltyReason::MANUAL:
                return GameStateData::MANUAL;
            default:
                throw std::runtime_error("Invalid Penalty Reason");
        }
    }
}
}
