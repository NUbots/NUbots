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
        // TODO.
        handles["game_controller"].push_back(on<Trigger<TeamColour>>([this](const TeamColour&) {}));
        handles["game_controller"].push_back(on<Trigger<Score>>([this](const Score&) {}));
        handles["game_controller"].push_back(on<Trigger<GoalScored<TEAM>>>([this](const GoalScored<TEAM>&) {}));
        handles["game_controller"].push_back(on<Trigger<GoalScored<OPPONENT>>>([this](const GoalScored<OPPONENT>&) {}));
        handles["game_controller"].push_back(on<Trigger<Penalisation<SELF>>>([this](const Penalisation<SELF>&) {}));
        handles["game_controller"].push_back(on<Trigger<Penalisation<TEAM>>>([this](const Penalisation<TEAM>&) {}));
        handles["game_controller"].push_back(on<Trigger<Penalisation<OPPONENT>>>([this](const Penalisation<OPPONENT>&) {}));
        handles["game_controller"].push_back(on<Trigger<Unpenalisation<SELF>>>([this](const Unpenalisation<SELF>&) {}));
        handles["game_controller"].push_back(on<Trigger<Unpenalisation<TEAM>>>([this](const Unpenalisation<TEAM>&) {}));
        handles["game_controller"].push_back(on<Trigger<Unpenalisation<OPPONENT>>>([this](const Unpenalisation<OPPONENT>&) {}));
        handles["game_controller"].push_back(on<Trigger<CoachMessage<TEAM>>>([this](const CoachMessage<TEAM>&) {}));
        handles["game_controller"].push_back(on<Trigger<CoachMessage<OPPONENT>>>([this](const CoachMessage<OPPONENT>&) {}));
        handles["game_controller"].push_back(on<Trigger<HalfTime>>([this](const HalfTime&) {}));
        handles["game_controller"].push_back(on<Trigger<BallKickedOut<TEAM>>>([this](const BallKickedOut<TEAM>&) {}));
        handles["game_controller"].push_back(on<Trigger<BallKickedOut<OPPONENT>>>([this](const BallKickedOut<OPPONENT>&) {}));
        handles["game_controller"].push_back(on<Trigger<KickOffTeam>>([this](const KickOffTeam&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::INITIAL>>>([this](const GamePhase<Phase::INITIAL>&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::READY>>>([this](const GamePhase<Phase::READY>&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::SET>>>([this](const GamePhase<Phase::SET>&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::PLAYING>>>([this](const GamePhase<Phase::PLAYING>&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::TIMEOUT>>>([this](const GamePhase<Phase::TIMEOUT>&) {}));
        handles["game_controller"].push_back(on<Trigger<GamePhase<Phase::FINISHED>>>([this](const GamePhase<Phase::FINISHED>&) {}));
        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::NORMAL>>>([this](const GameMode<Mode::NORMAL>&) {}));
        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::PENALTY_SHOOTOUT>>>([this](const GameMode<Mode::PENALTY_SHOOTOUT>&) {}));
        handles["game_controller"].push_back(on<Trigger<GameMode<Mode::OVERTIME>>>([this](const GameMode<Mode::OVERTIME>&) {}));
    }
}
}