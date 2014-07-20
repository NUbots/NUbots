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

#ifndef MODULES_INPUT_GAMECONTROLLER_H
#define MODULES_INPUT_GAMECONTROLLER_H

#include <nuclear>

#include "GameControllerData.h"
#include "messages/input/gameevents/GameEvents.h"

namespace modules {
    namespace input {

        /**
         * Monitors the match Game Controller
         *
         * @author Brendan Annable
         * @author Jordan Johnson
         * @author Trent Houliston
         */
        class GameController : public NUClear::Reactor {
            private:
                static constexpr const uint SUPPORTED_VERSION = 8;
                static constexpr const uint PLAYERS_PER_TEAM = 6;
                static constexpr const uint ACTIVE_PLAYERS_PER_TEAM = 4;
                static constexpr const uint NUM_TEAMS = 2;

                volatile bool listening = true;
                std::atomic<int> socket;
                uint port;
                uint TEAM_ID;
                uint PLAYER_ID;
                std::string BROADCAST_IP;

                bool penaltyOverride = false;
                bool selfPenalised = true;

                gamecontroller::GameControllerPacket packet;
                gamecontroller::Mode mode;

                void kill();
                void run();
                void process(const gamecontroller::GameControllerPacket& oldPacket, const gamecontroller::GameControllerPacket& newPacket);
                const gamecontroller::Team& getOwnTeam(const gamecontroller::GameControllerPacket& packet) const;
                const gamecontroller::Team& getOpponentTeam(const gamecontroller::GameControllerPacket& packet) const;
                messages::input::gameevents::PenaltyReason getPenaltyReason(const gamecontroller::PenaltyState& penaltyState) const;

                void sendReplyPacket(const gamecontroller::ReplyMessage& replyMessage) const;
            public:
                static constexpr const char* CONFIGURATION_PATH = "GameController.yaml";

                explicit GameController(std::unique_ptr<NUClear::Environment> environment);

        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_GAMECONTROLLER_H
