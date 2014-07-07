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

                volatile bool listening = true;
                std::atomic<int> socket;
                uint port;
                GameControllerPacket state;
                uint TEAM_ID;
                uint PLAYER_ID;

                void kill();
                void run();
                void process(GameControllerPacket oldState, GameControllerPacket newState);
                Team& getOwnTeam(GameControllerPacket& state);
                Team& getOpponentTeam(GameControllerPacket& state);
            public:
                static constexpr const char* CONFIGURATION_PATH = "GameController.yaml";

                explicit GameController(std::unique_ptr<NUClear::Environment> environment);

        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_GAMECONTROLLER_H
