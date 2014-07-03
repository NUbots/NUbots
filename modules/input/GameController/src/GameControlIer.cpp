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

#include "GameController.h"

namespace modules {
    namespace input {

        /**
         * TODO document
         *
         * @author Jordan Johnson
         */
        struct RobotInfo
        {
            uint8_t penalty;              // penalty state of the player
            uint8_t secsTillUnpenalised;  // estimate of time till unpenalised
        };

        /**
         * TODO document
         *
         * @author Jordan Johnson
         */
        struct TeamInfo
        {
            uint8_t teamNumber;           // unique team number
            uint8_t teamColour;           // colour of the team
            uint8_t score;                // team's score
            uint8_t penaltyShot;          // penalty shot counter
            uint16_t singleShots;         // bits represent penalty shot success
            uint8_t coachMessage[SPL_COACH_MESSAGE_SIZE]; // the coach's message to the team
            RobotInfo coach;
            RobotInfo players[MAX_NUM_PLAYERS]; // the team's players
        };

        /**
         * TODO document
         *
         * @author Jordan Johnson
         */
        struct RoboCupGameControlData
        {
            char header[4];               // header to identify the structure
            uint8_t version;              // version of the data structure
            uint8_t packetNumber;         // number incremented with each packet sent (with wraparound)
            uint8_t playersPerTeam;       // The number of players on a team
            uint8_t state;                // state of the game (STATE_READY, STATE_PLAYING, etc)
            uint8_t firstHalf;            // 1 = game in first half, 0 otherwise
            uint8_t kickOffTeam;          // the next team to kick off (TEAM_BLUE, TEAM_RED)
            uint8_t secondaryState;       // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
            uint8_t dropInTeam;           // team that caused last drop in
            uint16_t dropInTime;          // number of seconds passed since the last drop in.  -1 before first dropin
            uint16_t secsRemaining;       // estimate of number of seconds remaining in the half
            uint16_t secondaryTime;       // number of seconds shown as secondary time (remaining ready, until free ball, etc)
            TeamInfo teams[2];
        };
		
		
        GameController::GameController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
		
            on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>>() {
		
				printf("hello world");
		
			}
		
	
	
    }  // input
}  // modules
