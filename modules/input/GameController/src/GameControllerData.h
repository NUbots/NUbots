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

#ifndef MODULES_INPUT_GAMECONTROLLERDATA_H
#define MODULES_INPUT_GAMECONTROLLERDATA_H

namespace modules {
namespace input {
namespace gamecontroller {
    constexpr const size_t MAX_NUM_PLAYERS = 11;
    constexpr const size_t SPL_COACH_MESSAGE_SIZE = 40;
    constexpr const char RECEIVE_HEADER[4] = {'R', 'G', 'm', 'e'};
    constexpr const char RETURN_HEADER[4] = {'R', 'G', 'r', 't'};
    constexpr const size_t RETURN_VERSION = 2;

    #pragma pack(push, 1)
    enum class State : uint8_t {
        INITIAL  = 0,
        READY    = 1,
        SET      = 2,
        PLAYING  = 3,
        FINISHED = 4
    };

    enum class Mode : uint8_t {
        NORMAL           = 0,
        PENALTY_SHOOTOUT = 1,
        OVERTIME         = 2,
        TIMEOUT          = 3
    };

    enum class TeamColour : uint8_t {
        CYAN    = 0,
        MAGENTA = 1
    };

    enum class ReplyMessage : uint8_t {
        PENALISED = 0,
        UNPENALISED = 1,
        ALIVE = 2
    };

    enum class PenaltyState : uint8_t {
        UNPENALISED                   = 0,
        BALL_MANIPULATION             = 1,
        PHYSICAL_CONTACT              = 2,
        ILLEGAL_ATTACK                = 3,
        ILLEGAL_DEFENSE               = 4,
        REQUEST_FOR_PICKUP            = 5,
        REQUEST_FOR_SERVICE           = 6,
        REQUEST_FOR_PICKUP_TO_SERVICE = 7,
        SUBSTITUTE                    = 14,
        MANUAL                        = 15
    };


    struct Robot {
        PenaltyState penaltyState;  // penalty state of the player
        uint8_t penalisedTimeLeft;  // estimate of time till unpenalised (seconds)
    };

    struct Team {
        uint8_t teamId;                                        // unique team number
        TeamColour teamColour;                                 // colour of the team
        uint8_t score;                                         // team's score
        uint8_t penaltyShot;                                   // penalty shot counter
        uint16_t singleShots;                                  // bits represent penalty shot success
        std::array<char, SPL_COACH_MESSAGE_SIZE> coachMessage; // the coach's message to the team
        Robot coach;                                           // the coach
        Robot players[MAX_NUM_PLAYERS];                        // the team's players
    };

    struct GameControllerPacket {
        std::array<char, 4> header;    // header to identify the structure
        uint8_t version;               // version of the data structure
        uint8_t packetNumber;          // number incremented with each packet sent (with wraparound)
        uint8_t playersPerTeam;        // The number of players on a team
        State state;                   // state of the game (STATE_READY, STATE_PLAYING, etc)
        bool firstHalf;                // 1 = game in first half, 0 otherwise
        TeamColour kickOffTeam;        // the next team to kick off (TEAM_BLUE, TEAM_RED)
        Mode mode;                     // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
        TeamColour dropInTeam;         // team that caused last drop in
        int16_t dropInTime;            // number of seconds passed since the last drop in.  -1 before first dropin
        uint16_t secsRemaining;        // estimate of number of seconds remaining in the half
        uint16_t secondaryTime;        // number of seconds shown as secondary time (remaining ready, until free ball, etc)
        Team teams[2];
    };

    struct GameControllerReplyPacket {
        std::array<char, 4> header;
        uint8_t version;
        uint8_t team;    // team number
        uint8_t player;  // player number starts with 1
        ReplyMessage message; // one of the three messages defined above
    };
    #pragma pack(pop)
}
}
}

#endif  // MODULES_INPUT_GAMECONTROLLERDATA_H