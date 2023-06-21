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

#ifndef MODULES_INPUT_GAMECONTROLLERDATA_HPP
#define MODULES_INPUT_GAMECONTROLLERDATA_HPP

namespace module::input::gamecontroller {
    constexpr const size_t MAX_NUM_PLAYERS        = 11;
    constexpr const size_t SPL_COACH_MESSAGE_SIZE = 253;
    constexpr const char RECEIVE_HEADER[4]        = {'R', 'G', 'm', 'e'};
    constexpr const char RETURN_HEADER[4]         = {'R', 'G', 'r', 't'};
    constexpr const size_t RETURN_VERSION         = 2;

#pragma pack(push, 1)
    enum class State : uint8_t { INITIAL = 0, READY = 1, SET = 2, PLAYING = 3, FINISHED = 4 };

    enum class Mode : uint8_t {
        NORMAL            = 0,
        PENALTY_SHOOTOUT  = 1,
        OVERTIME          = 2,
        TIMEOUT           = 3,
        DIRECT_FREEKICK   = 4,
        INDIRECT_FREEKICK = 5,
        PENALTYKICK       = 6,
        CORNER_KICK       = 7,
        GOAL_KICK         = 8,
        THROW_IN          = 9,
    };

    enum class GameType : uint8_t { ROUND_ROBIN = 0, PLAYOFF = 1, DROPIN = 2 };

    enum class TeamColour : uint8_t { CYAN = 0, MAGENTA = 1, DROPBALL = 255 };

    enum class ReplyMessage : uint8_t { PENALISED = 0, UNPENALISED = 1, ALIVE = 2 };

    enum class PenaltyState : uint8_t {
        // General??
        UNPENALISED = 0,

        // SPL
        ILLEGAL_BALL_CONTACT   = 1,
        PLAYER_PUSHING         = 2,
        ILLEGAL_MOTION_IN_SET  = 3,
        INACTIVE_PLAYER        = 4,
        ILLEGAL_DEFENDER       = 5,
        LEAVING_THE_FIELD      = 6,
        KICK_OFF_GOAL          = 7,
        SPL_REQUEST_FOR_PICKUP = 8,
        COACH_MOTION           = 9,

        // General??
        SUBSTITUTE = 14,
        MANUAL     = 15,

        // HL
        BALL_MANIPULATION   = 30,
        PHYSICAL_CONTACT    = 31,
        ILLEGAL_ATTACK      = 32,
        ILLEGAL_DEFENSE     = 33,
        REQUEST_FOR_PICKUP  = 34,
        REQUEST_FOR_SERVICE = 35,

        // General??
        UNKNOWN = 255
    };

    struct Robot {
        PenaltyState penaltyState;  // penalty state of the player
        uint8_t penalisedTimeLeft;  // estimate of time till unpenalised (seconds)
        uint8_t numberOfWarnings;   // number of warnings
        uint8_t yellowCardCount;    // number of yellow cards
        uint8_t redCardCount;       // number of red cards
        bool goalKeeper;            // flags if robot is goal keeper
    };

    struct Team {
        uint8_t teamId;                                         // unique team number
        TeamColour teamColour;                                  // colour of the team
        uint8_t score;                                          // team's score
        uint8_t penaltyShot;                                    // penalty shot counter
        uint16_t singleShots;                                   // bits represent penalty shot success
        uint8_t coachSequence;                                  // sequence number of the coach's message
        std::array<char, SPL_COACH_MESSAGE_SIZE> coachMessage;  // the coach's message to the team
        Robot coach;                                            // the coach
        std::array<Robot, MAX_NUM_PLAYERS> players;             // the team's players
    };

    struct GameControllerPacket {
        std::array<char, 4> header;  // header to identify the structure
        uint16_t version;            // version of the data structure
        uint8_t packetNumber;        // number incremented with each packet sent (with wraparound)
        uint8_t playersPerTeam;      // the number of players on a team
        GameType gameType;           // type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
        State state;                 // state of the game (STATE_READY, STATE_PLAYING, etc)
        bool firstHalf;              // 1 = game in first half, 0 otherwise
        uint8_t kickOffTeam;         // the team number of the next team to kick off or DROPBALL
        Mode mode;                   // extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
        std::array<uint8_t, 4> secondaryStateInfo;  // Extra info on the secondary state
        TeamColour dropInTeam;                      // number of team that caused last drop in
        int16_t dropInTime;      // number of seconds passed since the last drop in. -1 (0xffff) before first dropin
        uint16_t secsRemaining;  // estimate of number of seconds remaining in the half
        uint16_t secondaryTime;  // number of seconds shown as secondary time (remaining ready, until free ball, etc)
        std::array<Team, 2> teams;
    };

    struct GameControllerReplyPacket {
        std::array<char, 4> header;
        uint8_t version;
        uint8_t team;          // team number
        uint8_t player;        // player number starts with 1
        ReplyMessage message;  // one of the three messages defined above
    };
#pragma pack(pop)

    inline std::ostream& operator<<(std::ostream& os, const Robot& robot) {
        os << "\t\tPenalty state......: " << uint(robot.penaltyState) << std::endl
           << "\t\tPenalised time left: " << uint(robot.penalisedTimeLeft) << std::endl
           << "\t\tNumber of warnings: " << uint(robot.numberOfWarnings) << std::endl
           << "\t\tYellow Card Count..: " << uint(robot.yellowCardCount) << std::endl
           << "\t\tRed Card Count.....: " << uint(robot.redCardCount) << std::endl
           << "\t\tGoalkeeper: " << std::boolalpha << robot.goalKeeper << std::endl;
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const Team& team) {
        os << "Team id: " << uint(team.teamId) << std::endl
           << "\tTeam colour...: " << uint(team.teamColour) << std::endl
           << "\tScore.........: " << uint(team.score) << std::endl
           << "\tPenalty shot..: " << uint(team.penaltyShot) << std::endl
           << "\tSingle shots..: " << uint(team.singleShots) << std::endl
           << "\tCoach Sequence: " << uint(team.coachSequence) << std::endl
           << "\tCoach message.: " << std::string(team.coachMessage.data()) << std::endl;

        for (uint i = 0; i < team.players.size(); i++) {
            os << "\tRobot " << i + 1 << ":" << std::endl << team.players[i] << std::endl;
        }
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const GameControllerPacket& packet) {
        os << "Version.............: " << uint(packet.version) << std::endl
           << "Packet number.......: " << uint(packet.packetNumber) << std::endl
           << "Players per team....: " << uint(packet.playersPerTeam) << std::endl
           << "Game Type...........: " << uint(packet.gameType) << std::endl
           << "State...............: " << uint(packet.state) << std::endl
           << "First half..........: " << std::boolalpha << packet.firstHalf << std::endl
           << "Kick off team.......: " << uint(packet.kickOffTeam) << std::endl
           << "Mode................: " << uint(packet.mode) << std::endl
           << "Secondary State Info: "
           << std::string(packet.secondaryStateInfo.begin(), packet.secondaryStateInfo.end()) << std::endl
           << "Drop in team........: " << uint(packet.dropInTeam) << std::endl
           << "Drop in time........: " << uint(packet.dropInTime) << std::endl
           << "Seconds remaining...: " << uint(packet.secsRemaining) << std::endl
           << "Secondary time......: " << uint(packet.secondaryTime) << std::endl
           << packet.teams[0] << std::endl
           << packet.teams[1] << std::endl;
        return os;
    }
}  // namespace module::input::gamecontroller

#endif  // MODULES_INPUT_GAMECONTROLLERDATA_HPP
