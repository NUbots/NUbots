/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULES_INPUT_GAMECONTROLLERDATA_HPP
#define MODULES_INPUT_GAMECONTROLLERDATA_HPP

namespace module::input::gamecontroller {
    constexpr const size_t MAX_NUM_PLAYERS = 20;
    constexpr const char RECEIVE_HEADER[4] = {'R', 'G', 'm', 'e'};
    constexpr const char RETURN_HEADER[4]  = {'R', 'G', 'r', 't'};
    constexpr const size_t RETURN_VERSION  = 4;

#pragma pack(push, 1)
    enum class State : uint8_t { INITIAL = 0, READY = 1, SET = 2, PLAYING = 3, FINISHED = 4 };

    enum class GamePhase : uint8_t {
        NORMAL           = 0,
        PENALTY_SHOOTOUT = 1,
        EXTRA_TIME       = 2,
        TIMEOUT          = 3,
    };

    enum class SetPlay : uint8_t {
        NONE               = 0,
        DIRECT_FREE_KICK   = 1,
        INDIRECT_FREE_KICK = 2,
        PENALTY_KICK       = 3,
        THROW_IN           = 4,
        GOAL_KICK          = 5,
        CORNER_KICK        = 6,
    };

    enum class TeamColour : uint8_t {
        BLUE   = 0,
        RED    = 1,
        YELLOW = 2,
        BLACK  = 3,
        WHITE  = 4,
        GREEN  = 5,
        ORANGE = 6,
        PURPLE = 7,
        BROWN  = 8,
        GRAY   = 9,
    };

    enum class PenaltyState : uint8_t {
        UNPENALISED             = 0,
        ILLEGAL_POSITIONING     = 1,
        MOTION_IN_SET           = 2,
        LOCAL_GAME_STUCK        = 3,
        INCAPABLE_ROBOT         = 4,
        PICK_UP                 = 5,
        BALL_HOLDING            = 6,
        LEAVING_THE_FIELD       = 7,
        PLAYING_WITH_ARMS_HANDS = 8,
        PLAYER_PUSHING          = 9,
        SENT_OFF                = 10,
        SUBSTITUTE              = 11,
    };

    struct Robot {
        PenaltyState penalty_state;   // penalty state of the player
        uint8_t penalised_time_left;  // estimate of time till unpenalised (seconds)
        uint8_t warnings;             // number of warnings
        uint8_t cautions;             // number of cautions (yellow cards)
    };

    struct Team {
        uint8_t team_id;                             // unique team number
        TeamColour field_player_colour;              // colour of the field players (TEAM_BLUE, etc)
        TeamColour goalkeeper_colour;                // colour of the goalkeeper (TEAM_BLUE, etc)
        uint8_t goalkeeper;                          // player number of the goalkeeper (0-MAX_NUM_PLAYERS)
        uint8_t score;                               // team's score
        uint8_t penalty_shot;                        // penalty shot counter
        uint16_t single_shots;                       // bits represent penalty shot success
        uint16_t message_budget;                     // remaining team message budget.
        std::array<Robot, MAX_NUM_PLAYERS> players;  // the team's players
    };

    struct GameControllerPacket {
        std::array<char, 4> header;  // header to identify the structure
        uint8_t version;             // version of the data structure
        uint8_t packet_number;       // number incremented with each packet sent
        uint8_t players_per_team;    // the number of players on a team
        uint8_t competition_type;    // type of the competition (small/middle/large)
        uint8_t stopped;             // 1 = play is currently stopped, 0 otherwise
        GamePhase game_phase;        // phase of the game
        State state;                 // state of the game
        SetPlay set_play;            // active set play
        bool first_half;             // 1 = game in first half, 0 otherwise
        uint8_t kicking_team;        // team number of the next team to kick off/free kick
        int16_t secs_remaining;      // estimate of seconds remaining in the half
        int16_t secondary_time;      // seconds shown as secondary time
        std::array<Team, 2> teams;
    };

    struct GameControllerReplyPacket {
        std::array<char, 4> header;  // "RGrt"
        uint8_t version;             // RETURN_VERSION
        uint8_t player;              // player number starts with 1
        uint8_t team;                // team number
        uint8_t fallen;              // 1 = fallen, 0 = upright
        float pose[3];               // x, y, theta in millimeters/radians
        float ball_age;              // seconds since ball last seen, -1 if never
        float ball[2];               // ball position relative to robot in millimeters
    };
#pragma pack(pop)

    inline std::ostream& operator<<(std::ostream& os, const Robot& robot) {
        os << "\t\tPenalty state......: " << uint(robot.penalty_state) << std::endl
           << "\t\tPenalised time left: " << uint(robot.penalised_time_left) << std::endl
           << "\t\tNumber of warnings: " << uint(robot.warnings) << std::endl
           << "\t\tYellow Card Count..: " << uint(robot.cautions) << std::endl;
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const Team& team) {
        os << "Team id: " << uint(team.team_id) << std::endl
           << "\tField player colour: " << uint(team.field_player_colour) << std::endl
           << "\tGoalkeeper colour..: " << uint(team.goalkeeper_colour) << std::endl
           << "\tGoalkeeper.........: " << uint(team.goalkeeper) << std::endl
           << "\tScore..............: " << uint(team.score) << std::endl
           << "\tPenalty shot.......: " << uint(team.penalty_shot) << std::endl
           << "\tSingle shots.......: " << uint(team.single_shots) << std::endl
           << "\tMessage budget.....: " << uint(team.message_budget) << std::endl;

        for (uint i = 0; i < team.players.size(); i++) {
            os << "\tRobot " << i + 1 << ":" << std::endl << team.players[i] << std::endl;
        }
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const GameControllerPacket& packet) {
        os << "Version.............: " << uint(packet.version) << std::endl
           << "Packet number.......: " << uint(packet.packet_number) << std::endl
           << "Players per team....: " << uint(packet.players_per_team) << std::endl
           << "Competition type....: " << uint(packet.competition_type) << std::endl
           << "Stopped.............: " << uint(packet.stopped) << std::endl
           << "Game phase..........: " << uint(packet.game_phase) << std::endl
           << "State...............: " << uint(packet.state) << std::endl
           << "Set play............: " << uint(packet.set_play) << std::endl
           << "First half..........: " << std::boolalpha << packet.first_half << std::endl
           << "Kicking team........: " << uint(packet.kicking_team) << std::endl
           << "Seconds remaining...: " << uint(packet.secs_remaining) << std::endl
           << "Secondary time......: " << uint(packet.secondary_time) << std::endl
           << packet.teams[0] << std::endl
           << packet.teams[1] << std::endl;
        return os;
    }
}  // namespace module::input::gamecontroller

#endif  // MODULES_INPUT_GAMECONTROLLERDATA_HPP
