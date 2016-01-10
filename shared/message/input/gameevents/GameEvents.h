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

 #ifndef MESSAGE_INPUT_GAMEEVENTS_H
 #define MESSAGE_INPUT_GAMEEVENTS_H

 #include <nuclear>

namespace message {
namespace input {
namespace gameevents {

    enum class Mode {
        NORMAL,
        PENALTY_SHOOTOUT,
        OVERTIME
    };

    enum class Phase {
        INITIAL,
        READY,
        SET,
        PLAYING,
        TIMEOUT,
        FINISHED
    };

    enum Context {
        SELF,
        TEAM,
        OPPONENT,
        UNKNOWN
    };

    enum TeamColour {
        CYAN,
        MAGENTA
    };

    enum class PenaltyReason {
        UNPENALISED,
        BALL_MANIPULATION,
        PHYSICAL_CONTACT,
        ILLEGAL_ATTACK,
        ILLEGAL_DEFENSE,
        REQUEST_FOR_PICKUP,
        REQUEST_FOR_SERVICE,
        REQUEST_FOR_PICKUP_TO_SERVICE,
        SUBSTITUTE,
        MANUAL
    };

    struct Score {
        uint ownScore;
        uint opponentScore;
    };

    template <enum Context>
    struct GoalScored {
        uint totalScore;
    };

    template <enum Context>
    struct Penalisation {
        uint robotId;
        NUClear::clock::time_point ends;
        PenaltyReason reason;
    };

    template <enum Context>
    struct Unpenalisation {
        uint robotId;
    };

    template <enum Context>
    struct CoachMessage {
        std::string message;
    };

    struct HalfTime {
        bool firstHalf;
    };

    template <enum Context>
    struct BallKickedOut {
        NUClear::clock::time_point time;
    };

    struct KickOffTeam {
        Context team;
    };

    template <enum Phase>
    struct GamePhase;

    template <>
    struct GamePhase<Phase::INITIAL> {
    };

    template <>
    struct GamePhase<Phase::READY> {
        NUClear::clock::time_point readyTime;
    };

    template <>
    struct GamePhase<Phase::SET> {
    };

    template <>
    struct GamePhase<Phase::PLAYING> {
        NUClear::clock::time_point endHalf;
        NUClear::clock::time_point ballFree;
    };

    template <>
    struct GamePhase<Phase::TIMEOUT> {
        NUClear::clock::time_point ends;
    };

    template <>
    struct GamePhase<Phase::FINISHED> {
        NUClear::clock::time_point nextHalf;
    };

    template <enum Mode>
    struct GameMode {
    };

    struct GameState {

        struct Robot {
            uint id;
            PenaltyReason penaltyReason;
            NUClear::clock::time_point unpenalised;
        };

        struct Team {
            uint teamId;                         // unique team number
            uint score;                              // team's score
            std::string coachMessage;  // the coach's message to the team
            std::vector<Robot> players;
        };

        Phase phase;
        Mode mode;
        bool firstHalf;
        bool kickedOutByUs;
        NUClear::clock::time_point kickedOutTime;
        bool ourKickOff;
        NUClear::clock::time_point primaryTime;
        NUClear::clock::time_point secondaryTime;
        Team team;
        Team opponent;
        Robot self;
    };

}
}
}

#endif  // MESSAGE_INPUT_GAMEEVENTS_H
