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
        Score() : ownScore(0), opponentScore(0) {}
        Score(uint own, uint opp) : ownScore(own), opponentScore(opp) {}

        uint ownScore;
        uint opponentScore;
    };

    template <enum Context>
    struct GoalScored {
        GoalScored() : totalScore(0) {}
        GoalScored(uint score) : totalScore(score) {}

        uint totalScore;
    };

    template <enum Context>
    struct Penalisation {
        Penalisation() : robotId(0), ends(), reason() {}
        Penalisation(uint id, const NUClear::clock::time_point& time, const PenaltyReason& penalty)
            : robotId(id), ends(time), reason(penalty) {}

        uint robotId;
        NUClear::clock::time_point ends;
        PenaltyReason reason;
    };

    template <enum Context>
    struct Unpenalisation {
        Unpenalisation() : robotId(0) {}
        Unpenalisation(uint id) : robotId(id) {}

        uint robotId;
    };

    template <enum Context>
    struct CoachMessage {
        CoachMessage() : message("") {}
        CoachMessage(const std::string& msg) : message(msg) {}

        std::string message;
    };

    struct HalfTime {
        HalfTime() : firstHalf(false) {}
        HalfTime(bool half) : firstHalf(half) {}

        bool firstHalf;
    };

    template <enum Context>
    struct BallKickedOut {
        BallKickedOut() : time() {}
        BallKickedOut(const NUClear::clock::time_point& time) : time(time) {}

        NUClear::clock::time_point time;
    };

    struct KickOffTeam {
        KickOffTeam() : team() {}
        KickOffTeam(const Context& team) : team(team) {}

        Context team;
    };

    template <enum Phase>
    struct GamePhase;

    template <>
    struct GamePhase<Phase::INITIAL> {
    };

    template <>
    struct GamePhase<Phase::READY> {
        GamePhase() : readyTime() {}
        GamePhase(const NUClear::clock::time_point& time) : readyTime(time) {}

        NUClear::clock::time_point readyTime;
    };

    template <>
    struct GamePhase<Phase::SET> {
    };

    template <>
    struct GamePhase<Phase::PLAYING> {
        GamePhase() : endHalf(), ballFree() {}
        GamePhase(const NUClear::clock::time_point& half, const NUClear::clock::time_point& ball)
            : endHalf(half), ballFree(ball) {}

        NUClear::clock::time_point endHalf;
        NUClear::clock::time_point ballFree;
    };

    template <>
    struct GamePhase<Phase::TIMEOUT> {
        GamePhase() : ends() {}
        GamePhase(const NUClear::clock::time_point& time) : ends(time) {}

        NUClear::clock::time_point ends;
    };

    template <>
    struct GamePhase<Phase::FINISHED> {
        GamePhase() : nextHalf() {}
        GamePhase(const NUClear::clock::time_point& time) : nextHalf(time) {}

        NUClear::clock::time_point nextHalf;
    };

    template <enum Mode>
    struct GameMode {
    };

    struct GameState {
        GameState() : phase(), mode(), firstHalf(false), kickedOutByUs(false), kickedOutTime(), ourKickOff(false),
                      primaryTime(), secondaryTime(), team(), opponent(), self() {}

        struct Robot {
            Robot() : id(0), penaltyReason(), unpenalised() {}
            Robot(uint id, const PenaltyReason& reason, const NUClear::clock::time_point& time)
                : id(id), penaltyReason(reason), unpenalised(time) {}

            uint id;
            PenaltyReason penaltyReason;
            NUClear::clock::time_point unpenalised;
        };

        struct Team {
            Team() : teamId(0), score(0), coachMessage(""), players() {}
            Team(uint id, uint score, const std::string& msg, const std::vector<Robot>& players)
                : teamId(id), score(score), coachMessage(msg), players(players) {}

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
