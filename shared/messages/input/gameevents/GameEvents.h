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

 #ifndef MESSAGES_INPUT_GAMEEVENTS_H
 #define MESSAGES_INPUT_GAMEEVENTS_H

 #include <nuclear>

namespace messages {
namespace input {
namespace gameevents {

    enum Context {
        SELF,
        TEAM,
        OPPONENT
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
    };

    template <enum Context>
    struct BallKickedOut {
        NUClear::clock::time_point time;
    };


}
}
}

#endif  // MESSAGES_INPUT_GAMEEVENTS_H