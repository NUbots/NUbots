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

#ifndef MESSAGES_BEHAVIOUR_ACTIONS_H
#define MESSAGES_BEHAVIOUR_ACTIONS_H

#include <nuclear>
#include "messages/input/ServoID.h"

namespace messages {
    namespace behaviour {

        enum class LimbID {
            LEFT_LEG = 0,
            RIGHT_LEG = 1,
            LEFT_ARM = 2,
            RIGHT_ARM = 3,
            HEAD = 4
        };

        struct RegisterAction {

            size_t id;
            std::string name;

            std::vector<std::pair<float, std::set<LimbID>>> limbSet;

            std::function<void (std::set<LimbID>)> start;
            std::function<void (std::set<LimbID>)> kill;
            std::function<void (std::set<messages::input::ServoID>)> completed;
        };

        struct ActionPriorites {
            size_t id;

            std::vector<float> priorities;
        };

        struct ServoCommand {
            size_t source;

            NUClear::clock::time_point time;
            input::ServoID id;
            float position;
            float gain;
        };

        struct ActionStart {
            size_t id;
            std::string name;

            std::set<LimbID> limbs;
        };

        struct ActionKill {
            size_t id;
            std::string name;

            std::set<LimbID> limbs;
        };

        std::set<messages::input::ServoID> servosForLimb(const LimbID& limb);
        LimbID limbForServo(const messages::input::ServoID& servo);
    }
}

#endif

