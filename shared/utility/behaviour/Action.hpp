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

#ifndef UTILITY_BEHAVIOUR_ACTIONS_HPP
#define UTILITY_BEHAVIOUR_ACTIONS_HPP

#include <nuclear>

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace utility::behaviour {

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    struct RegisterAction {

        size_t id;
        std::string name;

        std::vector<std::pair<float, std::set<LimbID>>> limbSet;

        std::function<void(std::set<LimbID>)> start;
        std::function<void(std::set<LimbID>)> kill;
        std::function<void(std::set<ServoID>)> completed;
    };

    struct ActionPriorities {
        size_t id;

        std::vector<float> priorities;
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
}  // namespace utility::behaviour

#endif
