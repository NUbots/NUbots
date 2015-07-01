/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_WALKPATHFOLLOWER_H
#define MODULES_BEHAVIOUR_PLANNING_WALKPATHFOLLOWER_H

#include <nuclear>

namespace modules {
namespace behaviour {
namespace planning {

    class WalkPathFollower : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the WalkPathFollower reactor.
        explicit WalkPathFollower(std::unique_ptr<NUClear::Environment> environment);

        /// @brief the path to the configuration file for WalkPathFollower
        static constexpr const char* CONFIGURATION_PATH = "WalkPathFollower.yaml";
    };

}
}
}

#endif  // MODULES_BEHAVIOUR_PLANNING_WALKPATHFOLLOWER_H
