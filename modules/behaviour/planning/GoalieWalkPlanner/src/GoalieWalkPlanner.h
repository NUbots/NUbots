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

#ifndef MODULES_BEHAVIOUR_PLANNING_GOALIEWALKPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_GOALIEWALKPLANNER_H

#include <nuclear>

namespace modules {
namespace behaviour {
namespace planning {

    class GoalieWalkPlanner : public NUClear::Reactor {
    private:
    	float command_timeout;
		float rotation_speed_factor;
		float max_rotation_speed;
		float translation_speed_factor;
		float max_translation_speed;
    public:
    	void updatePriority(const float& priority);
        /// @brief Called by the powerplant to build and setup the GoalieWalkPlanner reactor.
        explicit GoalieWalkPlanner(std::unique_ptr<NUClear::Environment> environment);
        int subsumptionId;
        ReactionHandle updateHandle;
        /// @brief the path to the configuration file for GoalieWalkPlanner
        static constexpr const char* CONFIGURATION_PATH = "GoalieWalkPlanner.yaml";
    };

}
}
}

#endif  // MODULES_BEHAVIOUR_PLANNING_GOALIEWALKPLANNER_H
