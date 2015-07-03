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

#ifndef MODULES_BEHAVIOUR_PLANNING_OMPLPATHPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_OMPLPATHPLANNER_H

#include <nuclear>
#include "PathPlanner.h"

namespace modules {
namespace behaviour {
namespace planning {

    class OMPLPathPlanner : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the OMPLPathPlanner reactor.
        explicit OMPLPathPlanner(std::unique_ptr<NUClear::Environment> environment);

        /// @brief the path to the configuration file for OMPLPathPlanner
        static constexpr const char* CONFIGURATION_PATH = "OMPLPathPlanner.yaml";

    private:
    	PathPlanner pathPlanner;
        NUClear::clock::time_point lastPlanningTime;

		struct Config {
            
            float planning_interval = 5;
            float planning_time_limit = 0.5;
            bool draw_planning_tree = false;
            arma::vec2 target_offset = {0, 0};

        } cfg_;
    };

}
}
}

#endif
