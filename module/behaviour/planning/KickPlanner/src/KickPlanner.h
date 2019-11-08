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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_H

#include <armadillo>
#include <nuclear>

#include "message/input/Sensors.h"
#include "message/motion/KickCommand.h"
namespace module {
namespace behaviour {
    namespace planning {

        class KickPlanner : public NUClear::Reactor {
        public:
            /// @brief Called by the powerplant to build and setup the KickPlanner reactor.
            explicit KickPlanner(std::unique_ptr<NUClear::Environment> environment);

        private:
            bool kickValid(const arma::vec3& ballPos);
            message::motion::KickPlannerConfig cfg;
            NUClear::clock::time_point ballLastSeen;
            NUClear::clock::time_point lastTimeValid;
        };
    }  // namespace planning
}  // namespace behaviour
}  // namespace module


#endif
