/*
 * Responsible for creating scripted walk sequences
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

#ifndef MODULES_BEHAVIOUR_PLANNERS_WALKPATHPLANNER_H
#define MODULES_BEHAVIOUR_PLANNERS_WALKPATHPLANNER_H

#include <nuclear>
#include <armadillo>



namespace modules {
    namespace behaviour {
        namespace skills {

                //using namespace messages;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Jake Fountain
                 */
                class WalkPathPlanner : public NUClear::Reactor {
            

                public:
                    explicit WalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
                    static constexpr const char* CONFIGURATION_PATH = "WalkPathPlanner.yaml";
            };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

