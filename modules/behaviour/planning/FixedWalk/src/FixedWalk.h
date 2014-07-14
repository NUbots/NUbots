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
#include <list>

#include "messages/input/Sensors.h"
#include "messages/motion/WalkCommand.h"
#include "messages/behaviour/FixedWalkCommand.h"



namespace modules {
    namespace behaviour {
        namespace planning {

                //using namespace messages;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Jake Fountain
                 */
                class FixedWalk : public NUClear::Reactor {
                private:
                    std::unique_ptr<messages::motion::WalkCommand> getWalkCommand(const messages::behaviour::FixedWalkCommand::WalkSegment& segment, 
                                                                NUClear::clock::duration t, 
                                                                const messages::input::Sensors& sensors);
                    std::list<messages::behaviour::FixedWalkCommand::WalkSegment> walkSegments; 
                    time_t segmentStart;
                    NUClear::clock::duration segmentElapsedTimeBeforeFall;

                    arma::mat beginningOrientation;
                    bool active = false;
                    bool fallen = false;
                public:
                    explicit FixedWalk(std::unique_ptr<NUClear::Environment> environment);
                    static constexpr const char* CONFIGURATION_PATH = "FixedWalk.yaml";
                };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

