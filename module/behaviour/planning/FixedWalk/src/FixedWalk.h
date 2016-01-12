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

#ifndef MODULES_BEHAVIOUR_PLANNERS_FIXEDWALK_H
#define MODULES_BEHAVIOUR_PLANNERS_FIXEDWALK_H

#include <nuclear>
#include <armadillo>
#include <list>

#include "message/input/Sensors.h"
#include "message/motion/WalkCommand.h"
#include "message/behaviour/FixedWalkCommand.h"



namespace module {
    namespace behaviour {
        namespace planning {

                //using namespace message;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Jake Fountain
                 */
                class FixedWalk : public NUClear::Reactor {
                private:
                    std::unique_ptr<message::motion::WalkCommand> getWalkCommand(const message::behaviour::FixedWalkCommand::WalkSegment& segment,
                                                                NUClear::clock::duration t,
                                                                const message::input::Sensors& sensors);
                    std::list<message::behaviour::FixedWalkCommand::WalkSegment> walkSegments;
                    NUClear::clock::time_point segmentStart;
                    NUClear::clock::duration segmentElapsedTimeBeforeFall;

                    arma::mat beginningOrientation;
                    bool active = false;
                    bool fallen = false;
                public:
                    explicit FixedWalk(std::unique_ptr<NUClear::Environment> environment);
                };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

