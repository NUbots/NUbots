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

#ifndef MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_H
#define MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_H

#include <nuclear>
#include <armadillo>
#include <cmath>
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/behaviour/MotionCommand.h"


namespace modules {
    namespace behaviour {
        namespace planning {

                //using namespace messages;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Josiah Walker
                 */
                class SimpleWalkPathPlanner : public NUClear::Reactor {
                private:
                    float turnSpeed;
                    float forwardSpeed;
                    float a;
                    float b;

                    //-----------non-config variables (not defined in WalkPathPlanner.yaml)-----------

                    //info for the current walk
                    arma::vec2 currentTargetPosition;
                    arma::vec2 currentTargetHeading;
                    messages::behaviour::WalkApproach planType;
                    messages::behaviour::WalkTarget targetHeading,targetPosition;

                public:
                    explicit SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_H

