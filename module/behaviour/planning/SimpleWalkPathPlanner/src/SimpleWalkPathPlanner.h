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
#include <cmath>

#include "extension/Configuration.h"

#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/KickPlan.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"


namespace module {
    namespace behaviour {
        namespace planning {

                //using namespace message;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Josiah Walker
                 */
                class SimpleWalkPathPlanner : public NUClear::Reactor {
                private:
                    message::behaviour::MotionCommand latestCommand;
                    const size_t subsumptionId;
                    float turnSpeed = 0.8;
                    float forwardSpeed = 1;
                    float a = 7;
                    float b = 0;
                    float search_timeout = 3;

                    //-----------non-config variables (not defined in WalkPathPlanner.yaml)-----------

                    //info for the current walk
                    Eigen::Vector2d currentTargetPosition;
                    Eigen::Vector2d currentTargetHeading;
                    message::behaviour::KickPlan targetHeading;
                    Eigen::Vector2d targetPosition = {0, 0};

                    NUClear::clock::time_point timeBallLastSeen;
                    Eigen::Vector3d rBWw = {10,0,0};
                    bool robot_ground_space = true;
                    Eigen::Vector3d position = {1,0,0};//ball pos rel to robot
                public:
                    explicit SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_H

