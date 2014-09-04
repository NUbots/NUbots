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

#ifndef MODULES_BEHAVIOUR_PLANNERS_WALKPATHPLANNER_H
#define MODULES_BEHAVIOUR_PLANNERS_WALKPATHPLANNER_H

#include <nuclear>
#include <armadillo>
#include <cmath>
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/behaviour/WalkStrategy.h"


namespace modules {
    namespace behaviour {
        namespace planning {

                //using namespace messages;
                /**
                 * Executes a getup script if the robot falls over.
                 *
                 * @author Josiah Walker
                 */
                class WalkPathPlanner : public NUClear::Reactor {
                private:
                    //XXX: load these values from walk config
                    float turnSpeed;
                    float forwardSpeed;
                    float footSeparation;

                    //XXX: load from robot model
                    float footSize;

                    //timers for starting turning and walking
                    double walkStartTime;
                    double walkTurnTime;

                    //walk accel/deccel controls
                    double accelerationTime;
                    float accelerationFraction;

                    //approach speeds
                    float closeApproachSpeed;
                    float closeApproachDistance;
                    float midApproachSpeed;
                    float midApproachDistance;

                    //turning values
                    float turnDeviation;

                    //state controls
                    int approachDistance;

                    //hystereses
                    float distanceHysteresis;
                    float turningHysteresis;
                    float positionHysteresis;

                    //ball lineup
                    //vector<float> ballApproachAngle;
                    //vector<int> ballKickFoot;
                    float ballLineupDistance;
                    float ballLineupMinDistance;

                    //extra config options
                    bool useAvoidance;
                    float assumedObstacleWidth;
                    float avoidDistance;

                    float bearingSensitivity;
                    float ApproachCurveFactor;


                    //-----------non-config variables-----------

                    //hysteresis variables
                    int turning;
                    int distanceIncrement;

                    //info for the current walk
                    arma::vec2 currentTargetPosition;
                    arma::vec2 currentTargetHeading;
                    messages::behaviour::WalkApproach planType;
                    messages::behaviour::WalkTarget targetHeading,targetPosition;

                    //-----------internal fns--------------
                    arma::vec generateWalk(const arma::vec& move, bool omniPositioning);
                    arma::vec approachFromDirection(const messages::localisation::Self& self,
                                                   const arma::vec2& target,
                                                   const arma::vec2& direction);
                    arma::vec goToPoint(const messages::localisation::Self& self,
                                         const arma::vec2& target,
                                         const arma::vec2& direction);
                    arma::vec avoidObstacles(const std::vector<messages::vision::Obstacle>& robotPosition,
                                              const arma::vec3& movePlan);


                public:
                    explicit WalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
                    static constexpr const char* CONFIGURATION_PATH = "WalkPathPlanner.yaml";
            };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

