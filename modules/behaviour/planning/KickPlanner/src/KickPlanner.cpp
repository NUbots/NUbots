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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KickPlanner.h"

#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"


namespace modules {
namespace behaviour {
namespace planning {

    using messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::motion::KickCommand;
    using messages::motion::WalkStopCommand;

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<Configuration<KickPlanner> > >([this](const Configuration<KickPlanner>& config) {
            TARGET_FIELD_POS = config["TARGET_FIELD_POS"];

            MIN_BALL_DISTANCE = config["MIN_BALL_DISTANCE"];
            KICK_CORRIDOR_WIDTH = config["KICK_CORRIDOR_WIDTH"];
            KICK_FORWARD_ANGLE_LIMIT = config["KICK_FORWARD_ANGLE_LIMIT"];
            KICK_SIDE_ANGLE_LIMIT = config["KICK_SIDE_ANGLE_LIMIT"];
        });


        on<Trigger<Ball>, With<Self>, With<FieldDescription>([this] (const Ball& ball, const Self& self) {

            // TODO check if the ball is within our kick box using some math or something

            // TODO check if we are alligned enough with the goal

            // TODO If we satisfy condtions then pick a kick to execute
            arma::vec3 goalPosition = arma::vec3({TARGET_FIELD_POS[0],TARGET_FIELD_POS[1],1});

            arma::vec2 normed_heading = arma::normalise(self.heading);
            arma::mat33 worldToRobotTransform = arma::mat33{      normed_heading[0],  normed_heading[1],         0,
                                                                 -normed_heading[1],  normed_heading[0],         0,
                                                                                  0,                 0,         1};

            worldToRobotTransform.submat(0,2,1,2) = -worldToRobotTransform.submat(0,0,1,1) * self.position;
            
            arma::vec3 homogeneousKickTarget = worldToRobotTransform * goalPosition;
            arma::vec2 kickTarget = homogeneousKickTarget.rows(0,1);    //In robot coords

            if(ball.position[0] < MIN_BALL_DISTANCE && std::fabs(ball.position[1]) < KICK_CORRIDOR_WIDTH / 2){
                float targetBearing = std::atan2(kickTarget[1],kickTarget[0]);
                if( std::fabs(targetBearing) < KICK_FORWARD_ANGLE_LIMIT / 2){
                    if(ball.position[1] < 0){
                        // Right front kick
                        emit(std::make_unique<WalkStopCommand>()); // Stop the walk
                        emit(std::make_unique<KickCommand>(KickCommand{{1,  0, 0}, LimbID::RIGHT_LEG }));
                        // TODO when the kick finishes, we need to start the walk
                        // Probably need to add something to the KickScript.cpp
                    } else {
                        // Left front kick
                        emit(std::make_unique<WalkStopCommand>()); // Stop the walk
                        emit(std::make_unique<KickCommand>(KickCommand{{1,  0, 0}, LimbID::LEFT_LEG }));
                        // TODO when the kick finishes, we need to start the walk
                        // Probably need to add something to the KickScript.cpp
                    }
                } else if (std::fabs(targetBearing) < KICK_SIDE_ANGLE_LIMIT) {
                    if(targetBearing < 0 && ball.position[1] < 0){
                        // Left side kick
                        emit(std::make_unique<WalkStopCommand>()); // Stop the walk
                        emit(std::make_unique<KickCommand>(KickCommand{{0, -1, 0}, LimbID::LEFT_LEG }));
                        // TODO when the kick finishes, we need to start the walk
                        // Probably need to add something to the KickScript.cpp
                    } else if(targetBearing > 0 && ball.position[1] > 0) {                    // Right side kick
                    emit(std::make_unique<WalkStopCommand>()); // Stop the walk
                    emit(std::make_unique<KickCommand>(KickCommand{{0,  1, 0}, LimbID::RIGHT_LEG }));
                    // TODO when the kick finishes, we need to start the walk
                    // Probably need to add something to the KickScript.cpp
                    }
                }
                
            }

            // Most of this code will be similar to that in PS3Walk.cpp

        });
    }

}
}
}

