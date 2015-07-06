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

#include "DivePlanner.h"

#include "utility/support/yaml_armadillo.h"
#include "messages/motion/DiveCommand.h"
#include "messages/motion/WalkCommand.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/behaviour/DivePlan.h"
#include "messages/vision/VisionObjects.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::motion::DiveCommand;
    using messages::support::Configuration;
    using messages::motion::WalkStopCommand;
    using messages::input::LimbID;
    using messages::behaviour::DivePlan;

    DivePlanner::DivePlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {
        
        
        // Do some configuration
        on<Trigger<Configuration<DivePlanner> > >([this](const Configuration<DivePlanner>& config) {
        	SPEED_THRESHOLD = config["SPEED_THRESHOLD"].as<float>();
        	DISTANCE_THRESHOLD = config["DISTANCE_THRESHOLD"].as<float>();
        });

        // DivePlan provides target (probablly ball?) position (vec2)
        on<Trigger<Ball>, With<std::vector<Self>>, With<std::vector<messages::vision::Ball>>, With<DivePlan>>([this] (
        	const Ball& ball,
        	const std::vector<Self>& selfs,
        	const std::vector<messages::vision::Ball>& vision_balls,
        	const DivePlan& divePlan) {

        	// TODO: why are these used?
        	(void)selfs;
        	(void)divePlan;

            


            if(vision_balls.size()>0 && //It means a ball was detected.
               ball.position[0] > 0 && //
               -ball.velocity[0] > SPEED_THRESHOLD && //Negative velocity means the ball is coming towards the goalie.
               ball.position[0] < DISTANCE_THRESHOLD // If the ball is close enough (0.5 metres?) to the goalie.
               ){

                //NUClear::log("Ball Vel:", -ball.velocity[0] , ball.position[0]);

                // Position [0] : x
                // Position [1] : y
                /*
                 * Dive direction is determined by the RT position of ball.
                 * TODO 1: This is problematic as there will be latence which has to be considered 
                 * if the velocity of ball is beyond a threshold. Might be good if there is any sort of prediction.
                 *
                 * TODO 2: Posture before dive. Banana dive?
                 *
                 * TODO 3: If banana dive, how can the robot tell if it is facing the wrong direction and 
                 * turn back to the correct direction (facing opponent)? Is that based on localisation?
                 *
                 *               + x
                 *                ^
                 *                |
                 *          + y   |   -y
                 *  LEFT<---------.--------- RIGHT
                 */
                
                if(ball.position[1]>0){
                    //Dive left
                    auto x = std::make_unique<DiveCommand>();
                    x->direction[0] = 0;
                    x->direction[1] = 1;

                    std::cerr << "DIVE LEFT!" << std::endl;

                    emit(std::move(x));
                } else {
                    //Dive right
                    auto x = std::make_unique<DiveCommand>();
                    x->direction[0] = 0;
                    x->direction[1] = -1;

                    std::cerr << "DIVE RIGHT!" << std::endl;

                    emit(std::move(x));
                }
            }
        });

    }

}
}
}

