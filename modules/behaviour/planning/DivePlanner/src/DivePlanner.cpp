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

        on<Trigger<Configuration<DivePlanner> > >([this](const Configuration<DivePlanner>& config) {
        	SPEED_THRESHOLD = config["SPEED_THRESHOLD"].as<float>();
        	DISTANCE_THRESHOLD = config["DISTANCE_THRESHOLD"].as<float>();
        });

        on<Trigger<Ball>, With<std::vector<Self>>, With<std::vector<messages::vision::Ball>>, With<DivePlan>>([this] (
        	const Ball& ball,
        	const std::vector<Self>& selfs,
        	const std::vector<messages::vision::Ball>& vision_balls,
        	const DivePlan& divePlan) {

        	// TODO: why are these used?
        	(void)selfs;
        	(void)divePlan;

            if(vision_balls.size()>0 &&
               ball.position[0] > 0 &&
               -ball.velocity[0] > SPEED_THRESHOLD &&
               ball.position[0] < DISTANCE_THRESHOLD ){

                //NUClear::log("Ball Vel:", -ball.velocity[0] , ball.position[0]);

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

