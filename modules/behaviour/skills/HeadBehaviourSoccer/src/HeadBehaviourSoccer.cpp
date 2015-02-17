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

#include "HeadBehaviourSoccer.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObects.h"
#include "messages/input/Sensors.h"

namespace modules {
    namespace motion {

        using messages::vision::Goal;
        using messages::vision::Ball;
        using messages::localisation::Ball;
        using messages::localisation::Self;
        using messages::input::Sensors;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<HeadBehaviourSoccer>>>([this] (const Configuration<HeadBehaviourSoccer>& config)
                {
                    //Gains                    
                    //head_gain = config["head_gain"].as<double>();
                    
                });

                on<Every<30, Per<std::chrono::seconds>>,
                    With<Sensors>,
                    With<Optional<std::vector<Ball>>>,
                    With<Optional<std::vector<Goal>>>
                  >([this] (const Sensors& sensors, const HeadCommand& command) {

                    if(fixatedOnBall){
                    
                    } else if(fixatedOnGoal){

                    } else if(panning){

                    }

                    //Emit result
                    emit(std::make_unique<HeadCommand>());
                });

              
            }

    }  // motion
}  // modules
