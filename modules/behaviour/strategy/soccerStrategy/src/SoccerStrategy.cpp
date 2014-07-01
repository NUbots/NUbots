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

#include "SoccerStrategy.h"

#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"


namespace modules {
    namespace behaviour {
        namespace strategy {

            using messages::vision::Ball;
            using messages::vision::Goal;
            using messages::behaviour::LookAtAngle;
            using messages::behaviour::LookAtPoint;
            using messages::behaviour::LookAtPosition;
            using messages::behaviour::HeadBehaviourConfig;
            using messages::input::Sensors;
            using messages::support::Configuration;

            SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

				
                on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
                    BALL_SEARCH_TIMEOUT_MILLISECONDS = config["BALL_SEARCH_TIMEOUT_MILLISECONDS"].as<float>();
                });

                //this reaction focuses on the ball - pan'n'scan if not visible and focus on as many objects as possible if visible
				on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>>([this](const time_t&) {
				


//                        emit(std::make_unique<std::vector<LookAtPosition>>(angles));
                    

                });
            }
        }  // planning
    }  // behaviours
}  // modules
