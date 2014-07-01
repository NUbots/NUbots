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
		bool penalisedButtonStatus = false, feetOnGround = true;
		bool selfInZone, ballInZone, goalInRange;
	
                on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
			MAX_BALL_DISTANCE = config["MAX_BALL_DISTANCE"].as<int>();
			MY_ZONE = config["ZONE"].as<std::vector<arma::vec>>();
                });

		on<Trigger<Last<10, messages::platform::darwin::DarwinSensors>>>([this](const messages::platform::darwin::DarwinSensors& sensors) {
			penalisedButtonStatus = (senosors.buttons.middle == true) ? !penalisedButtonStatus : penalisedButtonStatus;
		});

		on<Trigger<messages::input::Sensors>>([this](const messages::input::Sensors& sensors) {
			feetOnGround = (sensors.leftFootDown && sensors.rightFootDown);
		});

		on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>, 
                    With<std::vector<Goal>>,
                    With<messages::localisation::Ball>,
                    With<messages::localisation::Self>,
                    >([this](
				const time_t&,
				const std::shared_ptr<const messages::localisation::Goal>& goal,
				const std::shared_ptr<const messages::localisation::Ball>& ball,
				const std::shared_ptr<const messages::localisation::Self>& self) {
				
				// Am I in my zone?
				selfInZone = pointInPolygon(myZone, self.position);
						
				// Is the ball in my zone?
				ballInZone = pointInPolygon(myZone, ball.position);
						
				// Are the goals in range?
				// x = position[0]?
				// x = 0 = centre field.
				// Assumption: We could potentially kick a goal if we are in the other half of the field).
				// Assumption: goal.position is the x, y coordinates of the goals relative to us.
				goalInRange = (arma::norm(self.position - ball.position, 2) < MAX_BALL_DISTANCE) && (ball.position[0] > 0));


				// Approach ball Code (given by Josiah - need to update)
/*
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::Ball;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->walkMovementType = WalkApproach::ApproachFromDirection;
				approach->heading = arma::vec({-3,0});
				approach->target = arma::vec({0,0});
				emit(std::move(approach));
*/
                });
            }
        }  // planning
    }  // behaviours
}  // modules
