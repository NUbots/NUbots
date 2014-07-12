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

#include "LookAtBall.h"

#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"

#include "utility/motion/InverseKinematics.h"

namespace modules {
    namespace behaviour {
        namespace planning {

            using messages::vision::Ball;
            using messages::vision::Goal;
            using messages::behaviour::LookAtBallStart;
            using messages::behaviour::LookAtBallStop;
            using messages::behaviour::LookAtAngle;
            using messages::behaviour::LookAtPoint;
            using messages::behaviour::LookAtPosition;
            using messages::behaviour::HeadBehaviourConfig;
            using messages::input::Sensors;
            using messages::support::Configuration;

            LookAtBall::LookAtBall(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

                on<Trigger<Configuration<HeadBehaviourConfig>>>([this](const Configuration<HeadBehaviourConfig>& config) {
			BALL_SEARCH_TIMEOUT_MILLISECONDS = config["BALL_SEARCH_TIMEOUT_MILLISECONDS"].as<float>();
			X_FACTOR = config["X_FACTOR"].as<float>();	
			Y_FACTOR = config["Y_FACTOR"].as<float>();	
                });

		on<Trigger<LookAtBallStart>>([this](const LookAtBallStart&) {
			if (!handle.enabled()) {
				handle.enable();
			}
		});


		on<Trigger<LookAtBallStop>>([this](const LookAtBallStop&) {
			if (handle.enabled()) {
				handle.disable();
			}
		});

                //this reaction focuses on the ball - pan'n'scan if not visible and focus on as many objects as possible if visible
		handle = on<Trigger<std::vector<Ball>>,
			With<std::vector<Goal>>,
			With<Sensors>,
			With<Optional<messages::localisation::Ball>>
				>([this](const std::vector<Ball>& balls,
					const std::vector<Goal>& goals,
					const Sensors& sensors,
					const std::shared_ptr<const messages::localisation::Ball>& ball) {

			if (balls.size() > 0) {
				timeSinceLastSeen = sensors.timestamp;
				std::vector<LookAtAngle> angles;
				angles.reserve(4);

				angles.emplace_back(LookAtAngle {balls[0].screenAngular[0], -balls[0].screenAngular[1]});

				for (const auto& g : goals) {
					angles.emplace_back(LookAtAngle {g.screenAngular[0], -g.screenAngular[1]});
				}

				emit(std::make_unique<std::vector<LookAtAngle>>(angles));
			} 

			else if (ball != NULL) {
				double xFactor = X_FACTOR * std::sqrt(ball->sr_xx);
				double yFactor = Y_FACTOR * std::sqrt(ball->sr_yy);
	
				std::vector<LookAtAngle> angles;
				arma::vec2 screenAngular;
				angles.reserve(10);

				screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
				angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});

				screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0] + xFactor, ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
				angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});

				screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0] - xFactor, ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
				angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});

				screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1] + yFactor, 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
				angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});

				screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1] - yFactor, 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
				angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});

				for (const auto& g : goals) {
					angles.emplace_back(LookAtAngle {g.screenAngular[0], -g.screenAngular[1]});
				}
	
				emit(std::make_unique<std::vector<LookAtAngle>>(angles));
			} 

			else if(std::chrono::duration<float, std::ratio<1,1000>>(sensors.timestamp - timeSinceLastSeen).count() > BALL_SEARCH_TIMEOUT_MILLISECONDS){
				//do a blind scan'n'pan
				//XXX: this needs to be a look at sequence rather than a look at point
				std::vector<LookAtPosition> angles;

				const double scanYaw = 1.5;
				const double scanPitch = 0.8;

				const size_t panPoints = 4;

				for (size_t i = 0; i < panPoints+1; ++i) {
					angles.emplace_back(LookAtPosition {i * scanYaw / panPoints-scanYaw / 2.0, -scanPitch * (i % 2) + scanPitch});
				}

				for (size_t i = 0; i < panPoints+1; ++i) {
					angles.emplace_back(LookAtPosition {-(i * scanYaw / panPoints - scanYaw / 2.0), -scanPitch * ((i + 1) % 2) + scanPitch});
				}

				emit(std::make_unique<std::vector<LookAtPosition>>(angles));
			}

                });
            }
        }  // planning
    }  // behaviours
}  // modules
