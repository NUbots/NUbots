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
#include "utility/time/time.h"
#include "messages/input/ServoID.h"
#include "utility/motion/InverseKinematics.h"
#include "messages/support/FieldDescription.h"

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
			        BALL_UNCERNTAINTY_THRESHOLD = config["BALL_UNCERTAINTY_THRESHOLD"].as<float>();	
                });

		on<Trigger<LookAtBallStart>>([this](const LookAtBallStart&) {
			if (!handle.enabled()) {
std::cerr << "LookAtBall - enabled" << std::endl;
				handle.enable();
			}
		});


		on<Trigger<LookAtBallStop>>([this](const LookAtBallStop&) {
			if (handle.enabled()) {
std::cerr << "LookAtBall - disabled" << std::endl;
				handle.disable();
			}
		});

                //this reaction focuses on the ball - pan'n'scan if not visible and focus on as many objects as possible if visible
		handle = on<Trigger<std::vector<Ball>>,
			With<std::vector<Goal>>,
			With<Sensors>,
			With<Optional<messages::support::FieldDescription>>,
			With<Optional<std::vector<messages::localisation::Self>>>,
			With<Optional<messages::localisation::Ball>>
				>([this](const std::vector<Ball>& balls,
					const std::vector<Goal>& goals,
					const Sensors& sensors,
					const std::shared_ptr<const messages::support::FieldDescription>& fieldDesc,
					const std::shared_ptr<const std::vector<messages::localisation::Self>>& selfs,
					const std::shared_ptr<const messages::localisation::Ball>& ball) {

			// TODO If not doing Josiah's fancy code, remove optional withs
            (void)fieldDesc;
            (void)selfs;
            (void)ball;

            const bool ballIsLost = utility::time::TimeDifferenceSeconds(NUClear::clock::now(),timeLastSeen) > BALL_SEARCH_TIMEOUT_MILLISECONDS;
//            const bool ballIsUncertain = ((ball->sr_xx > BALL_UNCERNTAINTY_THRESHOLD) || (ball->sr_yy > BALL_UNCERNTAINTY_THRESHOLD));
            ++framesSinceSeen;
            
            //XXX: assume these values are correct for global angle conversion
            arma::vec2 cameraAngle = arma::vec2({sensors.servos[size_t(messages::input::ServoID::HEAD_YAW)].presentPosition,
                                                sensors.servos[size_t(messages::input::ServoID::HEAD_PITCH)].presentPosition});
            
            
            //if balls are seen, then place those and everything else that's useful into the look at list
			if (balls.size() > 0) {
			    framesSinceSeen = 0;
				timeLastSeen = NUClear::clock::now();
				std::vector<LookAtAngle> angles;
				angles.reserve(4);

				angles.emplace_back(LookAtAngle {balls[0].screenAngular[0], -balls[0].screenAngular[1]});
                
                //look for either visible or invisible goals
                if (goals.size() > 0) {
				    for (const auto& g : goals) {
					    angles.emplace_back(LookAtAngle {g.screenAngular[0], -g.screenAngular[1]});
				    }
				}
/*				} else if (fieldDesc != NULL && selfs != NULL) {
				    arma::vec2 screenAngular;
				    std::vector<arma::vec2> fieldGoalPos;
				    
				    
                    arma::vec normed_heading = arma::normalise(selfs->front().heading);
                    arma::mat robotToWorldRotation;
                    robotToWorldRotation << normed_heading[0] << -normed_heading[1] << arma::endr
                                         << normed_heading[1] <<  normed_heading[0];
				    
				    fieldGoalPos.push_back(robotToWorldRotation.t() * (fieldDesc->goalpost_bl - selfs->front().position));
				    fieldGoalPos.push_back(robotToWorldRotation.t() * (fieldDesc->goalpost_br - selfs->front().position));
				    fieldGoalPos.push_back(robotToWorldRotation.t() * (fieldDesc->goalpost_yl - selfs->front().position));
				    fieldGoalPos.push_back(robotToWorldRotation.t() * (fieldDesc->goalpost_yr - selfs->front().position));
				    
				    for (const auto& g : fieldGoalPos) {
				        //convert to local angle from global angle
				        screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt(
			                                                {g[0], g[1], 0}, 
			                                                sensors.orientationCamToGround, 
			                                                sensors.orientationBodyToGround) - cameraAngle;
			            //XXX: change to use the camera description to filter these
			            if (std::abs(balls[0].screenAngular[0] - screenAngular[0]) < M_PI/3.0 &&
			                std::abs(-balls[0].screenAngular[0] - screenAngular[1]) < M_PI/4.5) {
                            angles.emplace_back(LookAtAngle {screenAngular[0], screenAngular[1]});
                        }
			        }
				}
*/
				emit(std::make_unique<std::vector<LookAtAngle>>(angles));
			} else if (BALL_SEEN_COUNT_THRESHOLD > framesSinceSeen) {
			    //don't activate other options because we could still have the ball on screen
			}
			// if the ball isn't seen this frame, but we're certain of where it is, look there
//		else if ((ball != NULL) && !ballIsUncertain) {
//		    std::vector<LookAtAngle> angles;
//		    arma::vec2 screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt(
//		                                            {ball->position[0], ball->position[1], 0}, 
//		                                            sensors.orientationCamToGround, 
//		                                            sensors.orientationBodyToGround);
//		    
//			angles.emplace_back(LookAtAngle {screenAngular[0]-cameraAngle[0], screenAngular[1]-cameraAngle[1]}) ;
//		    emit(std::make_unique<std::vector<LookAtAngle>>(angles));
//    }
            //if the ball is lost, do a scan using uncertainties to try to find it
//	else if (ball != NULL and not ballIsLost and ballIsUncertain) {
//		double xFactor = X_FACTOR * std::sqrt(ball->sr_xx);
//		double yFactor = Y_FACTOR * std::sqrt(ball->sr_yy);
//
//		std::vector<LookAtPosition> angles;
//		arma::vec2 screenAngular;
//		
//		//load the offset path from where we think the ball is
//		arma::mat offsets(5,3);
//		offsets  << 0.0 << 0.0 << 0.0 << arma::endr
//         << -xFactor << 0.0 << 0.0 << arma::endr
//         << 0.0 << yFactor << 0.0 << arma::endr
//         << xFactor << 0.0 << 0.0 << arma::endr
//         << 0.0 << -yFactor << 0.0;
//		
//		angles.reserve(5);
//		arma::vec ballPos = arma::vec({ball->position[0], ball->position[1], 0.0});
//		for (size_t i = 0; i < offsets.n_rows; ++i) {
//		    arma::vec2 screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt(
//		                                        ballPos + offsets.row(i).t(),
//		                                        sensors.orientationCamToGround,
//		                                        sensors.orientationBodyToGround);
//		    angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});
//		}
//
//		emit(std::make_unique<std::vector<LookAtPosition>>(angles));
//	} 

			else if(ballIsLost){
				//do a blind scan'n'pan
				std::vector<LookAtPosition> angles;

				angles.emplace_back(LookAtPosition {-M_PI / 2, -M_PI / 4});
				angles.emplace_back(LookAtPosition {0, -M_PI / 4});
				angles.emplace_back(LookAtPosition {M_PI / 2, -M_PI / 4});
				angles.emplace_back(LookAtPosition {M_PI / 2, 0});
				angles.emplace_back(LookAtPosition {0, 0});
				angles.emplace_back(LookAtPosition {-M_PI / 2, 0});
				angles.emplace_back(LookAtPosition {-M_PI / 2, M_PI / 4});
				angles.emplace_back(LookAtPosition {0, M_PI / 4});
				angles.emplace_back(LookAtPosition {M_PI / 2, M_PI / 4});

				angles.emplace_back(LookAtPosition {M_PI / 2, -M_PI / 4});
				angles.emplace_back(LookAtPosition {0, -M_PI / 4});
				angles.emplace_back(LookAtPosition {-M_PI / 2, -M_PI / 4});
				angles.emplace_back(LookAtPosition {-M_PI / 2, 0});
				angles.emplace_back(LookAtPosition {0, 0});
				angles.emplace_back(LookAtPosition {M_PI / 2, 0});
				angles.emplace_back(LookAtPosition {M_PI / 2, M_PI / 4});
				angles.emplace_back(LookAtPosition {0, M_PI / 4});
				angles.emplace_back(LookAtPosition {-M_PI / 2, M_PI / 4});
//			const double scanYaw = 1.5;
//			const double scanPitch = 0.8;
//
//			const size_t panPoints = 4;
//
//			for (size_t i = 0; i < panPoints+1; ++i) {
//				angles.emplace_back(LookAtPosition {-(i * scanYaw / panPoints - scanYaw / 2.0), -scanPitch * ((i + 1) % 2) + scanPitch});
//			}
//
//			for (size_t i = 0; i < panPoints+1; ++i) {
//				angles.emplace_back(LookAtPosition {i * scanYaw / panPoints-scanYaw / 2.0, -scanPitch * (i % 2) + scanPitch});
//			}
//
				emit(std::make_unique<std::vector<LookAtPosition>>(angles));
			}

                });
            }
        }  // planning
    }  // behaviours
}  // modules
