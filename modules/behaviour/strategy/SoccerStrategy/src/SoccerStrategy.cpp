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

#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/WalkStrategy.h"
#include "messages/input/Sensors.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/support/Configuration.h"
#include "messages/motion/KickCommand.h"
#include "messages/motion/WalkCommand.h"

#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/support/armayamlconversions.h"

namespace modules {
    namespace behaviour {
        namespace strategy {

		using messages::localisation::Ball;
		using messages::localisation::Self;
		using messages::behaviour::LookAtAngle;
		using messages::behaviour::LookAtPoint;
		using messages::behaviour::LookAtPosition;
		using messages::behaviour::HeadBehaviourConfig;
		using messages::input::Sensors;
		using messages::platform::darwin::DarwinSensors;
		using messages::support::Configuration;
		using messages::support::FieldDescription;
		using messages::motion::KickCommand;
		using messages::motion::WalkCommand;
		using messages::behaviour::WalkStrategy;
		using messages::behaviour::WalkTarget;
		using messages::behaviour::WalkApproach;

		using utility::math::geometry::Plane;
		using utility::math::geometry::ParametricLine;

		SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			bool penalisedButtonStatus = false, feetOnGround = true, isKicking = false, isWalking = false;
			KickCommand kickData;
			WalkCommand walkData;

			on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
				MAX_BALL_DISTANCE = config["MAX_BALL_DISTANCE"].as<float>();
				KICK_DISTANCE_THRESHOLD = config["KICK_DISTANCE_THRESHOLD"].as<float>();
				BALL_CERTAINTY_THRESHOLD = config["BALL_CERTAINTY_THRESHOLD"].as<float>();
				IS_GOALIE = (config["GOALIE"].as<int>() == 1);
				START_POSITION = config["START_POSITION"].as<arma::vec2>();
				MY_ZONE = config["ZONE"].as<std::vector<arma::vec2>>();
			});

			// Last 10 to do some button debouncing.
			on<Trigger<Last<10, messages::platform::darwin::DarwinSensors>>>([this, &penalisedButtonStatus](const std::vector<std::shared_ptr<const messages::platform::darwin::DarwinSensors>>& sensors) {
				int trueCount = 0;

				for (int i = 0; i < sensors.size(); i++) {
					if (sensors.at(i)->buttons.middle) {
						trueCount++;
					}
				}

				penalisedButtonStatus = ((trueCount / 10) >= 0.75) ? !penalisedButtonStatus : penalisedButtonStatus;
			});

			// Check to see if both feet are on the ground.
			on<Trigger<messages::input::Sensors>>([this, &feetOnGround](const messages::input::Sensors& sensors) {
				feetOnGround = (sensors.leftFootDown && sensors.rightFootDown);
			});

			// Check to see if we are about to kick.
			on<Trigger<messages::motion::KickCommand>>([this, &isKicking, &kickData](const messages::motion::KickCommand& kick) {
				isKicking = true;
				kickData = kick;
			});

			// Check to see if the kick has finished.
			on<Trigger<messages::motion::KickFinished>>([this, &isKicking](const messages::motion::KickFinished& kick) {
				isKicking = false;
			});

			// Check to see if we are walking.
			on<Trigger<messages::motion::WalkStartCommand>, With<messages::motion::WalkCommand>>
				([this, &isWalking, &walkData](const messages::motion::WalkStartCommand& walkStart,
					const messages::motion::WalkCommand& walk) {
				isWalking = true;
				walkData = walk;
			});

			// Check to see if we are no longer walking.
			on<Trigger<messages::motion::WalkStopCommand>>([this, &isWalking](const messages::motion::WalkStopCommand& walkStop) {
				isWalking = false;
			});

			// Get the field description.
			on<Trigger<messages::support::FieldDescription>>([this](const messages::support::FieldDescription& field) {
				FIELD_DESCRIPTION = field;
			});

			on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>, 
				With<messages::localisation::Ball>,
				With<messages::localisation::Self>
				/* Need to add in game controller triggers. */
				/* Need to add team localisation triggers. */>([this, &penalisedButtonStatus, &feetOnGround, &isKicking, &isWalking, &kickData, &walkData](
											const time_t&,
											const messages::localisation::Ball& ball,
											const messages::localisation::Self& self
											/* Need to add in game controller parameters. */
											/* Need to add in team localisation  parameters. */) {

					// Our ball is almost always the corrent ball to use, unless we dont know where it is.
					auto certainBall = ball;

					// Make a copy of the previous state.					
					memcpy(&previousState, &currentState, sizeof(State));

					// Store current position and heading.
					currentState.currentTransform = self.robot_to_world_rotation;
					currentState.currentPosition = self.position;
					currentState.currentHeading = atan2(currentState.currentTransform(1, 0), currentState.currentTransform(0, 0));

					// What state is the game in?
					// Initial?
					/* currentState.gameStateInitial = gameController[STATE_INITIAL] */;
					// Set?
					/* currentState.gameStateSet = gameController[STATE_SET] */;
					// Ready?
					/* currentState.gameStateReady = gameController[STATE_READY] */;
					// Finish?
					/* currentState.gameStateFinish = gameController[STATE_FINISH] */;
					// Playing?
					/* currentState.gameStatePlaying = gameController[STATE_PLAYING] */;
					
					// Are we kicking off?
					/* currentState.kickOff = gameController[KICK_OFF] */;

					// Am I the kicker?
					// Does my zone contain the centre field point?
					/* currentState.kicker = pointInPolygon(MY_ZONE, arma::zeros<arma::vec>(2)); */

					// Have I been picked up?
					currentState.pickedUp = !feetOnGround;

					// Am I penalised?
					currentState.penalised = penalisedButtonStatus /* || gameController[PENALISED] */;

					// Was I just put down?
					currentState.putDown = feetOnGround && previousState.pickedUp;
					
					// Did I just become unpenalised?
					currentState.unPenalised = !currentState.penalised && previousState.penalised;

					// Am I in my zone?
					/* currentState.selfInZone = pointInPolygon(MY_ZONE, self.position); */
							
					// Can I see the ball?
					currentState.ballSeen = ((ball.sr_xx < BALL_CERTAINTY_THRESHOLD) && (ball.sr_xy < BALL_CERTAINTY_THRESHOLD) && (ball.sr_yy < BALL_CERTAINTY_THRESHOLD));

					// Can anyone else see the ball?
					/* currentState.teamBallSeen = ((teamBall.sr_xx < BALL_CERTAINTY_THRESHOLD) && (teamBall.sr_xy < BALL_CERTAINTY_THRESHOLD) && (teamBall.sr_yy < BALL_CERTAINTY_THREHOLD)); */

					// Is the ball lost?
					currentState.ballLost = !currentState.ballSeen && !currentState.teamBallSeen;

					// Select the best ball to use (our ball or the teams ball).
					// We could be a little more sophisticated here and consider how certain we are about the 
					// balls location.
					if (!currentState.ballLost && currentState.ballSeen) {
						certainBall = ball;
					}

					else if (!currentState.ballLost && currentState.teamBallSeen)  {
						/* certainBall = teamBall */;
					}

					// Is the ball in my zone?
					currentState.ballInZone = (!currentState.ballLost /* && pointInPolygon(MY_ZONE, certainBall->position) */);
							
					// Are the goals in range?
					// x = position[0]?
					// x = 0 = centre field.
					// Assumption: We could potentially kick a goal if we are in the other half of the field.
					// Assumption: goal.position is the x, y coordinates of the goals relative to us.
					currentState.goalInRange = (!currentState.ballLost && (arma::norm(certainBall.position, 2) < MAX_BALL_DISTANCE) && (certainBall.position[0] > 0));

					// Am I in position to kick the ball?
					bool kickThreshold = arma::norm(ball.position, 2) < KICK_DISTANCE_THRESHOLD;
					bool inPosition = (currentState.currentPosition[0] == previousState.targetPosition[0]) && (currentState.currentPosition[1] == previousState.targetPosition[1]);
					bool correctHeading = (currentState.currentHeading[0] == previousState.targetHeading[0]) && (currentState.currentHeading[1] == previousState.targetHeading[1]);
					currentState.kickPosition = (inPosition && correctHeading && kickThreshold);
					
					// Is the ball heading in my direction?
					currentState.ballApproaching = !currentState.ballLost && (arma::dot(-certainBall.position, certainBall.velocity) > 0);
				
					// Is the ball approaching our goals?
					Plane<2> plane;
					ParametricLine<2> line;
					arma::vec2 xaxis = {1, 0};
					arma::vec2 fieldWidth = {-FIELD_DESCRIPTION.dimensions.field_length / 2, 0};

					plane.setFromNormal(xaxis, fieldWidth);
					line.setFromDirection((currentState.currentTransform * certainBall.velocity), (currentState.currentTransform * certainBall.position + currentState.currentPosition));

					auto intersection = plane.intersect(line);

					if (intersection.first) {
						currentState.ballApproachingGoal = ((intersection.second[1] <= (FIELD_DESCRIPTION.dimensions.goal_area_width / 2)) && (intersection.second[1] >= -(FIELD_DESCRIPTION.dimensions.goal_area_width / 2)));

						currentState.ballGoalIntersection = intersection.second;
					}	



					// Determine current state and appropriate action(s).
					if (currentState.gameStateInitial || currentState.gameStateSet || currentState.gameStateFinished || currentState.penalised || currentState.pickedUp) {
						stopMoving();
		
						NUClear::log<NUClear::INFO>("Standing still.");
					}
	
					else if (!currentState.selfInZone) {
						/* goToPoint(polygonCentroid(MY_ZONE)); */

						NUClear::log<NUClear::INFO>("I am not where I should be. Going there now.");
					}

					else if (currentState.gameStateReady) {
						goToPoint(START_POSITION);

						NUClear::log<NUClear::INFO>("Game is about to start. I should be in my starting position.");
					}

					else if (isKicking) {
						watchBall(certainBall);

						NUClear::log<NUClear::INFO>("I be looking at what I am kicking.");
					}

					else if (currentState.unPenalised || currentState.putDown) {
						findSelf();

						NUClear::log<NUClear::INFO>("I don't know where I am. Beginning s spiritual journey to find myself.");
					}


					else if (currentState.ballLost) {
						std::vector<messages::localisation::Ball> hints = {ball/*, teamBall*/};

						findBall(hints);

						NUClear::log<NUClear::INFO>("Don't know where the ball is. Looking for it.");
					}

					else if (currentState.goalInRange || currentState.ballInZone || currentState.ballApproaching) {
						approachBall(certainBall, self);

						NUClear::log<NUClear::INFO>("Walking to ball.");
					}

					else if (currentState.kickPosition && !isKicking) {
						kickBall(certainBall);
	
						NUClear::log<NUClear::INFO>("In kicking position. Kicking ball.");
					}

					else if (IS_GOALIE && currentState.ballApproachingGoal) {
						goToPoint(currentState.ballGoalIntersection);

						NUClear::log<NUClear::INFO>("Ball is approaching goal. Goalie moving to block it.");
					}

					else {
						std::vector<messages::localisation::Ball> hints = {ball/*, teamBall*/};

						findSelf();
						findBall(hints);
	
						NUClear::log<NUClear::INFO>("Unknown behavioural state. Finding self, finding ball.");
					}
				});
			}

			void SoccerStrategy::stopMoving() {
			}

			void SoccerStrategy::findSelf() {
			}

			void SoccerStrategy::findBall(const std::vector<Ball>& hints) {
			}

			void SoccerStrategy::goToPoint(const arma::vec2& point) {
			}

			void SoccerStrategy::watchBall(const Ball& ball) {
			}

			void SoccerStrategy::approachBall(const Ball& ball, const Self& self) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::Ball;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->walkMovementType = WalkApproach::ApproachFromDirection;
				approach->heading = arma::vec({-3,0});
				approach->target = currentState.currentTransform * ball.position + currentState.currentPosition;
				emit(std::move(approach));
			}

			void SoccerStrategy::kickBall(const Ball& ball) {
			}
		}  // strategy
	}  // behaviours
}  // modules
