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

#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/input/Sensors.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/support/Configuration.h"


namespace modules {
    namespace behaviour {
        namespace strategy {

		using messages::localisation::Goal;
		using messages::localisation::Ball;
		using messages::localisation::Self;
		using messages::behaviour::LookAtAngle;
		using messages::behaviour::LookAtPoint;
		using messages::behaviour::LookAtPosition;
		using messages::behaviour::HeadBehaviourConfig;
		using messages::input::Sensors;
		using messages::platform::darwin::DarwinSensors;
		using messages::support::Configuration;

		SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			bool penalisedButtonStatus = false, feetOnGround = true, isKicking = false, isWalking = false;
			const messages::motion::KickCommand& kickData;
			const messages::motion::WalkCommand& walkData;

			on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
				MAX_BALL_DISTANCE = config["MAX_BALL_DISTANCE"].as<float>();
				KICK_DISTANCE_THRESHOLD = config["KICK_DISTANCE_THRESHOLD"].as<float>();
				BALL_CERTAINTY_THRESHOLD = config["BALL_CERTAINTY_THRESHOLD"].as<float>();
				IS_GOALIE = (config["GOALIE"].as<int>() == 1);
				START_POSITION = config["START_POSITION"].as<arma::vec2>();
				MY_ZONE = config["ZONE"].as<std::vector<arma::vec2>>();
			});

			// Last 10 to do some button debouncing.
			on<Trigger<Last<10, messages::platform::darwin::DarwinSensors>>>([this](const messages::platform::darwin::DarwinSensors& sensors) {
				penalisedButtonStatus = (senosors.buttons.middle == true) ? !penalisedButtonStatus : penalisedButtonStatus;
			});

			// Check to see if both feet are on the ground.
			on<Trigger<messages::input::Sensors>>([this](const messages::input::Sensors& sensors) {
				feetOnGround = (sensors.leftFootDown && sensors.rightFootDown);
			});

			// Check to see if we are about to kick.
			on<Trigger<messages::motion::KickCommand>>([this](const messages::motion::KickCommand& kick) {
				isKicking = true;
				kickData = kick;
			});

			// Check to see if the kick has finished.
			on<Trigger<messages::motion::KickFinished>>([this](const messages::motion::KickFinished& kick) {
				isKicking = false;
			});

			// Check to see if we are walking.
			on<Trigger<messages::motion::WalkStartCommand>, With<messages::motion::WalkCommand>>
				([this](const messages::motion::WalkStartCommand& walkStart,
					const messages::motion::WalkCommand& walk) {
				isWalking = true;
				walkData = walk;
			});

			// Check to see if we are no longer walking.
			on<Trigger<messages::motion::WalkStopCommand>([this](const messages::motion::WalkStopCommand& walkStop) {
				isWalking = false;
			});

			on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>, 
				With<messages::lcoalisation::Goal>,
				With<messages::localisation::Ball>,
				With<messages::localisation::Self>
				/* Need to add in game controller triggers. */
				/* Need to add team localisation triggers. */>([this](	const time_t&,
											const std::shared_ptr<const messages::localisation::Goal>& goal,
											const std::shared_ptr<const messages::localisation::Ball>& ball,
											const std::shared_ptr<const messages::localisation::Self>& self
											/* Need to add in game controller parameters. */
											/* Need to add in team localisation  parameters. */) {

					// Our ball is almost always the corrent ball to use, unless we dont know where it is.
					Ball& certainBall = ball;

					// Make a copy of the previous state.					
					memcpy(&previousState, &currentState, sizeof(State));

					// Store current position and heading.
					currentState.currentHeading = self.heading;
					currentState.currentPosition = self.position;

					// What state is the game in?
					// Initial?
					/* curretnState.gameStateInitial = gameController[STATE_INITIAL] */;
					// Set?
					/* curretnState.gameStateSet = gameController[STATE_SET] */;
					// Ready?
					/* curretnState.gameStateReady = gameController[STATE_READY] */;
					// Finish?
					/* curretnState.gameStateFinish = gameController[STATE_FINISH] */;
					// Playing?
					/* curretnState.gameStatePlaying = gameController[STATE_PLAYING] */;
					
					// Are we kicking off?
					/* currentState.kickOff = gameController[KICK_OFF] */;

					// Am I the kicker?
					// Does my zone contain the centre field point?
					currentState.kicker = pointInPolygon(MY_ZONE, zeros<arma::vec>(2)); 

					// Have I been picked up?
					currentState.pickedUp = !feetOnGround;

					// Am I penalised?
					currentStatus.penalised = penalisedButtonStatus /* || gameController[PENALISED] */;

					// Was I just put down?
					currentState.putDown = feetOnGround && previousState.pickedUp;
					
					// Did I just become unpenalised?
					currentState.unPenalised = !penalised && previousState.penalised;

					// Am I in my zone?
					currentState.selfInZone = pointInPolygon(MY_ZONE, self.position);
							
					// Can I see the ball?
					currentState.ballSeen = ((ball.sr_xx < BALL_CERTAINTY_THRESHOLD) && (ball.sr_xy < BALL_CERTAINTY_THRESHOLD) && (ball.sr_yy < BALL_CERTAINTY_THREHOLD)));

					// Can anyone else see the ball?
					/* currentState.teamBallSeen = ((teamBall.sr_xx < BALL_CERTAINTY_THRESHOLD) && (teamBall.sr_xy < BALL_CERTAINTY_THRESHOLD) && (teamBall.sr_yy < BALL_CERTAINTY_THREHOLD))); */

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
					currentState.ballInZone = (!currentState.ballLost && pointInPolygon(MY_ZONE, certainBall.position));
							
					// Are the goals in range?
					// x = position[0]?
					// x = 0 = centre field.
					// Assumption: We could potentially kick a goal if we are in the other half of the field).
					// Assumption: goal.position is the x, y coordinates of the goals relative to us.
					currentState.goalInRange = (!currentState.ballLost && (arma::norm(certainBall.position, 2) < MAX_BALL_DISTANCE) && (certainBall.position[0] > 0)));

					// Am I in position to kick the ball?
					currentState.kickPosition = (currentState.currentPosition == previousSate.targetPosition) &&
									(currentState.currentHeading == previousState.targetHeading) &&
									(arma::norm(ball.position, 2) < KICK_DISTANCE_THRESHOLD);
					
					// Is the ball heading in my direction?
					currentState.ballApproaching = !currentState.ballLost && (arma::dot(-certainBall.position, certainBall.velocity) > 0);
				
					// Is the ball approaching our goals?
					currentState.ballAppraochingGoal = ;

					if (currentState.gameStateInitial || currentState.gameStateSet || currentState.gameStateFinished || currentState.penalised || currentState.isPickedUp) {
						stopMoving();
		
						emit("Standing still.");
					}
	
					else if (!currentState.selfInZone) {
						goToPoint(polygonCentroid(MY_ZONE));

						emit("I am not where I should be. Going there now.");
					}

					else if (currentState.gameStateReady) {
						goToPoint(START_POSITION);

						emit("Game is about to start. I should be in my starting position.");
					}

					else if (isKicking) {
						watchBall(certainBall);

						emit("I be looking at what I am kicking.");
					}

					else if (currentState.unPenalised || currentState.putDown) {
						findSelf();

						emit("I don't know where I am. Beginning s spiritual journey to find myself.");
					}


					else if (currentState.ballLost) {
						std::vector<messages::localisation::Ball> hints = {ball, teamBall};

						findBall(hints);

						emit("Don't know where the ball is. Looking for it.");
					}

					else if (currentState.goalInRange || currentState.ballInZone || currentState.ballApproaching) {
						approachBall(certainBall);

						emit("Walking to ball.");
					}

					else if (currentState.kickPosition && !isKicking) {
						kickBall(certainBall);
	
						emit("In kicking position. Kicking ball.");
					}

					else {
						std::vector<messages::localisation::Ball> hints = {ball, teamBall};

						findSelf();
						findBall(hints);
	
						emit("Unknown behavioural state. Finding self, finding ball.");
					}

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

			void SoccerStrategy::stopMoving() {
			}

			void SoccerStrategy::findSelf() {
			}

			void SoccerStrategy::findBall(const std::vector<messages::localisation::Ball>& hints) {
			}

			void SoccerStrategy::goToPoint(const arma::vec2& point) {
			}

			void SoccerStrategy::watchBall(const messages::localisation::Ball& ball) {
			}

			void SoccerStrategy::approachBall(const messages::localisation::Ball& ball) {
			}

			void SoccerStrategy::kickBall(const messages::localisation::Ball& ball) {
			}
		}  // strategy
	}  // behaviours
}  // modules
