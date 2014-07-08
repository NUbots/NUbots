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
#include "messages/input/gameevents/GameEvents.h"

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

//		using messages::input::gameevents::GameState;
//		using messages::input::gameevents::KickOffTeam;
//		using messages::input::gameevents::TeamColour;
//		using messages::input::gameevents::BallKickedOut;
//		using messages::input::gameevents::HalfTime;
//		using messages::input::gameevents::CoachMessage;
//		using messages::input::gameevents::Penalisation;
//		using messages::input::gameevents::Unpenalisation;
//		using messages::input::gameevents::GoalScored;
//		using messages::input::gameevents::Score;

		using utility::math::geometry::Polygon;
		using utility::math::geometry::Plane;
		using utility::math::geometry::ParametricLine;

		SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			penalisedButtonStatus = false;
			feetOnGround = true;
			isKicking = false;
			isWalking = false;

			on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
				std::vector<arma::vec2> zone;

				MAX_BALL_DISTANCE = config["MAX_BALL_DISTANCE"].as<float>();
				KICK_DISTANCE_THRESHOLD = config["KICK_DISTANCE_THRESHOLD"].as<float>();
				BALL_CERTAINTY_THRESHOLD = config["BALL_CERTAINTY_THRESHOLD"].as<float>();
				BALL_SELF_INTERSECTION_REGION = config["BALL_SELF_INTERSECTION_REGION"].as<float>();
				BALL_LOOK_ERROR = config["BALL_LOOK_ERROR"].as<arma::vec2>();
				BALL_MOVEMENT_THRESHOLD = config["BALL_MOVEMENT_THRESHOLD"].as<float>();
				GOAL_LOOK_ERROR = config["GOAL_LOOK_ERROR"].as<arma::vec2>();
				IS_GOALIE = (config["GOALIE"].as<int>() == 1);
				START_POSITION = config["START_POSITION"].as<arma::vec2>();
				MY_ZONE = config["MY_ZONE"].as<int>();

				ZONE_DEFAULTS.push_back(config["ZONE_1_DEFAULT"].as<arma::vec2>());
				zone = config["ZONE_1"].as<std::vector<arma::vec2>>();
				ZONES.push_back(Polygon(zone));

				ZONE_DEFAULTS.push_back(config["ZONE_2_DEFAULT"].as<arma::vec2>());
				zone = config["ZONE_2"].as<std::vector<arma::vec2>>();
				ZONES.push_back(Polygon(zone));

				ZONE_DEFAULTS.push_back(config["ZONE_3_DEFAULT"].as<arma::vec2>());
				zone = config["ZONE_3"].as<std::vector<arma::vec2>>();
				ZONES.push_back(Polygon(zone));

				ZONE_DEFAULTS.push_back(config["ZONE_4_DEFAULT"].as<arma::vec2>());
				zone = config["ZONE_4"].as<std::vector<arma::vec2>>();
				ZONES.push_back(Polygon(zone));
			});

			// Last 10 to do some button debouncing.
			on<Trigger<Last<10, messages::platform::darwin::DarwinSensors>>>([this](const std::vector<std::shared_ptr<const messages::platform::darwin::DarwinSensors>>& sensors) {
				float leftCount = 0.0, middleCount = 0.0;

				for (size_t i = 0; i < sensors.size(); i++) {
					if (sensors.at(i)->buttons.left) {
						leftCount += 0.1;
					}
					if (sensors.at(i)->buttons.middle) {
						middleCount += 0.1;
					}
				}

				gameStateButtonStatus = (leftCount > 0.7);
				penalisedButtonStatus = (middleCount > 0.7) ? !penalisedButtonStatus : penalisedButtonStatus;
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
				(void)kick;

				isKicking = false;
			});

			// Check to see if we are walking.
			on<Trigger<messages::motion::WalkStartCommand>, With<messages::motion::WalkCommand>>
				([this](const messages::motion::WalkStartCommand& walkStart,
					const messages::motion::WalkCommand& walk) {
				(void)walkStart;

				isWalking = true;
				walkData = walk;
			});

			// Check to see if we are no longer walking.
			on<Trigger<messages::motion::WalkStopCommand>>([this](const messages::motion::WalkStopCommand& walkStop) {
				(void)walkStop;

				isWalking = false;
			});

			// Get the field description.
			on<Trigger<messages::support::FieldDescription>>([this](const messages::support::FieldDescription& field) {
				FIELD_DESCRIPTION = field;
			});

			on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>, 
				With<messages::localisation::Ball>,
				With<messages::localisation::Self>
//				With<messages::input::gameevents::GameState<>>,
//				With<messages::input::gameevents::KickOffTeam>,
//				With<messages::input::gameevents::TeamColour>,
//				With<messages::input::gameevents::BallKickedOut>,
//				With<messages::input::gameevents::HalfTime>,
//				With<messages::input::gameevents::CoachMessage>,
//				With<messages::input::gameevents::Penalisation>,
//				With<messages::input::gameevents::Unpenalisation>,
//				With<messages::input::gameevents::GoalScored>,
//				With<messages::input::gameevents::Score>
				/* Need to add team localisation triggers. */>([this](
											const time_t&,
											const messages::localisation::Ball& ball,
											const messages::localisation::Self& self
//											const messages::input::gameevents::GameState& gameState,
//											const messages::input::gameevents::KickOffTeam& kickOffTeam,
//											const messages::input::gameevents::TeamColour& colour,
//											const messages::input::gameevents::BallKickedOut& ballOut,
//											const messages::input::gameevents::HalfTime& halfTime,
//											const messages::input::gameevents::CoachMessage message,
//											const messages::input::gameevents::Penalisation& penalisation,
//											const messages::input::gameevents::Unpenalisation& unpenalisation,
//											const messages::input::gameevents::GoalScored& goalScored,
//											const messages::input::gameevents::Score& score
											/* Need to add in team localisation  parameters. */) {

					// Make a copy of the previous state.					
					memcpy(&previousState, &currentState, sizeof(State));

					// Make a copy of the ball.
					memcpy(&currentState.ball, &ball, sizeof(Ball));

					// Store current position and heading.
					currentState.transform = self.robot_to_world_rotation;
					currentState.position = self.position;
					currentState.heading = atan2(currentState.transform(1, 0), currentState.transform(0, 0));

					// Allow the back panel button to cycle through the primary game states.
					if (gameStateButtonStatus) {
						gameStateButtonStatus = false;
						currentState.primaryGameState++;
					}

					// What state is the game in?
					// Initial?
					/* currentState.gameState.Initial = gameController[STATE_INITIAL] */;
					// Set?
					/* currentState.gameState.Set = gameController[STATE_SET] */;
					// Ready?
					/* currentState.gameState.Ready = gameController[STATE_GameStatePrimary::READY] */;
					// Finish?
					/* currentState.gameState.Finish = gameController[STATE_FINISH] */;
					// Playing?
					/* currentState.gameState.Playing = gameController[STATE_PLAYING] */;
					// Penalty kick?
					/* currentState.gameState.penaltyKick = gameController[STATE_PENALTY_KICK] */;
					// Free kick?
					/* currentState.gameState.freeKick = gameController[STATE_FREE_KICK] */;
					// Goal kick?
					/* currentState.gameState.goalKick = gameController[STATE_GOAL_KICK] */;
					// Corner kick?
					/* currentState.gameState.cornerKick = gameController[STATE_CORNER_KICK] */;
					// Throw-In?
					/* currentState.gameState.throwIn = gameController[STATE_THROW_IN] */;
					// Paused?
					/* currentState.gameState.paused = gameController[STATE_PAUSED] */;
					
					// Are we kicking off?
					/* currentState.kickOff = gameController[KICK_OFF] */;

					// Am I the kicker?
					// Is my start position inside the centre circle? 
					currentState.kicker = ((arma::norm(START_POSITION, 2) < (FIELD_DESCRIPTION.dimensions.center_circle_diameter / 2)) && (currentState.primaryGameState == GameStatePrimary::READY || 
								currentState.primaryGameState == GameStatePrimary::SET || currentState.primaryGameState == GameStatePrimary::PLAYING)) || ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK || 
								currentState.secondaryGameState == GameStateSecondary::FREE_KICK || currentState.secondaryGameState == GameStateSecondary::GOAL_KICK || currentState.secondaryGameState == GameStateSecondary::CORNER_KICK) && 
								currentState.primaryGameState == GameStatePrimary::PLAYING /* && currentState.??? */);

					// Have I been picked up?
					currentState.pickedUp = !feetOnGround;

					// Am I penalised?
					currentState.penalised = penalisedButtonStatus /* || gameController[PENALISED] */;

					// Was I just put down?
					currentState.putDown = feetOnGround && previousState.pickedUp;
					
					// Did I just become unpenalised?
					currentState.unPenalised = !currentState.penalised && previousState.penalised;

					// Am I in my zone?
					currentState.selfInZone = ZONES.at(MY_ZONE).pointContained(self.position);
							
					// Can I see the ball?
					currentState.ballSeen = ((currentState.ball.sr_xx < BALL_CERTAINTY_THRESHOLD) && (currentState.ball.sr_xy < BALL_CERTAINTY_THRESHOLD) && (currentState.ball.sr_yy < BALL_CERTAINTY_THRESHOLD));

					// Is the ball lost?
					currentState.ballLost = !currentState.ballSeen;

					// Has the ball moved?
					currentState.ballHasMoved = arma::norm(currentState.ball.position - previousState.ball.position, 2) > BALL_MOVEMENT_THRESHOLD;

					// Is the ball in my zone?
					currentState.ballInZone = (!currentState.ballLost && ZONES.at(MY_ZONE).pointContained(transformPoint(currentState.ball.position) + currentState.position));
							
					// Are the goals in range?
					// x = 0 = centre field.
					// Assumption: We could potentially kick a goal if we are in the other half of the field.
					// Assumption: goal.position is the x, y coordinates of the goals relative to us.
					currentState.goalInRange = (!currentState.ballLost && (arma::norm(currentState.ball.position, 2) < MAX_BALL_DISTANCE) && ((transformPoint(currentState.ball.position) + currentState.position)[0] > 0));

					// Am I in position to kick the ball?
					bool kickThreshold = arma::norm(currentState.ball.position, 2) < KICK_DISTANCE_THRESHOLD;
					bool inPosition = (currentState.position[0] == previousState.targetPosition[0]) && (currentState.position[1] == previousState.targetPosition[1]);
					bool correctHeading = (currentState.heading[0] == previousState.targetHeading[0]) && (currentState.heading[1] == previousState.targetHeading[1]);
					currentState.kickPosition = (inPosition && correctHeading && kickThreshold);
				
					// Make preparations to calculate whether the ball is approaching our own goals or ourselves.
					Plane<2> planeGoal, planeSelf;
					ParametricLine<2> line;
					arma::vec2 xaxis = {1, 0};
					arma::vec2 fieldWidth = {-FIELD_DESCRIPTION.dimensions.field_length / 2, 0};

					planeGoal.setFromNormal(xaxis, fieldWidth);
					planeSelf.setFromNormal(transformPoint(currentState.ball.position), currentState.position);

					line.setFromDirection(transformPoint(currentState.ball.velocity), (transformPoint(currentState.ball.position) + currentState.position));

					// Is the ball approaching our goals?
					try {
						// Throws std::domain_error if there is no intersection.
						currentState.ballGoalIntersection = planeGoal.intersect(line);

						currentState.ballApproachingGoal = arma::norm(fieldWidth - currentState.ballGoalIntersection, 2) <= (FIELD_DESCRIPTION.dimensions.goal_area_width / 2);
					}

					catch (std::domain_error e) {
						currentState.ballApproachingGoal = false;
					}

					
					// Is the ball heading in my direction?
					try {
						// Throws std::domain_error if there is no intersection.
						currentState.ballSelfIntersection = planeSelf.intersect(line);

						currentState.ballApproaching = arma::norm(currentState.position - currentState.ballSelfIntersection, 2) <= (BALL_SELF_INTERSECTION_REGION / 2);
					}

					catch (std::domain_error e) {
						currentState.ballApproaching = false;
					}






					// Calculate the optimal zone position.
					arma::vec2 optimalPosition = findOptimalPosition(ZONES.at(MY_ZONE), transformPoint(currentState.ball.position) + currentState.position);

					// Determine current state and appropriate action(s).
					if ((currentState.primaryGameState == GameStatePrimary::INITIAL) || (currentState.primaryGameState == GameStatePrimary::SET) || (currentState.primaryGameState == GameStatePrimary::FINISHED) || currentState.pickedUp) {
						stopMoving();
		
						NUClear::log<NUClear::INFO>("Standing still.");
					}
	
					else if (currentState.primaryGameState == GameStatePrimary::READY) {
						goToPoint(START_POSITION);

						NUClear::log<NUClear::INFO>("Game is about to start. I should be in my starting position.");
					}

					else if(currentState.penalised && !currentState.pickedUp && !currentState.putDown) {
						stopMoving();

						NUClear::log<NUClear::INFO>("I am penalised and have not been picked up yet. Don't move");
					}

					else if(currentState.penalised && currentState.putDown) {
						findSelf();
						findBall();

						NUClear::log<NUClear::INFO>("I am penalised and have been put down. I must be on the side line somewhere. Where am I?");
					}

					else if (!currentState.selfInZone) {
						goToPoint(optimalPosition);

						NUClear::log<NUClear::INFO>("I am not where I should be. Going there now.");
					}

					else if (currentState.unPenalised) {
						goToPoint(optimalPosition);

						NUClear::log<NUClear::INFO>("I am unpenalised, I should already know where I am and where the ball is. So find the most optimal location in my zone to go to.");
					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && IS_GOALIE && currentState.ballLost) {
						findBall();

						NUClear::log<NUClear::INFO>("Penalty kick in progress. Locating ball.");
					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && IS_GOALIE && !currentState.ballLost && currentState.ballHasMoved && !currentState.ballApproachingGoal) {
						arma::vec2 blockPosition = {currentState.position[0], (transformPoint(currentState.ball.position) + currentState.position)[1]};
						sideStepToPoint(blockPosition);

						NUClear::log<NUClear::INFO>("Penalty kick in progress. Locating ball.");
					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && currentState.ballLost && currentState.kicker) {
						findBall();

						NUClear::log<NUClear::INFO>("Penalty kick in progress. Locating ball.");
					}

					else if (currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK && !currentState.ballLost && currentState.kicker) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						approachBall(goal);

						NUClear::log<NUClear::INFO>("Penalty kick in progress. Approaching ball.");
					}

					else if ((previousState.primaryGameState == GameStatePrimary::SET) && (currentState.primaryGameState == GameStatePrimary::PLAYING) && currentState.kickOff && currentState.kicker) {
						kickBall(arma::normalise(currentState.heading, 2));

						NUClear::log<NUClear::INFO>("Game just started. Time to kick off.");
					}

					else if (isKicking) {
						watchBall();

						NUClear::log<NUClear::INFO>("I be looking at what I be kicking.");
					}


					else if (currentState.ballLost) {
						findBall();

						NUClear::log<NUClear::INFO>("Don't know where the ball is. Looking for it.");
					}

					else if ((currentState.ballInZone || currentState.ballApproaching) && currentState.goalInRange) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						approachBall(goal);

						NUClear::log<NUClear::INFO>("Walking to ball.");
					}

					else if ((currentState.ballInZone || currentState.ballApproaching) && !currentState.goalInRange) {
						arma::vec2 nearestZone = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0}; // Find the optimal point in the nearest zone, reflect the position closer to the enemy goal.
						approachBall(nearestZone);

						NUClear::log<NUClear::INFO>("Walking to ball.");
					}

					else if (currentState.kickPosition && !isKicking) {
						kickBall(arma::normalise(currentState.heading, 2));
	
						NUClear::log<NUClear::INFO>("In kicking position. Kicking ball.");
					}

					else if (currentState.goalInRange && !isKicking) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						approachBall(goal);
						kickBall(arma::normalise(goal, 2));

						NUClear::log<NUClear::INFO>("Kick for goal.");
					}

					else if (IS_GOALIE && currentState.ballApproachingGoal) {
						sideStepToPoint(currentState.ballGoalIntersection);

						NUClear::log<NUClear::INFO>("Ball is approaching goal. Goalie moving to block it.");
					}

					else {
						findSelf();
						findBall();
						goToPoint(ZONE_DEFAULTS.at(MY_ZONE));
	
						NUClear::log<NUClear::INFO>("Unknown behavioural state. Finding self, finding ball, moving to default position.");
					}
				});
			}

			arma::vec2 SoccerStrategy::findOptimalPosition(const Polygon& zone, const arma::vec2& point) {
				return(zone.projectPointToPolygon(point));
			}

			void SoccerStrategy::stopMoving() {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
//				// These four parameters are not important for standing still.
//				approach->targetPositionType = WalkTarget::Robot;
//				approach->targetHeadingType = WalkTarget::Robot;
//				approach->heading = currentState.heading; 
//				approach->target = currentState.position; 
				approach->walkMovementType = WalkApproach::StandStill;
				emit(std::move(approach));
			}

			void SoccerStrategy::findSelf() {
				/* Try to locate both goals. */
				/* Look at closest goal for short period to reset localisation. */
				arma::vec2 goalPosition = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
				arma::vec2 ballPosition = transformPoint(currentState.ball.position) + currentState.position;

				// Look at our goals.
				auto lookAtGoal = std::make_unique<messages::behaviour::LookAtPoint>();
				lookAtGoal->x = -goalPosition[0];
				lookAtGoal->y = goalPosition[1];
				lookAtGoal->xError = GOAL_LOOK_ERROR[0];
				lookAtGoal->yError = GOAL_LOOK_ERROR[1];
				emit(std::move(lookAtGoal));

				// Look at enemy goals.
				lookAtGoal->x = goalPosition[0];
				lookAtGoal->y = goalPosition[1];
				lookAtGoal->xError = GOAL_LOOK_ERROR[0];
				lookAtGoal->yError = GOAL_LOOK_ERROR[1];
				emit(std::move(lookAtGoal));

				// Look for ball.
			}

			void SoccerStrategy::findBall() {
				// This needs to be replaced with a proper lookAtBall command.
				arma::vec2 ballPosition = transformPoint(currentState.ball.position) + currentState.position;

				auto look = std::make_unique<messages::behaviour::LookAtPoint>();
				look->x = ballPosition[0];
				look->y = ballPosition[1];
				look->xError = BALL_LOOK_ERROR[0];
				look->yError = BALL_LOOK_ERROR[1];
				emit(std::move(look));
			}

			void SoccerStrategy::goToPoint(const arma::vec2& position) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::WayPoint;
				approach->targetHeadingType = WalkTarget::Ball;
				approach->walkMovementType = WalkApproach::WalkToPoint;
				approach->heading = transformPoint(currentState.ball.position) + currentState.position;
				approach->target = position; 

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::watchBall() {
				// This needs to be replaced with a proper lookAtBall command.
				arma::vec2 ballPosition = transformPoint(currentState.ball.position) + currentState.position;

				auto look = std::make_unique<messages::behaviour::LookAtPoint>();
				look->x = ballPosition[0];
				look->y = ballPosition[1];
				look->xError = BALL_LOOK_ERROR[0];
				look->yError = BALL_LOOK_ERROR[1];
				emit(std::move(look));
			}

			void SoccerStrategy::sideStepToPoint(const arma::vec2& position) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::WayPoint;
				approach->targetHeadingType = WalkTarget::Ball;
				approach->walkMovementType = WalkApproach::OmnidirectionalReposition;
				approach->heading = transformPoint(currentState.ball.position) + currentState.position;
				approach->target = position; 

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::approachBall(const arma::vec2& heading) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::Ball;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->walkMovementType = WalkApproach::ApproachFromDirection;
				approach->heading = heading;
				approach->target = transformPoint(currentState.ball.position) + currentState.position;

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::kickBall(const arma::vec2& direction) {
				// Emit aim vector.
				// Currently not implemented. Kick should happen when robot is close enough to the ball.
				(void)direction;
			}

			arma::vec2 SoccerStrategy::transformPoint(const arma::vec2& point) {
				return (currentState.transform * point);
			}
		}  // strategy
	}  // behaviours
}  // modules
