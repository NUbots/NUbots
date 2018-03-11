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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "WalkPathFollower.h"

#include <limits>

#include "extension/Configuration.h"

#include "message/behaviour/KickPlan.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/WalkPath.h"
#include "message/motion/KickCommand.h"
#include "message/motion/WalkCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace behaviour {
    namespace skills {

        using extension::Configuration;

        using Self = message::localisation::Self;
        using Ball = message::localisation::Ball;

        using message::behaviour::MotionCommand;
        using message::behaviour::WalkPath;

        using message::motion::DisableWalkEngineCommand;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::KickFinished;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        using utility::math::angle::vectorToBearing;
        using utility::math::geometry::RotatedRectangle;
        using utility::math::matrix::Transform2D;

        WalkPathFollower::WalkPathFollower(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            // Register the path follower with the subsumption system:
            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "WalkPathFollower",
                {
                    // Limb sets required by the walk engine:
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                },
                [this](const std::set<LimbID>& givenLimbs) {
                    if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                        // Enable the walk engine.
                        emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<LimbID>& takenLimbs) {
                    if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                        // Shut down the walk engine, since we don't need it right now.
                        emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<ServoID>&) {
                    // nothing
                }}));

            on<Configuration>("WalkPathFollower.yaml").then([this](const Configuration& config) {
                // Use configuration here from file WalkPathFollower.yaml

                cfg_.waypoint_visit_distance = config["waypoint_visit_distance"].as<double>();
                cfg_.draw_estimated_path     = config["draw_estimated_path"].as<bool>();

                cfg_.walk_about_x_strafe         = config["walk_about_xstrafe"].as<double>();
                cfg_.walk_about_y_strafe         = config["walk_about_ystrafe"].as<double>();
                cfg_.walk_about_rotational_speed = config["walk_about_rotational_speed"].as<double>();
                cfg_.walk_to_far_forward_speed   = config["walk_to_far_forward_speed"].as<double>();
                cfg_.walk_to_near_speed          = config["walk_to_near_speed"].as<double>();
                cfg_.goal_close_distance         = config["goal_close_distance"].as<double>();

            });

            // Enable/Disable path following based on the current motion command.
            on<Trigger<MotionCommand>>().then([this](const MotionCommand& command) {
                if (command.type == MotionCommand::Type::WalkToState
                    || command.type == MotionCommand::Type::BallApproach) {
                    followPathReaction.enable();
                    updatePathReaction.enable();
                    // Increase priority?
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {25, 10}}));
                }
                else {
                    followPathReaction.disable();
                    updatePathReaction.disable();
                    // Decrease priority?
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {0, 0}}));
                }
            });

            // // TODO: Review the interaction of the kick with the WalkPathFollower.
            // on<Trigger<KickFinished>>().then([this] (const KickFinished&) {
            //     Re-issue walk command with target velocity...
            // });

            updatePathReaction =
                on<Trigger<WalkPath>, With<std::vector<Self>>, Sync<WalkPathFollower>, Single>()
                    .then("Update current path plan",
                          [this](const WalkPath& walkPath, const std::vector<Self>& selfs) {

                              currentPath = walkPath;

                              // Draw the robot's estimated path:
                              if (cfg_.draw_estimated_path) {
                                  if (selfs.empty() || currentPath.states.empty()) {
                                      NUClear::log(__FILE__, __LINE__, "empty self and current states");
                                      return;
                                  }
                                  auto self                = selfs.front();
                                  Transform2D currentState = {self.position, vectorToBearing(self.heading)};
                                  auto estPath             = estimatedPath(currentState, currentPath, 0.01, 2000, 40);
                                  emit(utility::nubugger::drawPath(
                                      "WPF_EstimatedPath", estPath.states, 0.05, {1, 0.8, 0}));
                              }
                          })
                    .disable();

            followPathReaction =
                on<Every<20, Per<std::chrono::seconds>>,
                   With<std::vector<Self>>,
                   With<Ball>,
                   // With<WalkPath>,
                   Sync<WalkPathFollower>,
                   Single>()
                    .then(
                        "Follow current path plan",
                        [this](const std::vector<Self>& selfs, const Ball& ball) {
                            if (selfs.empty() || currentPath.states.empty()) {
                                return;
                            }
                            auto self = selfs.front();

                            // Get the robot's current state as a Transform2D:
                            Transform2D currentState = {self.position, vectorToBearing(self.heading)};
                            emit(utility::nubugger::drawRectangle("WPF_RobotFootprint",
                                                                  RotatedRectangle(currentState, {0.12, 0.17})));
                            emit(utility::nubugger::drawRectangle("WPF_GoalState",
                                                                  RotatedRectangle(currentPath.goal, {0.12, 0.17}),
                                                                  {0.4, 0.4, 0.4},
                                                                  0.123));

                            // if (cfg_.follow_path_in_ball_space &&
                            //     currentPath.command.type == MotionCommand::Type::BallApproach) {
                            //     // Ball space is a space that has the ball position
                            //     // as its origin, and the x-axis in the direction of the kick
                            //     // target from the ball.

                            //     // Find current ball space.
                            //     arma::vec2 worldBall = currentState.localToWorld({ball.position, 0}).xy();
                            //     Transform2D currentBallSpace = Transform2D::lookAt(worldBall,
                            //     currentPath.command.kickTarget);

                            //     // Transform robot from world space into (current) ball space:
                            //     auto ballSpaceState = currentBallSpace.worldToLocal(currentState);

                            //     // Transform the robot from ball space into the world space at
                            //     // the time of path planning (note that walk commands will be
                            //     // valid as they in robot space):
                            //     currentState = currentPath.ballSpace.localToWorld(ballSpaceState);
                            // }


                            // TODO: Remove.
                            // RoboCup HACK - Just aim for the goal state:
                            Transform2D targetState;
                            if (currentPath.command.type == MotionCommand::Type::BallApproach) {
                                arma::vec2 worldBall = currentState.localToWorld({ball.position, 0}).xy();
                                Transform2D currentBallSpace =
                                    Transform2D::lookAt(worldBall, currentPath.command.kickTarget);
                                targetState = currentBallSpace;
                            }
                            else {
                                targetState = currentPath.goal;
                            }


                            std::unique_ptr<WalkCommand> walkCommand;
                            if (isGoalClose(
                                    currentState,
                                    targetState)) {  // TODO: Consider only doing walkBetweenFar/walkBetweenNear.
                                walkCommand = std::make_unique<WalkCommand>(walkBetweenNear(currentState, targetState));
                            }
                            else {
                                // Make a walk command to move towards the target state
                                walkCommand = std::make_unique<WalkCommand>(walkBetweenFar(currentState, targetState));
                            }

                            // emit(utility::nubugger::drawArrow("WPF_Closest_Arrow",
                            // currentState.localToWorld(walkCommand->command), {1,1,1}, 1));
                            arma::vec2 arrowTip = currentState.localToWorld(walkCommand->command).xy();
                            arma::vec2 dirPoint =
                                currentState
                                    .localToWorld({walkCommand->command.rotation() * walkCommand->command.xy(), 0})
                                    .xy();

                            emit(utility::nubugger::drawArrow("WPF_Closest_Arrow", currentState.xy(), arrowTip, 1));
                            emit(utility::nubugger::drawArrow("WPF_Closest_Arrow_Rotation", arrowTip, dirPoint, 1));

                            emit(std::move(walkCommand));

                            emit(utility::nubugger::drawRectangle(
                                "WPF_Closest", RotatedRectangle(targetState, {0.12, 0.17}), {0, 0, 0}));

                            emit(utility::nubugger::drawArrow("WPF_Closest_Arrow", targetState, {1, 0, 1}, 1));

                            // // Remove unnecessary (visited) states from the path:
                            // int removed = trimPath(currentState, currentPath);
                            // if (removed && cfg_.draw_estimated_path) {
                            //     auto estPath = estimatedPath(currentState, currentPath, 0.01, 2000, 40);
                            //     emit(utility::nubugger::drawPath("WPF_EstimatedPath", estPath.states, 0.05,
                            //     {1,0.8,0}));
                            // }

                            // // Emit a walk command to move towards the target state:
                            // emit(std::move(walkToNextNode(currentState)));
                        })
                    .disable();
        }

        int WalkPathFollower::trimPath(const Transform2D& currentState, WalkPath& walkPath) {
            auto size = walkPath.states.size();

            // Find the index of the closest state:
            auto closestIndex = closestPathIndex(currentState, walkPath);
            // emit(utility::nubugger::drawRectangle("WPF_Closest", RotatedRectangle(walkPath.states[closestIndex],
            // {0.12, 0.17}), {0, 0, 0}));

            // Check if we're close enough to have 'visited' the closest state:
            if (!isVisited(currentState, walkPath.states[closestIndex])) {
                return 0;
            }

            // Remove all states before the closest state:
            if (closestIndex != 0) {
                walkPath.states.erase(walkPath.states.begin(), walkPath.states.begin() + closestIndex);
                // emit(utility::nubugger::drawPath("OMPLPP_Path", walkPath.states, 0.1, {0,0.5,0.5}));
            }

            // Return the number of states removed from the path.
            return size - walkPath.states.size();
        }

        bool WalkPathFollower::isVisited(const Transform2D& currentState, const Transform2D& visitState) {
            // TODO: Abstract away the distance metric used between states.
            double dist = arma::norm(visitState.xy() - currentState.xy());
            return dist < cfg_.waypoint_visit_distance;
        }

        bool WalkPathFollower::isGoalClose(const Transform2D& currentState, const Transform2D& visitState) {
            // TODO: Abstract away the distance metric used between states.
            double dist = arma::norm(visitState.xy() - currentState.xy());
            return dist < cfg_.goal_close_distance;
        }

        int WalkPathFollower::closestPathIndex(const Transform2D& currentState, const WalkPath& walkPath) {
            int numStates      = walkPath.states.size();
            int closestIndex   = 0;
            double closestDist = std::numeric_limits<double>::infinity();
            for (int i = 0; i < numStates; i++) {
                // TODO: Abstract away the distance metric used between states.
                double dist = arma::norm(walkPath.states[i].xy() - currentState.xy());
                if (dist < closestDist) {
                    closestDist  = dist;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }

        std::unique_ptr<WalkCommand> WalkPathFollower::walkToNextNode(const Transform2D& currentState,
                                                                      bool /*noLogging*/) {
            // Aim for the index after the closest state:
            int targetIndex         = std::min(1, int(currentPath.states.size()) - 1);
            Transform2D targetState = currentPath.states[targetIndex];  // {3, 3, 3.14};
            emit(utility::nubugger::drawRectangle(
                "WPF_TargetState", RotatedRectangle(targetState, {0.12, 0.17}), {1, 0, 0}));

            std::unique_ptr<WalkCommand> command;

            // If we have reached our target
            // if (targetIndex == 0 && isGoalClose(currentState, targetState)) {
            //     command = std::make_unique<WalkCommand>(walkBetweenNear(currentState, targetState));
            // } else {
            // Make a walk command to move towards the target state
            command = std::make_unique<WalkCommand>(walkBetweenFar(currentState, targetState));
            // }

            return command;
        }

        WalkCommand WalkPathFollower::walkBetweenFar(const Transform2D& currentState, const Transform2D& targetState) {
            auto diff      = arma::vec2(targetState.xy() - currentState.xy());
            auto dir       = vectorToBearing(diff);
            double wcAngle = utility::math::angle::signedDifference(dir, currentState.angle());
            // TODO: Consider the heading of targetState in planning.

            WalkCommand command(subsumptionId, {cfg_.walk_to_far_forward_speed, 0, wcAngle});
            return command;
        }

        WalkCommand WalkPathFollower::walkBetweenNear(const Transform2D& currentState, const Transform2D& targetState) {
            Transform2D localTarget = currentState.worldToLocal(targetState);  // creating local target first
            int angleSign           = (localTarget.angle() < 0) ? -1 : 1;      // angle must be normalised.

            double rotationSpeed      = angleSign * cfg_.walk_about_rotational_speed;
            arma::vec2 translationVec = arma::normalise(localTarget.xy());

            double translationAngle = utility::math::angle::vectorToBearing(translationVec);

            double translationSpeed = (1 - std::abs(translationAngle) * (0.25 / M_PI));

            arma::vec2 translationVelocity = translationVec * translationSpeed;

            Transform2D velocity = {translationVelocity, rotationSpeed};

            WalkCommand command(subsumptionId, velocity);

            return command;


            // Angle between current heading and target heading
            /// double walkAboutAngle = utility::math::angle::signedDifference(targetState.angle(),
            /// currentState.angle());
            /// int angleSign = (walkAboutAngle < 0) ? -1 : 1;
            // TODO: Consider using a smaller, non-constant speed.
            /// double rotationSpeed = angleSign * cfg_.walk_about_rotational_speed;

            // if (std::abs(walkAboutAngle) < M_PI*0.125) {
            ///     Transform2D velocity = {cfg_.walk_to_near_speed * arma::normalise(targetState.xy() -
            ///     currentState.xy()), rotationSpeed}; //TODO make 20 seconds the variable update_frequency
            ///     WalkCommand command(subsumptionId, velocity);
            ///     return command;
            // } else {
            //     arma::vec2 strafe = { std::max(cfg_.walk_about_x_strafe, 0.0), -angleSign * cfg_.walk_about_y_strafe
            //     };

            //     arma::vec2 strafeClipped = arma::normalise(strafe) * std::min(1.0, arma::norm(strafe));

            //     Transform2D velocity = {strafeClipped, rotationSpeed}; //TODO make 20 seconds the variable
            //     update_frequency
            //     WalkCommand command(subsumptionId, velocity);
            //     return command;
            // }
        }

        WalkPath WalkPathFollower::estimatedPath(const Transform2D& currentState,
                                                 const WalkPath& walkPath,
                                                 double timeStep,
                                                 int simSteps,
                                                 int sample) {
            if (sample <= 0) {
                sample = 1;
            }
            int stepNum = 0;

            auto state = currentState;
            auto path  = walkPath;
            WalkPath robotPath;
            robotPath.states.push_back(state);

            for (int i = 0; i < simSteps; i++) {
                trimPath(state, path);

                auto command = walkToNextNode(state, true);

                command->command.xy() = state.rotation() * command->command.xy() * 0.2;
                state += command->command * timeStep;
                stepNum++;
                if (stepNum % sample == 0) {
                    robotPath.states.push_back(state);
                }
            }

            return robotPath;
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
