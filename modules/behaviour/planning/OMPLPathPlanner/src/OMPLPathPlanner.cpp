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

#include "OMPLPathPlanner.h"

#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"

namespace modules {
namespace behaviour {
namespace planning {
    using messages::support::Configuration;
    using messages::input::Sensors;
    using messages::motion::WalkCommand;
    // using messages::behaviour::WalkTarget;
    // using messages::behaviour::WalkApproach;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::KickFinished;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::nubugger::graph;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;

    using LocalisationBall = messages::localisation::Ball;
    using Self = messages::localisation::Self;
    using VisionBall = messages::vision::Ball;
    using VisionObstacle = messages::vision::Obstacle;

    using messages::support::Configuration;
    using messages::behaviour::KickPlan;

    namespace ob = ompl::base;


    OMPLPathPlanner::OMPLPathPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<OMPLPathPlanner>>>([this] (const Configuration<OMPLPathPlanner>&/* config*/) {
            // Use configuration here from file OMPLPathPlanner.yaml
        });

        on<Trigger<KickFinished>>([this] (const KickFinished&) {
            emit(std::move(std::make_unique<WalkStartCommand>()));
        });

        on<Trigger<Every<20, Per<std::chrono::seconds>>>,
            With<std::vector<Self>>,
            Options<Sync<OMPLPathPlanner>, Single>
           >("Follow current path plan", [this] (
             const NUClear::clock::time_point&/* current_time*/,
             const std::vector<Self>& selfs
             ) {
            if (selfs.empty() || currentPath == nullptr) {
                return;
            }
            auto self = selfs.front();

            // TODO: Make the state space a member variable of PathPlanner.

            // Construct the robot state space in which we're planning.
            ob::StateSpacePtr space(new ob::SE2StateSpace());

            // Set the bounds of space to the field area:
            // TODO: Use a FieldDescription from the config system for the lengths.
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-4.5);
            bounds.setHigh(4.5);
            space->as<ob::SE2StateSpace>()->setBounds(bounds);

            // Get the robot's current state as an OMPL SE2 state:
            Transform2D currTrans = {self.position(0), self.position(1), vectorToBearing(self.heading)};
            ob::ScopedState<> currentScopedState(space);
            auto* currentState = currentScopedState->as<ob::SE2StateSpace::StateType>();
            currentState->setX(currTrans.x());
            currentState->setY(currTrans.y());
            currentState->setYaw(currTrans.angle());

            // Find the closest state on the path to the robot's current state:
            auto pathGeom = boost::static_pointer_cast<ompl::geometric::PathGeometric>(currentPath);
            int numStates = pathGeom->getStateCount();
            int index = pathGeom->getClosestIndex(currentState);
            // std::vector<ob::State*>& states = pathGeom->getStates();

            if (index < 0 || numStates <= index + 1) {
                return;
            }

            // Make the robot head towards the next state on the path:

            // ob::State* currState = pathGeom->getState(index);
            // ob::State* nextState = pathGeom->getState(index + 1);
            const ob::SE2StateSpace::StateType* nextState =
                pathGeom->getState(index + 1)->as<ob::SE2StateSpace::StateType>();

            // Transform2D currTrans = { currState->getX(), currState->getY(), currState->getYaw() };
            Transform2D nextTrans = { nextState->getX(), nextState->getY(), nextState->getYaw() };

            std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
            command->command = nextTrans - currTrans;

            emit(std::move(std::make_unique<WalkStartCommand>()));
            emit(std::move(command));
        });

        on<Trigger<Every<2, std::chrono::seconds>>,
            With<LocalisationBall>,
            With<std::vector<Self>>,
            With<KickPlan>,
            With<Optional<std::vector<VisionObstacle>>>,
            Options<Sync<OMPLPathPlanner>, Single>
           >("Generate new path plan", [this] (
             const NUClear::clock::time_point& current_time,
             const LocalisationBall& ball,
             const std::vector<Self>& selfs,
             const KickPlan& kickPlan,
             const std::shared_ptr<const std::vector<VisionObstacle>>&/* robots*/) {

            if (selfs.empty()) {
                return;
            }
            auto self = selfs.front();

            // Generate a new path:
            Transform2D start = {self.position(0), self.position(1), vectorToBearing(self.heading)};
            Transform2D localGoal = {ball.position(0), ball.position(1), 0};

            // Determine the goal position and heading:
            // auto goalHeading = kickPlan.target - self.position;
            arma::vec2 ballPos = start.localToWorld(localGoal).rows(0,1);
            arma::vec2 goalHeading = kickPlan.target - ballPos;
            arma::vec goalPosition = ballPos - 0.1 * arma::normalise(goalHeading);

            NUClear::log("OMPLPP:ballPos:", ballPos.t());
            NUClear::log("OMPLPP: goalPosition:", goalPosition.t());
            NUClear::log("OMPLPP: kickPlan.target:", kickPlan.target.t());

            Transform2D goal = {goalPosition(0), goalPosition(1), vectorToBearing(goalHeading)};
            double timeLimit = 1; // Time limit in seconds.
            auto path = pathPlanner.obstacleFreePathBetween(start, goal, ball, timeLimit);
            // LocalisationBall testBall;
            // testBall.position = {0,1};
            // auto path = pathPlanner.obstacleFreePathBetween({-4,1,0}, {3,3,3.14}, testBall, timeLimit);


            // Store as the current path:
            if (path != nullptr) {
                currentPath = path;
            } else {
                NUClear::log("OMPLPP: Returned path was a nullptr.");
            }

            // Emit the new path to NUSight.
            if (currentPath != nullptr) {

                // Get the path states:
                auto pathGeom = boost::static_pointer_cast<ompl::geometric::PathGeometric>(currentPath);
                std::vector<ob::State*>& states = pathGeom->getStates();

                // Build the path to emit:
                std::vector<arma::vec> positions;

                for (auto* state : states) {
                    // Cast the state as an SE2 state:
                    auto* se2State = state->as<ob::SE2StateSpace::StateType>();

                    // Add the position:
                    arma::vec2 pos = { se2State->getX(), se2State->getY() };
                    positions.push_back(pos);
                }

                emit(utility::nubugger::drawPolyline("OMPLPP_Path", positions));
            }
        });

        // on<Trigger<messages::behaviour::WalkStrategy>,
        //    Options<Sync<OMPLPathPlanner>>>([this] (const messages::behaviour::WalkStrategy& cmd) {
        //     //reset hysteresis variables when a command changes
        // });
    }
}
}
}
