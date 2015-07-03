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

#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/behaviour/WalkPath.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"


namespace modules {
namespace behaviour {
namespace planning {

    using messages::support::Configuration;
    using Self = messages::localisation::Self;

    using messages::behaviour::WalkPath;

    using messages::motion::WalkCommand;
    using messages::motion::KickFinished;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    
    using utility::math::geometry::RotatedRectangle;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;


    WalkPathFollower::WalkPathFollower(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<WalkPathFollower>>>([this] (const Configuration<WalkPathFollower>& config) {
            // Use configuration here from file WalkPathFollower.yaml

            cfg_.waypoint_visit_distance = config["waypoint_visit_distance"].as<float>();
            cfg_.draw_estimated_path = config["draw_estimated_path"].as<bool>();

        });

        on<Trigger<KickFinished>>([this] (const KickFinished&) {
            emit(std::move(std::make_unique<WalkStartCommand>()));
        });

        on<Trigger<WalkPath>,
            With<std::vector<Self>>,
            Options<Sync<WalkPathFollower>, Single>
           >("Update current path plan", [this] (
             const WalkPath& walkPath,
             const std::vector<Self>& selfs
             ) {

            currentPath = walkPath;

            // Draw the robot's estimated path:
            if (cfg_.draw_estimated_path) {
                if (selfs.empty() || currentPath.states.empty()) {
                    return;
                }
                auto self = selfs.front();
                Transform2D currentState = {self.position, vectorToBearing(self.heading)};
                auto estPath = estimatedPath(currentState, currentPath, 0.01, 2000, 40);
                emit(utility::nubugger::drawPath("WPF_EstimatedPath", estPath.states, 0.05, {1,0.8,0}));
            }
        });

        on<Trigger<Every<20, Per<std::chrono::seconds>>>,
            With<std::vector<Self>>,
            // With<WalkPath>,
            Options<Sync<WalkPathFollower>, Single>
           >("Follow current path plan", [this] (
             const NUClear::clock::time_point& current_time,
             const std::vector<Self>& selfs
             // const WalkPath& walkPath
             ) {
            if (selfs.empty() || currentPath.states.empty()) {
                return;
            }
            auto self = selfs.front();

            // TODO: Try representing the path in a space that has the ball position
            // as its origin, and the x-axis in the direction of the kick
            // target from the ball.

            // Get the robot's current state as a Transform2D:
            Transform2D currentState = {self.position, vectorToBearing(self.heading)};
            emit(utility::nubugger::drawRectangle("WPF_RobotFootprint", RotatedRectangle(currentState, {0.12, 0.17})));
            emit(utility::nubugger::drawRectangle("WPF_GoalState", RotatedRectangle(currentPath.goal, {0.12, 0.17}), {0.4, 0.4, 0.4}, 0.123));

            // Remove unnecessary (visited) states from the path:
            int removed = trimPath(currentState, currentPath);
            if (removed && cfg_.draw_estimated_path) {
                auto estPath = estimatedPath(currentState, currentPath, 0.01, 2000, 40);
                emit(utility::nubugger::drawPath("WPF_EstimatedPath", estPath.states, 0.05, {1,0.8,0}));
            }

            // Aim for the index after the closest state:
            int targetIndex = std::min(1, int(currentPath.states.size()));
            Transform2D targetState = currentPath.states[targetIndex]; // {3, 3, 3.14};
            emit(utility::nubugger::drawRectangle("WPF_TargetState", RotatedRectangle(targetState, {0.12, 0.17}), {1, 0, 0}));

            // Emit a walk command to move towards the target state:
            auto command = std::make_unique<WalkCommand>(walkBetween(currentState, targetState));
            emit(std::move(std::make_unique<WalkStartCommand>()));
            emit(std::move(command));
        });
    }

    int WalkPathFollower::trimPath(const Transform2D& currentState, WalkPath& walkPath) {
        auto size = walkPath.states.size();

        // Find the index of the closest state:
        auto closestIndex = closestPathIndex(currentState, walkPath);
        // emit(utility::nubugger::drawRectangle("WPF_Closest", RotatedRectangle(walkPath.states[closestIndex], {0.12, 0.17}), {0, 0, 0}));

        // Check if we're close enough to have 'visited' the closest state:
        if (!isVisited(currentState, walkPath.states[closestIndex])) {
            return 0;
        }

        // Remove all states before the closest state:
        if (closestIndex != 0) {
            walkPath.states.erase(walkPath.states.begin(),
                                  walkPath.states.begin() + closestIndex);
            // emit(utility::nubugger::drawPath("OMPLPP_Path", walkPath.states, 0.1, {0,0.5,0.5}));
        }

        // Return the number of states removed from the path.
        return size - walkPath.states.size();
    }

    bool WalkPathFollower::isVisited(const Transform2D& currentState, const Transform2D& visitState) {
        double dist = arma::norm(visitState.xy() - currentState.xy());
        
        return dist < cfg_.waypoint_visit_distance;
    }

    int WalkPathFollower::closestPathIndex(const Transform2D& currentState, const WalkPath& walkPath) {
        int numStates = walkPath.states.size();
        int closestIndex = 0;
        double closestDist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < numStates; i++) {
            double dist = arma::norm(walkPath.states[i].xy() - currentState.xy());
            if (dist < closestDist) {
                closestDist = dist;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    WalkCommand WalkPathFollower::walkBetween(const Transform2D& currentState, const Transform2D& targetState) {
        auto diff = arma::vec2(targetState.xy() - currentState.xy());
        auto dir = vectorToBearing(diff);
        double wcAngle = utility::math::angle::signedDifference(dir, currentState.angle());
        
        // TODO: Consider the heading of targetState in planning.

        WalkCommand command;
        command.command = {1, 0, wcAngle};
        return command;
    }

    WalkPath WalkPathFollower::estimatedPath(const Transform2D& currentState, const WalkPath& walkPath, float timeStep, int simSteps, int sample) {
        if (sample <= 0) {
            sample = 1;
        }
        int stepNum = 0;

        auto state = currentState;
        auto path = walkPath;
        WalkPath robotPath;
        robotPath.states.push_back(state);

        for (int i = 0; i < simSteps; i++) {
            trimPath(state, path);

            int targetIndex = std::min(1, int(path.states.size()));
            Transform2D targetState = path.states[targetIndex];

            auto command = walkBetween(state, targetState);

            command.command.xy() = state.rotation() * command.command.xy() * 0.2;
            state += command.command * timeStep;
            stepNum++;
            if (stepNum % sample == 0) {
                robotPath.states.push_back(state);
            }
        }

        return robotPath;
    }

}
}
}
