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
        });

        on<Trigger<KickFinished>>([this] (const KickFinished&) {
            emit(std::move(std::make_unique<WalkStartCommand>()));
        });

        // on<Trigger<WalkPath>,
        //     Options<Single>
        //    >("Update current path plan", [this] (
        //      const WalkPath& walkPath
        //      ) {
        //      // Reset all path following state:

        //      // Set current state to 0:

        //      // Set start time to current time:
        // });

        on<Trigger<Every<20, Per<std::chrono::seconds>>>,
            With<std::vector<Self>>,
            With<WalkPath>,
            Options<Single>
           >("Follow current path plan", [this] (
             const NUClear::clock::time_point& current_time,
             const std::vector<Self>& selfs,
             const WalkPath& walkPath
             ) {
            if (selfs.empty() || walkPath.states.empty()) {
                return;
            }
            auto self = selfs.front();

            // TODO: Represent the path in a space that has the ball position
            // as its origin, and the x-axis in the direction of the kick
            // target from the ball.

            // Get the robot's current state as a Transform2D:
            Transform2D currState = {self.position(0), self.position(1), vectorToBearing(self.heading)};
            emit(utility::nubugger::drawRectangle("WPF_RobotFootprint", RotatedRectangle(currState, {0.12, 0.17})));

            
            // Find the index of the closest state:
            int numStates = walkPath.states.size();
            int closestIndex = 0;
            double closestDist = std::numeric_limits<double>::infinity();
            for (int i = 0; i < numStates; i++) {
                double dist = arma::norm(walkPath.states[i] - currState);
                if (dist < closestDist) {
                    closestDist = dist;
                    closestIndex = i;
                }
            }
            // Aim for the index after the closest state:
            int targetIndex = std::min(closestIndex + 1, numStates);
            Transform2D targetState = walkPath.states[targetIndex]; // {3, 3, 3.14};

            // // Remove all states before the closest state:
            // if (closestIndex != 0) {
            //     walkPath.states.erase(walkPath.states.begin(),
            //                            walkPath.states.begin() + (closestIndex - 1));
            // }

            emit(utility::nubugger::drawRectangle("WPF_TargetState", RotatedRectangle(targetState, {0.12, 0.17}), {1, 0, 0}));


            // Emit a walk command to move towards the target state:
            std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
            auto diff = arma::vec2(targetState.xy() - currState.xy());
            auto dir = vectorToBearing(diff);
            double wcAngle = utility::math::angle::signedDifference(dir, currState.angle());
            command->command = {1, 0, wcAngle};
            emit(std::move(std::make_unique<WalkStartCommand>()));
            emit(std::move(command));
        });
    }
}
}
}
