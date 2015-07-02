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
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/behaviour/WalkPath.h"
#include "messages/behaviour/KickPlan.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"

namespace modules {
namespace behaviour {
namespace planning {
    using messages::support::Configuration;
    
    using utility::nubugger::graph;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;

    using LocalisationBall = messages::localisation::Ball;
    using Self = messages::localisation::Self;
    using VisionBall = messages::vision::Ball;
    using VisionObstacle = messages::vision::Obstacle;

    using messages::behaviour::KickPlan;

    namespace ob = ompl::base;

    OMPLPathPlanner::OMPLPathPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<OMPLPathPlanner>>>([this] (const Configuration<OMPLPathPlanner>&/* config*/) {
            // Use configuration here from file OMPLPathPlanner.yaml
        });

        // TODO: Add planning frequency to config.
        on<Trigger<Every<10, std::chrono::seconds>>,
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
            arma::vec2 ballPos = start.localToWorld(localGoal).rows(0,1);
            arma::vec2 goalHeading = kickPlan.target - ballPos;
            arma::vec goalPosition = ballPos - 0.1 * arma::normalise(goalHeading);

            Transform2D goal = {goalPosition(0), goalPosition(1), vectorToBearing(goalHeading)};
            double timeLimit = 0.1; // Time limit in seconds. TODO: Add to config.

            // TODO: Use an artificially larger ball, offset away from the
            // goal position for obstacle avoidance.
            auto path = pathPlanner.obstacleFreePathBetween(start, goal, ballPos, timeLimit);

            // Emit the new path to NUSight:
            if (path != nullptr) {
                emit(utility::nubugger::drawPath("OMPLPP_Path", path->states, 0.1, {1,0.5,1}));
                emit(utility::nubugger::drawTree("OMPLPP_DebugTree", pathPlanner.debugPositions, pathPlanner.debugParentIndices, 0.02, {0.5, 0,0.5}));
            }

            // Emit the new path:
            if (path != nullptr) {
                emit(std::move(path));
            } else {
                NUClear::log("OMPLPP: Returned path was a nullptr.");
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
