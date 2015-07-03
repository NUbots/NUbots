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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_OMPLPATHPLANNER_PATHPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_OMPLPATHPLANNER_PATHPLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/geometry/Circle.h"
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/WalkPath.h"
#include "messages/support/FieldDescription.h"

namespace modules {
namespace behaviour {
namespace planning {

	using messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::behaviour::WalkPath;
    using messages::support::FieldDescription;
    using utility::math::matrix::Transform2D;
    using utility::math::geometry::Circle;

    class PathPlanner {
    public:

        struct Config {
            float goalpost_safety_margin = 0;
            float ball_obstacle_margin = 0;
            bool calculate_debug_planning_tree = false;
            arma::vec2 robot_footprint_dimensions = {0.12, 0.17};
            // float ball_obstacle_radius = 0;
            // arma::vec2 ball_obstacle_offset = {0, 0};
        } cfg_;


        PathPlanner() {}
        PathPlanner(const FieldDescription& desc, const PathPlanner::Config& config);

        std::unique_ptr<WalkPath> obstacleFreePathBetween(Transform2D start, Transform2D goal, Transform2D ballSpace, double timeLimit);

        std::unique_ptr<WalkPath> omplPathToWalkPath(ompl::base::PathPtr omplPath);

        ompl::base::PathPtr omplPlanPath(Transform2D start, Transform2D goal, Transform2D ballSpace, double timeLimit);

        Circle getBallObstacle(Transform2D ballSpace);

        // The state space used for the last planning run:
        ompl::base::StateSpacePtr stateSpace;

        // The graph generated to calculate the optimal path:
        std::vector<arma::vec> debugPositions;
        std::vector<uint> debugParentIndices;

	// private:
        std::vector<Circle> staticObstacles;
        float ballRadius = 0.05;
    };
}
}
}
#endif
