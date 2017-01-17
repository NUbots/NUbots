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

#include <sstream>
#include "extension/Configuration.h"
#include "message/support/FieldDescription.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/WalkPath.h"
#include "message/behaviour/KickPlan.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/Circle.h"

namespace module {
namespace behaviour {
namespace planning {
    using extension::Configuration;
    using message::support::FieldDescription;

    using utility::nubugger::graph;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;
    using utility::math::geometry::Circle;

    using LocalisationBall = message::localisation::Ball;
    using Self = message::localisation::Self;

    using message::behaviour::MotionCommand;
    using message::behaviour::KickPlan;

    namespace ob = ompl::base;

    OMPLPathPlanner::OMPLPathPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration, Trigger<FieldDescription>>("OMPLPathPlanner.yaml").then("OMPLPathPlanner Configuration", [this](const Configuration& config, const FieldDescription& desc) {
            // Use configuration here from file OMPLPathPlanner.yaml

            lastPlanningTime = NUClear::clock::time_point::min();

            cfg_.planning_interval = config["planning_interval"].as<float>();
            cfg_.planning_time_limit = config["planning_time_limit"].as<float>();

            cfg_.draw_planning_tree = config["draw_planning_tree"].as<bool>();
            cfg_.target_offset = config["target_offset"].as<arma::vec2>();

            PathPlanner::Config ppConfig;
            ppConfig.goalpost_safety_margin = config["goalpost_safety_margin"].as<float>();
            ppConfig.ball_obstacle_margin = config["ball_obstacle_margin"].as<float>();
            ppConfig.robot_footprint_dimensions = config["robot_footprint_dimensions"].as<arma::vec2>();
            ppConfig.calculate_debug_planning_tree = cfg_.draw_planning_tree;
            // ppConfig.ball_obstacle_radius = config["ball_obstacle_radius"].as<float>();
            // ppConfig.ball_obstacle_offset = config["ball_obstacle_offset"].as<arma::vec2>();

            pathPlanner = PathPlanner(desc, ppConfig);
        });

        // Enable/Disable path generation based on the current motion command.
        on<Trigger<MotionCommand>>().then([this] (const MotionCommand& command) {
            if (command.type == MotionCommand::Type::WalkToState ||
                command.type == MotionCommand::Type::BallApproach) {
                generatePathReaction.enable();
            } else {
                generatePathReaction.disable();
            }
        });

        generatePathReaction = on<Every<10, Per<std::chrono::seconds>>
                                , With<MotionCommand>
                                , With<LocalisationBall>
                                , With<std::vector<Self>>
                                , With<KickPlan>
                                , Sync<OMPLPathPlanner>
                                , Single
           >().then("Generate new path plan", [this] (
             const MotionCommand& command,
             const LocalisationBall& ball,
             const std::vector<Self>& selfs,
             const KickPlan& kickPlan
             ) {

            // Ensure that we have robot positions:
            if (selfs.empty()) {
                return;
            }
            auto self = selfs.front();

            // Enforce the planning interval:
            // TODO: Override the planning interval if the environment has changed significantly.
            // (or if command is marked as 'urgent'?)
            auto now = NUClear::clock::now(); // TODO: use current_time instead.
            auto msSinceLastPlan = std::chrono::duration_cast<std::chrono::microseconds>(now - lastPlanningTime).count();
            double timeSinceLastPlan = 1e-6 * static_cast<double>(msSinceLastPlan);
            if (std::abs(timeSinceLastPlan) < cfg_.planning_interval) {
                return;
            }
            lastPlanningTime = now;

            // Generate a new path:

            // Use the robot's current position estimate as the start state:
            Transform2D start = {self.position, vectorToBearing(self.heading)};

            // Create the 'ball space' (origin at ball pos, and x-axis along
            // target kick direction) used to determine the ball obstacle and
            // possibly the goal position:
            Transform2D localBall = {ball.position, 0};
            arma::vec2 globalBall = start.localToWorld(localBall).xy();
            Transform2D ballSpace = Transform2D::lookAt(globalBall, kickPlan.target);

            // Determine the goal position and heading:
            Transform2D goal;
            switch (command.type) {
                case MotionCommand::Type::WalkToState: {
                    goal = command.goalState;
                } break;

                case MotionCommand::Type::BallApproach: {
                    // Calculate the goal position which places the robot in a
                    // position to kick the ball toward the kicktarget with a
                    // forward kick.
                    // TODO: Clean this up (shouldn't be accessing path planner state).
                    arma::vec2 ballSpaceGoal = arma::vec2({-0.5 * pathPlanner.cfg_.robot_footprint_dimensions(0), 0})
                                         + arma::vec2({-pathPlanner.ballRadius, 0})
                                         + cfg_.target_offset;
                    goal = ballSpace.localToWorld({ballSpaceGoal, 0});
                } break;

                default: {
                    NUClear::log<NUClear::WARN>("OMPLPathPlanner: Unexpected MotionCommand::Type: ", int(command.type));
                    return;
                }
            }

            // Plan the path:
            // TODO (not too important): Detect the case where the initial
            // state is invalid, and handle it more intelligently.
            auto path = pathPlanner.obstacleFreePathBetween(start, goal, ballSpace, cfg_.planning_time_limit);


            // Draw obstacles:
            emit(utility::nubugger::drawCircle("OMPLPP_BallShadow", Circle(pathPlanner.ballRadius, ballSpace.xy()), 0.123, {1, 0.8, 0}));
            emit(utility::nubugger::drawCircle("OMPLPP_BallObstacle", pathPlanner.getBallObstacle(ballSpace), 0.12, {0.6, 0.4, 0.2}));
            int obsNum = 0;
            for (auto& obs : pathPlanner.staticObstacles) {
                std::stringstream str;
                str << "OMPLPP_GoalObstacle" << (obsNum++);
                emit(utility::nubugger::drawCircle(str.str(), obs, 0.012, {0.6, 0.4, 0.2}));
            }
            // Emit the new path to NUSight:
            if (path != nullptr) {
                emit(utility::nubugger::drawPath("OMPLPP_Path", path->states, 0.1, {1,0.5,1}));

                if (cfg_.draw_planning_tree) {
                    emit(utility::nubugger::drawTree("OMPLPP_DebugTree", pathPlanner.debugPositions, pathPlanner.debugParentIndices, 0.02, {0.5, 0,0.5}));
                }
            }

            // Emit the new path:
            if (path != nullptr) {
                path->command = command;
                emit(std::move(path));
            } else {
                NUClear::log("OMPLPP: Returned path was a nullptr.");
            }
        }).disable();
    }
}
}
}
