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
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/behaviour/WalkPath.h"
#include "messages/behaviour/KickPlan.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/geometry/Intersection.h"

namespace modules {
namespace behaviour {
namespace planning {
    using messages::support::Configuration;
    using messages::support::FieldDescription;
    
    using utility::nubugger::graph;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;
    using utility::math::geometry::Circle;
    using utility::math::geometry::RotatedRectangle;
    namespace intersection = utility::math::geometry::intersection;

    using LocalisationBall = messages::localisation::Ball;
    using Self = messages::localisation::Self;
    using VisionBall = messages::vision::Ball;
    using VisionObstacle = messages::vision::Obstacle;

    using messages::behaviour::KickPlan;

    namespace ob = ompl::base;

    OMPLPathPlanner::OMPLPathPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        auto updateConfigLambda = [this](const Configuration<OMPLPathPlanner>& config, const FieldDescription& desc) {
            // Use configuration here from file OMPLPathPlanner.yaml

            lastPlanningTime = NUClear::clock::time_point::min();

            cfg_.planning_interval = config["planning_interval"].as<float>();
            cfg_.planning_time_limit = config["planning_time_limit"].as<float>();
            
            cfg_.draw_planning_tree = config["draw_planning_tree"].as<bool>();

            PathPlanner::Config ppConfig;
            ppConfig.goalpost_safety_margin = config["goalpost_safety_margin"].as<float>();
            ppConfig.ball_obstacle_margin = config["ball_obstacle_margin"].as<float>();
            ppConfig.calculate_debug_planning_tree = cfg_.draw_planning_tree;
            // ppConfig.ball_obstacle_radius = config["ball_obstacle_radius"].as<float>();
            // ppConfig.ball_obstacle_offset = config["ball_obstacle_offset"].as<arma::vec2>();

            pathPlanner = PathPlanner(desc, ppConfig);
        };

        // TODO: Find out why these don't compile with Options<Sync<OMPLPathPlanner>.
        on<With<Configuration<OMPLPathPlanner>>, Trigger<FieldDescription>>("OMPLPathPlanner Configuration", updateConfigLambda);
        on<Trigger<Configuration<OMPLPathPlanner>>, With<FieldDescription>>("OMPLPathPlanner Configuration", updateConfigLambda);


        on<Trigger<Every<10, Per<std::chrono::seconds>>>,
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

            // Ensure that we have robot positions:
            if (selfs.empty()) {
                return;
            }
            auto self = selfs.front();

            // Enforce the planning interval:
            auto now = NUClear::clock::now();
            auto msSinceLastPlan = std::chrono::duration_cast<std::chrono::microseconds>(now - lastPlanningTime).count();
            double timeSinceLastPlan = 1e-6 * static_cast<double>(msSinceLastPlan);
            if (std::abs(timeSinceLastPlan) < cfg_.planning_interval) {
                return;
            }
            lastPlanningTime = now;

            // Generate a new path:
            Transform2D start = {self.position(0), self.position(1), vectorToBearing(self.heading)};
            Transform2D localGoal = {ball.position(0), ball.position(1), 0};

            // Determine the goal position and heading:
            arma::vec2 ballPos = start.localToWorld(localGoal).xy();
            arma::vec2 goalHeading = kickPlan.target - ballPos;
            arma::vec2 goalPosition = ballPos - 0.1 * arma::normalise(goalHeading);
            double goalBearing = vectorToBearing(goalHeading);
            Transform2D goal = {goalPosition(0), goalPosition(1), goalBearing};

            // Create the ball space (origin at ball pos, and x-axis along target kick direction):
            Transform2D ballSpace = {ballPos(0), ballPos(1), goalBearing};

            // Draw obstacles:
            emit(utility::nubugger::drawCircle("OMPLPP_BallShadow", Circle(pathPlanner.ballRadius, ballSpace.xy()), 0.123, {1, 0.8, 0}));
            emit(utility::nubugger::drawCircle("OMPLPP_BallObstacle", pathPlanner.getBallObstacle(ballSpace), 0.12, {0.6, 0.4, 0.2}));
            int obsNum = 0;
            for (auto& obs : pathPlanner.staticObstacles) {
                std::stringstream str;
                str << "OMPLPP_GoalObstacle" << (obsNum++);
                emit(utility::nubugger::drawCircle(str.str(), obs, 0.012, {0.6, 0.4, 0.2}));
            }

            // Plan the path:
            auto path = pathPlanner.obstacleFreePathBetween(start, goal, ballSpace, cfg_.planning_time_limit);

            // Emit the new path to NUSight:
            if (path != nullptr) {
                emit(utility::nubugger::drawPath("OMPLPP_Path", path->states, 0.1, {1,0.5,1}));
                
                if (cfg_.draw_planning_tree) {
                    emit(utility::nubugger::drawTree("OMPLPP_DebugTree", pathPlanner.debugPositions, pathPlanner.debugParentIndices, 0.02, {0.5, 0,0.5}));
                }
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
