/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "PlanWalkPath.hpp"

#include <tinyrobotics/math.hpp>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::localisation::Ball;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkAlign;
    using message::planning::WalkDirect;
    using message::planning::WalkTo;

    using message::skill::Walk;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.max_translational_velocity_magnitude = config["max_translational_velocity_magnitude"].as<double>();
            cfg.min_translational_velocity_magnitude = config["min_translational_velocity_magnitude"].as<double>();

            cfg.acceleration                = config["acceleration"].as<double>();
            cfg.approach_radius             = config["approach_radius"].as<double>();
            cfg.align_radius                = config["align_radius"].as<double>();
            cfg.walk_direct_error_threshold = config["walk_direct_error_threshold"].as<double>();


            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.min_angular_velocity = config["min_angular_velocity"].as<double>();

            log<NUClear::INFO>("Max translational velocity magnitude: ", cfg.max_translational_velocity_magnitude);

            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>, Uses<Walk>>().then([this](const WalkTo& walk_to, const Uses<Walk>& walk) {
            // If we haven't got an active walk task, then reset the velocity to minimum velocity
            if (walk.run_state == GroupInfo::RunState::NO_TASK) {
                velocity_magnitude = cfg.min_translational_velocity_magnitude;
            }

            auto pose_error               = tinyrobotics::homogeneous_error(walk_to.Hrd, Eigen::Isometry3d::Identity());
            double goal_position_error    = pose_error.head(3).norm();
            double goal_orientation_error = pose_error.tail(3).norm();
            double walk_direct_orientation_error =
                std::abs(std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x()));

            emit(graph("Goal position error", goal_position_error));
            emit(graph("Goal orientation error", goal_orientation_error));
            emit(graph("Walk direct angle error", walk_direct_orientation_error));

            // 1. If far away, and not facing the goal, then rotate on spot to face the goal
            if (goal_position_error > cfg.approach_radius
                && walk_direct_orientation_error > cfg.walk_direct_error_threshold) {
                emit<Task>(std::make_unique<TurnOnSpot>(
                    std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x()) < 0));
                log<NUClear::INFO>("Rotating on spot to face goal");
                return;
            }

            // 2. If far away, and facing the goal, then walk towards the goal directly
            if (goal_position_error > cfg.approach_radius
                && walk_direct_orientation_error < cfg.walk_direct_error_threshold) {
                emit<Task>(std::make_unique<WalkDirect>(walk_to.Hrd));
                log<NUClear::INFO>("Walking directly towards goal");
                return;
            }

            // 3. If close to the goal, then walk towards the goal directly but do not rotate towards the goal
            if (goal_position_error < cfg.approach_radius && goal_position_error > cfg.align_radius) {
                emit<Task>(std::make_unique<WalkAlign>(walk_to.Hrd));
                log<NUClear::INFO>("Aligning towards goal");
                return;
            }

            // 4. If close to the goal, but not aligned with the goal, then rotate on spot to face the goal
            if (goal_position_error < cfg.align_radius && goal_orientation_error > cfg.walk_direct_error_threshold) {
                emit<Task>(std::make_unique<TurnOnSpot>(walk_to.Hrd(1, 0) < 0));
                log<NUClear::INFO>("Rotating on spot to face goal");
                return;
            }

            // 5. If close to the goal, and aligned with the goal, then walk slowly towards the goal
            emit<Task>(std::make_unique<WalkAlign>(walk_to.Hrd));
        });

        on<Provide<WalkDirect>>().then([this](const WalkDirect& walk_direct) {
            // Accelerate to max velocity
            velocity_magnitude += cfg.acceleration;
            velocity_magnitude = std::max(cfg.min_translational_velocity_magnitude,
                                          std::min(velocity_magnitude, cfg.max_translational_velocity_magnitude));
            // Obtain the unit vector to desired target in robot space and scale by magnitude
            Eigen::Vector3d velocity_target = walk_direct.Hrd.translation().normalized() * velocity_magnitude;

            // Set the angular velocity component of the velocity_target with the angular displacement and clamp
            auto heading        = std::atan2(walk_direct.Hrd.translation().y(), walk_direct.Hrd.translation().x());
            velocity_target.z() = utility::math::clamp(cfg.min_angular_velocity, heading, cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(velocity_target));
            log<NUClear::DEBUG>("Walking directly towards goal");
        });

        on<Provide<WalkAlign>>().then([this](const WalkAlign& walk_align) {
            // Decelerate to min velocity
            velocity_magnitude -= cfg.acceleration;
            velocity_magnitude = std::max(velocity_magnitude, cfg.min_translational_velocity_magnitude);

            // Obtain the unit vector to desired target in robot space and scale by magnitude
            Eigen::Vector3d velocity_target = walk_align.Hrd.translation().normalized() * velocity_magnitude;

            // Set the angular velocity component to the angular displacement and clamp
            velocity_target.z() = utility::math::clamp(cfg.min_angular_velocity,
                                                       mat_to_rpy_intrinsic(walk_align.Hrd.linear()).z(),
                                                       cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(velocity_target));
            log<NUClear::DEBUG>("Aligning towards goal");
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Provide<TurnAroundBall>>().then([this](const TurnAroundBall& turn_around_ball) {
            // Determine the direction of rotation
            int sign = turn_around_ball.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(cfg.pivot_ball_velocity_x,
                                                              sign * cfg.pivot_ball_velocity_y,
                                                              sign * cfg.pivot_ball_velocity)));
        });
    }
}  // namespace module::planning
