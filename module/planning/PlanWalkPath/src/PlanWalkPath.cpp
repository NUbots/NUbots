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

#include <ranges>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/geometry/intersection.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Robots;
    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkProposal;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;

    using utility::math::angle::vector_to_bearing;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::math::geometry::intersection_line_and_circle;
    using utility::nusight::graph;
    using utility::support::Expression;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // WalkTo controller tuning
            cfg.walk_to.max_velocity          = config["max_velocity"].as<Expression>();
            cfg.walk_to.max_backward_velocity = config["max_backward_velocity"].as<double>();
            cfg.walk_to.k_translation         = config["k_translation"].as<double>();
            cfg.walk_to.k_theta               = config["k_theta"].as<double>();
            cfg.walk_to.max_align_radius      = config["max_align_radius"].as<double>();
            cfg.walk_to.min_align_radius      = config["min_align_radius"].as<double>();
            cfg.walk_to.max_angle_error       = config["max_angle_error"].as<Expression>();
            cfg.walk_to.min_angle_error       = config["min_angle_error"].as<Expression>();

            // RL policy command dead-zone
            cfg.min_velocity   = config["min_velocity"].as<Expression>();
            cfg.zero_tolerance = config["zero_tolerance"].as<Expression>();

            // TurnOnSpot tuning
            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            // PivotAroundPoint tuning
            cfg.pivot_angular_velocity = config["pivot_angular_velocity"].as<double>();
            cfg.pivot_radius           = config["pivot_radius"].as<double>();
            cfg.pivot_forward_velocity = config["pivot_forward_velocity"].as<double>();

            cfg.obstacle_radius = config["obstacle_radius"].as<double>();

            // Exponential smoothing configuration
            cfg.tau = config["tau"].as<Expression>();
            // Ensure safety for division by zero if tau is zero
            cfg.alpha = (cfg.tau.array() > 0.01)
                            .select((1.0 - (-1.0 / (UPDATE_FREQUENCY * cfg.tau.array())).exp()).matrix(),
                                    Eigen::Vector3d::Ones());
            cfg.one_minus_alpha = Eigen::Vector3d::Ones() - cfg.alpha;
        });

        on<Trigger<Stability>>().then([this](const Stability& new_stability) {
            // If transitioning from FALLEN/STANDING to DYNAMIC, reset the smoothed walk command
            if ((stability == Stability::FALLEN && new_stability == Stability::DYNAMIC)
                || (stability == Stability::STANDING && new_stability == Stability::DYNAMIC)) {
                previous_walk_command = Eigen::Vector3d::Zero();
                log<DEBUG>("Resetting smoothed walk command");
            }

            stability = new_stability;
        });

        on<Provide<WalkTo>, Optional<With<Robots>>, With<Sensors>>().then(
            [this](const WalkTo& walk_to, const std::shared_ptr<const Robots>& robots, const Sensors& sensors) {
                // Get all the parameters we need to calculate the walk path
                const auto& Hrd = walk_to.Hrd;
                const auto& Hrw = sensors.Hrw;

                // Decompose the target pose into position and orientation
                Eigen::Vector2d rDRr = Hrd.translation().head(2);
                // Calculate the angle between the robot and the target point (x, y)
                double angle_to_target = vector_to_bearing(rDRr);
                // Calculate the angle between the robot and the final desired heading
                double angle_to_final_heading = vector_to_bearing(Hrd.linear().col(0).head(2));

                // If there are robots, check if they're in the way
                if (robots) {
                    // Get the positions of all robots in the world
                    std::vector<Eigen::Vector2d> all_obstacles{};
                    for (const auto& robot : robots->robots) {
                        all_obstacles.emplace_back((Hrw * robot.rRWw).head(2));
                    }
                    // Sort obstacles based on distance from the robot
                    std::ranges::sort(all_obstacles, {}, &Eigen::Vector2d::squaredNorm);
                    // Get the obstacles in the way of the current path
                    const auto obstacles = get_obstacles(all_obstacles, rDRr);

                    // If there are obstacles in the way, walk around them
                    if (!obstacles.empty()) {
                        log<DEBUG>("Path planning around", obstacles.size(), "obstacles.");

                        // Calculate a perpendicular vector to the direction of the target point
                        const Eigen::Vector2d perp(rDRr.normalized().y(), -rDRr.normalized().x());

                        // Projection onto the perpendicular vector tells us how "out of the way" an obstacle is
                        auto proj = [&perp](const Eigen::Vector2d& v) { return v.dot(perp); };

                        // Most positive and negative projections of the obstacles to find the two candidate target
                        // paths. Target is the vector to the obstacle, adjusted outwards by the obstacle radius, note
                        // this can cause us to slightly clip into the obstacle radius, especially if the obstacle is
                        // very close to the target
                        const Eigen::Vector2d left =
                            *std::ranges::min_element(obstacles, {}, proj) - perp * cfg.obstacle_radius;
                        const Eigen::Vector2d right =
                            *std::ranges::max_element(obstacles, {}, proj) + perp * cfg.obstacle_radius;

                        // Total path length by traversing the triangle from the robot->obstacle->original target
                        auto path = [&rDRr](const Eigen::Vector2d& v) { return (v - rDRr).norm() + v.norm(); };

                        // Take the shorter path
                        rDRr = path(left) < path(right) ? left : right;

                        // Angle to the target point may have changed after adjusting for obstacles
                        angle_to_target = vector_to_bearing(rDRr);
                        // Final heading should now point toward the original target point from the adjusted target
                        angle_to_final_heading = vector_to_bearing(Hrd.translation().head(2) - rDRr);
                    }
                }

                // Calculate the target velocity
                const auto result = walk_path::walk_to_velocity(rDRr, angle_to_final_heading, cfg.walk_to);

                // Limit the velocity to the maximum translational and angular velocity
                const Eigen::Vector3d velocity_target =
                    walk_path::constrain_velocity(result.velocity,
                                                  cfg.walk_to.max_velocity,
                                                  cfg.walk_to.max_backward_velocity);

                // Emit the walk task with the calculated velocities
                emit<Task>(std::make_unique<WalkProposal>(velocity_target));

                // Emit debugging information for visualisation and monitoring
                auto debug_information                       = std::make_unique<WalkToDebug>();
                debug_information->Hrd.translation().head(2) = rDRr;
                debug_information->Hrd.linear() =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(0.0, 0.0, result.desired_heading));
                debug_information->min_align_radius        = cfg.walk_to.min_align_radius;
                debug_information->max_align_radius        = cfg.walk_to.max_align_radius;
                debug_information->min_angle_error         = cfg.walk_to.min_angle_error;
                debug_information->max_angle_error         = cfg.walk_to.max_angle_error;
                debug_information->angle_to_target         = angle_to_target;
                debug_information->angle_to_final_heading  = angle_to_final_heading;
                debug_information->translational_error     = rDRr.norm();
                debug_information->velocity_target         = velocity_target;
                emit(debug_information);
            });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            const Eigen::Vector3d turn_vector(cfg.rotate_velocity_x,
                                              sign * cfg.rotate_velocity_y,
                                              sign * cfg.rotate_velocity);
            emit<Task>(std::make_unique<WalkProposal>(turn_vector));
        });

        on<Provide<PivotAroundPoint>>().then([this](const PivotAroundPoint& pivot_around_point) {
            // Determine the direction of rotation
            int sign = pivot_around_point.clockwise ? -1 : 1;
            // Orbit a point at pivot_radius directly ahead while facing it: vy = -vtheta * radius
            const double angular_velocity = sign * cfg.pivot_angular_velocity;
            const Eigen::Vector3d pivot_vector(cfg.pivot_forward_velocity,
                                               -angular_velocity * cfg.pivot_radius,
                                               angular_velocity);
            emit<Task>(std::make_unique<WalkProposal>(pivot_vector));
        });

        // Intercept Walk commands, apply smoothing and dead-zone compensation
        on<Provide<WalkProposal>, Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this](const WalkProposal&
                                                                                                        walk) {
            // Apply exponential smoothing to the walk command
            Eigen::Vector3d smoothed_command =
                walk.velocity_target.cwiseProduct(cfg.alpha) + previous_walk_command.cwiseProduct(cfg.one_minus_alpha);

            Eigen::Vector3d smooth_diff = smoothed_command - walk.velocity_target;

            // Apply the dead-zone after smoothing so the command sent is never in the unresponsive band
            const Eigen::Vector3d walk_command = walk_path::apply_dead_zone(smoothed_command,
                                                                            cfg.min_velocity,
                                                                            cfg.zero_tolerance,
                                                                            cfg.walk_to.max_velocity);

            // Visualise the walk path in NUsight
            emit(graph("Walk Proposal", walk.velocity_target.x(), walk.velocity_target.y(), walk.velocity_target.z()));
            emit(graph("Smoothed Walk Command", smoothed_command.x(), smoothed_command.y(), smoothed_command.z()));
            emit(graph("Walk Smoothing Difference", smooth_diff.x(), smooth_diff.y(), smooth_diff.z()));
            emit(graph("Walk Command", walk_command.x(), walk_command.y(), walk_command.z()));

            // Store the pre-dead-zone command so the smoother dynamics stay clean
            previous_walk_command = smoothed_command;

            // Forward the smoothed command to the actual Walk skill
            emit<Task>(std::make_unique<Walk>(walk_command));
        });
    }

    const std::vector<Eigen::Vector2d> PlanWalkPath::get_obstacles(const std::vector<Eigen::Vector2d>& all_obstacles,
                                                                   const Eigen::Vector2d& rDRr) {
        // If there are no obstacles, return an empty group
        if (all_obstacles.empty()) {
            return {};
        }

        // Find the first obstacle in the way
        auto it = std::ranges::find_if(all_obstacles, [&](const Eigen::Vector2d& obstacle) {
            const bool in_front        = rDRr.normalized().dot(obstacle.normalized()) > 0.0;
            const bool before_target   = obstacle.norm() < rDRr.norm();
            const bool close_to_target = (obstacle - rDRr).norm() < cfg.obstacle_radius;
            const bool intersects =
                intersection_line_and_circle(Eigen::Vector2d::Zero(), rDRr, obstacle, cfg.obstacle_radius);

            return in_front && before_target && intersects && !close_to_target;
        });

        // If no obstacles are in the way, return the empty group
        if (it == all_obstacles.end())
            return {};

        std::vector<Eigen::Vector2d> avoid_obstacles{*it};

        // Find any obstacles close to our first obstacle, as the robot needs to go around the whole group
        for (const Eigen::Vector2d& obstacle : all_obstacles) {
            // If the obstacle is close to the group, add it to the group
            if (std::ranges::any_of(avoid_obstacles, [&](const Eigen::Vector2d& ao) {
                    return (obstacle - ao).norm()
                           < cfg.obstacle_radius * 3;  // 3 represents two obstacles and the robot
                })) {
                avoid_obstacles.push_back(obstacle);
            }
        }

        return avoid_obstacles;
    }

}  // namespace module::planning
