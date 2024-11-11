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

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/geometry/intersection.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Robots;
    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;
    using message::strategy::StandStill;

    using message::strategy::StandStill;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::math::geometry::intersection_line_and_circle;
    using utility::nusight::graph;
    using utility::support::Expression;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // WalkTo tuning
            cfg.max_translational_velocity_x = config["max_translational_velocity_x"].as<double>();
            cfg.max_translational_velocity_y = config["max_translational_velocity_y"].as<double>();
            cfg.max_velocity_magnitude = std::max(cfg.max_translational_velocity_x, cfg.max_translational_velocity_y);
            cfg.max_angular_velocity   = config["max_angular_velocity"].as<double>();
            cfg.acceleration           = config["acceleration"].as<double>();

            cfg.max_align_radius = config["max_align_radius"].as<double>();
            cfg.min_align_radius = config["min_align_radius"].as<double>();
            cfg.max_angle_error  = config["max_angle_error"].as<Expression>();
            cfg.min_angle_error  = config["min_angle_error"].as<Expression>();
            cfg.strafe_gain      = config["strafe_gain"].as<double>();
            cfg.max_strafe_angle = config["max_strafe_angle"].as<Expression>();

            // TurnOnSpot tuning
            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            // PivotAroundPoint tuning
            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();

            cfg.obstacle_radius = config["obstacle_radius"].as<double>();
        });

        on<Provide<WalkTo>, Optional<With<Robots>>, With<Sensors>>().then(
            [this](const WalkTo& walk_to, const std::shared_ptr<const Robots>& robots, const Sensors& sensors) {
                // Decompose the target pose into position and orientation
                Eigen::Vector2d rDRr = walk_to.Hrd.translation().head(2);
                // Calculate the angle between the robot and the target point (x, y)
                const double angle_to_target = std::atan2(rDRr.y(), rDRr.x());
                // Calculate the angle between the robot and the final desired heading
                double angle_to_final_heading =
                    std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());

                // If there are robots, check if there are obstacles in the way
                if (robots != nullptr) {
                    // Get the positions of all robots in the world
                    std::vector<Eigen::Vector2d> all_obstacles{};
                    for (const auto& robot : robots->robots) {
                        all_obstacles.emplace_back((sensors.Hrw * robot.rRWw).head(2));
                    }
                    // Sort obstacles based on distance from the robot
                    std::sort(all_obstacles.begin(),
                              all_obstacles.end(),
                              [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return a.norm() < b.norm(); });

                    // Get the obstacles in the way of the current path
                    const std::vector<Eigen::Vector2d> obstacles = get_obstacles(all_obstacles, rDRr);

                    // If there are obstacles in the way, walk around them
                    if (!obstacles.empty()) {
                        log<NUClear::DEBUG>("Path planning around", obstacles.size(), "obstacles.");

                        // Calculate a perpendicular vector to the direction of the target point
                        const Eigen::Vector2d perp_direction(rDRr.normalized().y(), -rDRr.normalized().x());

                        // Find leftmost and rightmost and see which is a better path
                        Eigen::Vector2d leftmost  = obstacles[0];
                        Eigen::Vector2d rightmost = obstacles[0];
                        for (const auto& pos : obstacles) {
                            leftmost  = pos.dot(perp_direction) < leftmost.dot(perp_direction) ? pos : leftmost;
                            rightmost = pos.dot(perp_direction) > rightmost.dot(perp_direction) ? pos : rightmost;
                        }

                        // Add on the obstacle radius
                        leftmost  = leftmost - perp_direction * cfg.obstacle_radius;
                        rightmost = rightmost + perp_direction * cfg.obstacle_radius;

                        // Determine if leftmost or rightmost position has a quicker path
                        const double left_distance  = (leftmost - rDRr).norm() + leftmost.norm();
                        const double right_distance = (rightmost - rDRr).norm() + rightmost.norm();

                        // Scale the perpendicular vector by obstacle_radius to ensure clearance
                        rDRr = left_distance < right_distance ? leftmost : rightmost;

                        // Override the heading when walking around obstacles
                        angle_to_final_heading = std::atan2(rDRr.y(), rDRr.x());
                    }
                }

                // Calculate the translational error between the robot and the target point (x, y)
                const double translational_error = rDRr.norm();

                // Linearly interpolate between angle to the target and desired heading when inside the align radius
                // region
                const double translation_progress = std::clamp(
                    (cfg.max_align_radius - translational_error) / (cfg.max_align_radius - cfg.min_align_radius),
                    0.0,
                    1.0);
                double desired_heading =
                    (1 - translation_progress) * angle_to_target + translation_progress * angle_to_final_heading;

                double desired_velocity_magnitude = 0.0;
                if (translational_error > cfg.max_align_radius) {
                    // "Accelerate"
                    velocity_magnitude += cfg.acceleration;
                    // Limit the velocity magnitude to the maximum velocity
                    velocity_magnitude = std::min(velocity_magnitude, cfg.max_velocity_magnitude);
                    // Scale the velocity by angle error to have robot rotate on spot when far away and not facing
                    // target [0 at max_angle_error, linearly interpolate between, 1 at min_angle_error]
                    const double angle_error_gain = std::clamp(
                        (cfg.max_angle_error - std::abs(desired_heading)) / (cfg.max_angle_error - cfg.min_angle_error),
                        0.0,
                        1.0);
                    desired_velocity_magnitude = angle_error_gain * velocity_magnitude;
                    log<NUClear::DEBUG>("Normal, angle error gain: ",
                                        angle_error_gain,
                                        "desired_velocity_magnitude: ",
                                        desired_velocity_magnitude);
                }
                else {
                    // Normalise error between [0, 1] inside align radius
                    const double error = translational_error / cfg.max_align_radius;
                    // "Decelerate"
                    velocity_magnitude -= cfg.acceleration;
                    // If we are aligned with the final heading, we are close to the target
                    // and the angle to the target is too large, step backwards
                    if ((std::abs(angle_to_final_heading < 0.2) && std::abs(angle_to_target) > cfg.max_strafe_angle)) {
                        // Limit the velocity to -1
                        velocity_magnitude = std::max(velocity_magnitude, -cfg.max_velocity_magnitude);
                        log<NUClear::DEBUG>("Backwards velocity magnitude : ", velocity_magnitude);
                        // Step backwards, but keep the direction forward
                        rDRr = Eigen::Vector2d(-1, rDRr.y());
                        // Do not rotate the robot
                        desired_heading = 0.0;
                        // Accelerate away from the target, faster when closer
                        desired_velocity_magnitude = 1 - (cfg.strafe_gain * error * velocity_magnitude);
                        // TODO: adjust velocity magnitude?
                    }
                    // Go towards target
                    else {
                        // Limit the velocity to zero
                        velocity_magnitude = std::max(velocity_magnitude, 0.0);
                        // "Proportional control" to strafe towards the target inside align radius
                        desired_velocity_magnitude = cfg.strafe_gain * error;
                    }
                }

                // Calculate the target velocity
                const Eigen::Vector2d desired_translational_velocity = desired_velocity_magnitude * rDRr.normalized();
                Eigen::Vector3d velocity_target;
                velocity_target << desired_translational_velocity, desired_heading;

                // Limit the velocity to the maximum translational and angular velocity
                velocity_target = constrain_velocity(velocity_target);

                // Emit the walk task with the calculated velocities
                emit<Task>(std::make_unique<Walk>(velocity_target));

                // Emit debugging information for visualization and monitoring
                auto debug_information              = std::make_unique<WalkToDebug>();
                Eigen::Isometry3d Hrd               = Eigen::Isometry3d::Identity();
                Hrd.translation().head(2)           = rDRr;
                Hrd.linear()                        = rpy_intrinsic_to_mat(Eigen::Vector3d(0.0, 0.0, desired_heading));
                debug_information->Hrd              = Hrd;
                debug_information->min_align_radius = cfg.min_align_radius;
                debug_information->max_align_radius = cfg.max_align_radius;
                debug_information->min_angle_error  = cfg.min_angle_error;
                debug_information->max_angle_error  = cfg.max_angle_error;
                debug_information->angle_to_target  = angle_to_target;
                debug_information->angle_to_final_heading = angle_to_final_heading;
                debug_information->translational_error    = translational_error;
                debug_information->velocity_target        = velocity_target;
                emit(debug_information);
            });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Provide<PivotAroundPoint>>().then([this](const PivotAroundPoint& pivot_around_point) {
            // Determine the direction of rotation
            int sign = pivot_around_point.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(cfg.pivot_ball_velocity_x,
                                                              sign * cfg.pivot_ball_velocity_y,
                                                              sign * cfg.pivot_ball_velocity)));
        });
    }

    Eigen::Vector3d PlanWalkPath::constrain_velocity(const Eigen::Vector3d& v) {
        Eigen::Vector2d translational_velocity = v.head<2>();
        // If either translational component exceeds the limit, scale the vector to fit within the limits
        if (std::abs(v.x()) >= cfg.max_translational_velocity_x
            || std::abs(v.y()) >= cfg.max_translational_velocity_y) {
            double sx = v.x() != 0.0 ? cfg.max_translational_velocity_x / std::abs(v.x()) : 0.0;
            double sy = v.y() != 0.0 ? cfg.max_translational_velocity_y / std::abs(v.y()) : 0.0;
            // Select the minimum scale factor to ensure neither limit is exceeded but direction is maintained
            double s               = std::min(sx, sy);
            translational_velocity = v.head<2>() * s;
        }
        // Ensure the angular velocity is within the limits
        double angular_velocity = std::clamp(v.z(), -cfg.max_angular_velocity, cfg.max_angular_velocity);
        return Eigen::Vector3d(translational_velocity.x(), translational_velocity.y(), angular_velocity);
    }

    const std::vector<Eigen::Vector2d> PlanWalkPath::get_obstacles(const std::vector<Eigen::Vector2d>& all_obstacles,
                                                                   const Eigen::Vector2d& rDRr) {
        // If there are no obstacles, return an empty group
        if (all_obstacles.empty()) {
            return {};
        }

        // The obstacles in the way of our current path
        std::vector<Eigen::Vector2d> avoid_obstacles{};

        // Find the first obstacle in the way
        for (const auto& obstacle : all_obstacles) {
            // Check if the obstacle is in front of the robot
            const bool in_front = rDRr.normalized().dot(obstacle.normalized()) > 0.0;
            // Check if the obstacle is closer than the target point
            const bool closer = obstacle.norm() < rDRr.norm();
            // Check if the obstacle intersects with the path
            const bool intersects =
                intersection_line_and_circle(Eigen::Vector2d::Zero(), rDRr, obstacle, cfg.obstacle_radius);
            // Check if the obstacle is close to the target position
            const bool close_to_target = (obstacle - rDRr).norm() < cfg.obstacle_radius;

            // Check if obstacle intersects with the path
            if (in_front && closer && intersects && !close_to_target) {
                avoid_obstacles.push_back(obstacle);
                break;
            }
        }

        // If no obstacles are in the way, return the empty group
        if (avoid_obstacles.empty()) {
            return avoid_obstacles;
        }

        // Find any obstacles close to our first obstacle, as the robot needs to go around the whole group
        for (const auto& obstacle : all_obstacles) {
            // If the obstacle is close to the group, add it to the group
            // 3 represents two obstacles and the robot
            for (const auto& avoid_obstacle : avoid_obstacles) {
                if ((obstacle - avoid_obstacle).norm() < cfg.obstacle_radius * 3) {
                    avoid_obstacles.push_back(obstacle);
                    break;
                }
            }
        }

        return avoid_obstacles;
    }

}  // namespace module::planning
