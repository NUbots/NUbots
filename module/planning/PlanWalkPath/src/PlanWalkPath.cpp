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

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/geometry/intersection.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Robots;
    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkProposal;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;
    using message::strategy::StandStill;

    using message::strategy::StandStill;

    using utility::math::angle::vectorToBearing;
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
            cfg.starting_velocity      = config["starting_velocity"].as<double>();
            cfg.acceleration           = config["acceleration"].as<double>();

            cfg.max_align_radius = config["max_align_radius"].as<double>();
            cfg.min_align_radius = config["min_align_radius"].as<double>();
            cfg.max_angle_error  = config["max_angle_error"].as<Expression>();
            cfg.min_angle_error  = config["min_angle_error"].as<Expression>();
            cfg.strafe_gain      = config["strafe_gain"].as<double>();

            // Backwards tuning
            cfg.max_strafe_angle        = config["max_strafe_angle"].as<Expression>();
            cfg.backward_buffer         = config["backward_buffer"].as<Expression>();
            cfg.max_aligned_angle       = config["max_aligned_angle"].as<Expression>();
            cfg.acceleration_multiplier = config["acceleration_multiplier"].as<double>();
            cfg.backwards_vector        = config["backwards_vector"].as<Expression>();

            // TurnOnSpot tuning
            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            // PivotAroundPoint tuning
            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();

            // Enhanced obstacle avoidance
            cfg.obstacle_radius           = config["obstacle_radius"].as<double>();
            cfg.obstacle_radius_with_ball = config["obstacle_radius_with_ball"].as<double>();
            cfg.ball_possession_threshold = config["ball_possession_threshold"].as<double>();
            cfg.cautious_velocity_scale   = config["cautious_velocity_scale"].as<double>();

            // Exponential smoothing configuration
            cfg.tau = config["tau"].as<Expression>();
            // Ensure safety for division by zero if tau is zero
            cfg.alpha = (cfg.tau.array() > 0.01)
                            .select((1.0 - (-1.0 / (UPDATE_FREQUENCY * cfg.tau.array())).exp()).matrix(),
                                    Eigen::Vector3d::Ones());
            cfg.one_minus_alpha = Eigen::Vector3d::Ones() - cfg.alpha;
            log<INFO>("Smoothing walk with time constant tau: (",
                      cfg.tau.x(),
                      ", ",
                      cfg.tau.y(),
                      ", ",
                      cfg.tau.z(),
                      ") corresponding to alpha: (",
                      cfg.alpha.x(),
                      ", ",
                      cfg.alpha.y(),
                      ", ",
                      cfg.alpha.z(),
                      ")");
        });

        on<Provide<WalkTo>, Optional<With<Robots>>, Optional<With<Ball>>, With<Sensors>>().then(
            [this](const WalkTo& walk_to,
                   const std::shared_ptr<const Robots>& robots,
                   const std::shared_ptr<const Ball>& ball,
                   const Sensors& sensors) {
                // Get all the parameters we need to calculate the walk path
                const auto& Hrd  = walk_to.Hrd;
                const auto& Hrw  = sensors.Hrw;
                const auto& rBWw = ball ? ball->rBWw : Eigen::Vector3d::Zero();

                // Decompose the target pose into position and orientation
                Eigen::Vector2d rDRr = Hrd.translation().head(2);
                // Calculate the angle between the robot and the target point (x, y)
                double angle_to_target = vectorToBearing(rDRr);
                // Calculate the angle between the robot and the final desired heading
                double angle_to_final_heading = vectorToBearing(Hrd.linear().col(0).head(2));

                // Determine if we have the ball (for more aggressive obstacle avoidance)
                const bool have_ball     = ball ? (Hrw * rBWw).norm() < cfg.ball_possession_threshold : false;
                bool other_robots_nearby = false;

                // If we can see any robots, check if they are in the way
                if (robots) {
                    std::vector<Eigen::Vector2d> all_obstacles{};
                    std::transform(robots->robots.begin(),
                                   robots->robots.end(),
                                   std::back_inserter(all_obstacles),
                                   [&](const auto& robot) { return (Hrw * robot.rRWw).head(2); });

                    // Sort obstacles based on distance from the robot
                    std::ranges::sort(all_obstacles, {}, &Eigen::Vector2d::squaredNorm);

                    // Use enhanced obstacle radius when we have the ball
                    const double obstacle_radius = have_ball ? cfg.obstacle_radius_with_ball : cfg.obstacle_radius;

                    // Get the obstacles in the way of the current path
                    const auto obstacles = get_obstacles(all_obstacles, rDRr, obstacle_radius);

                    // If there are obstacles in the way, walk around them
                    if (!obstacles.empty()) {
                        // Adjust the target direction to avoid obstacles
                        rDRr = adjust_target_direction_for_obstacles(rDRr, obstacles, obstacle_radius);
                        // Angle to the target point may have changed after adjusting for obstacles
                        angle_to_target = vectorToBearing(rDRr);
                        // Final heading should now point toward the original target point from the adjusted target
                        angle_to_final_heading = vectorToBearing(Hrd.translation().head(2) - rDRr);
                    }

                    // Finally, record if a robot is very close to us so we can later scale the velocity
                    other_robots_nearby =
                        !all_obstacles.empty() && all_obstacles.front().norm() < obstacle_radius * 1.5;
                }

                // Desired velocity magnitude and heading
                double desired_magnitude, desired_heading = angle_to_target;

                // Calculate the translational error between the robot and the target point (x, y)
                const double translational_error = rDRr.norm();

                // If we are far from the target point, accelerate and align ourselves towards it
                if (translational_error > cfg.max_align_radius) {
                    desired_magnitude = accelerate_to_target(desired_heading);
                }
                else {
                    // Add a buffer to prevent oscillation between forwards and backwards movement
                    const double max_strafe_angle =
                        is_walking_backwards ? cfg.max_strafe_angle - cfg.backward_buffer : cfg.max_strafe_angle;

                    const bool heading_aligned    = std::abs(angle_to_final_heading) < cfg.max_aligned_angle;
                    const bool large_strafe_angle = std::abs(angle_to_target) > max_strafe_angle;

                    // If we're aligned with the final heading, but the target angle is larger than we can strafe, then
                    // walk backwards first
                    if (heading_aligned && large_strafe_angle) {
                        rDRr              = walk_backwards();
                        desired_magnitude = velocity_magnitude;
                        desired_heading   = 0.0;
                    }
                    // Otherwise align ourselves with the target
                    else {
                        // What fraction through the alignment region we are
                        const double align_progress = std::clamp((cfg.max_align_radius - translational_error)
                                                                     / (cfg.max_align_radius - cfg.min_align_radius),
                                                                 0.0,
                                                                 1.0);
                        // What fraction to the target we are
                        const double strafe_error = translational_error / cfg.max_align_radius;
                        desired_magnitude         = strafe_to_target(strafe_error);
                        // Interpolate between angle to the target and desired heading when inside the alignment region
                        desired_heading =
                            (1.0 - align_progress) * angle_to_target + align_progress * angle_to_final_heading;
                    }
                }

                // Scale velocity when we have the ball and there are nearby obstacles
                if (have_ball && other_robots_nearby) {
                    desired_magnitude *= cfg.cautious_velocity_scale;
                }

                // Calculate the target velocity
                const Eigen::Vector2d desired_translational_velocity = desired_magnitude * rDRr.normalized();
                Eigen::Vector3d velocity_target(desired_translational_velocity.x(),
                                                desired_translational_velocity.y(),
                                                desired_heading);

                // Limit the velocity to the maximum translational and angular velocity
                velocity_target = constrain_velocity(velocity_target);

                // Emit the walk task with the calculated velocities
                emit<Task>(std::make_unique<WalkProposal>(velocity_target));

                // Emit debugging information for visualization and monitoring
                auto debug_information              = std::make_unique<WalkToDebug>();
                Eigen::Isometry3d _Hrd              = Eigen::Isometry3d::Identity();
                _Hrd.translation().head(2)          = rDRr;
                _Hrd.linear()                       = rpy_intrinsic_to_mat(Eigen::Vector3d(0.0, 0.0, desired_heading));
                debug_information->Hrd              = _Hrd;
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
            emit<Task>(std::make_unique<WalkProposal>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Provide<PivotAroundPoint>>().then([this](const PivotAroundPoint& pivot_around_point) {
            // Determine the direction of rotation
            int sign = pivot_around_point.clockwise ? -1 : 1;
            // Turn around the ball
            emit<Task>(std::make_unique<WalkProposal>(Eigen::Vector3d(cfg.pivot_ball_velocity_x,
                                                                      sign * cfg.pivot_ball_velocity_y,
                                                                      sign * cfg.pivot_ball_velocity)));
        });

        // Intercept Walk commands and apply smoothing
        on<Provide<WalkProposal>, Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this](const WalkProposal&
                                                                                                        walk) {
            // Apply exponential smoothing to the walk command
            Eigen::Vector3d smoothed_command =
                walk.velocity_target.cwiseProduct(cfg.alpha) + previous_walk_command.cwiseProduct(cfg.one_minus_alpha);

            Eigen::Vector3d smooth_diff = smoothed_command - walk.velocity_target;

            // Visualise the walk path in NUsight
            emit(graph("Walk Proposal", walk.velocity_target.x(), walk.velocity_target.y(), walk.velocity_target.z()));
            emit(graph("Smoothed Walk Command", smoothed_command.x(), smoothed_command.y(), smoothed_command.z()));
            emit(graph("Walk Smoothing Difference", smooth_diff.x(), smooth_diff.y(), smooth_diff.z()));

            // Forward the smoothed command to the actual Walk skill
            emit<Task>(std::make_unique<Walk>(smoothed_command));

            // Store for next iteration
            previous_walk_command = smoothed_command;
        });
    }

    double PlanWalkPath::strafe_to_target(const double error) {
        // If we are walking backwards, change direction
        is_walking_backwards = false;
        log<DEBUG>("[PlanWalkPath::strafe_to_target] is_walking_backwards set to false, velocity_magnitude = ",
                   velocity_magnitude);
        // "Accelerate", assuring velocity is always positive
        velocity_magnitude = std::max(velocity_magnitude, cfg.starting_velocity);
        // "Proportional control" to strafe towards the target inside align radius
        return cfg.strafe_gain * error;
    }

    Eigen::Vector2d PlanWalkPath::walk_backwards() {
        // Set flag so that we have a backwards buffer
        is_walking_backwards = true;
        log<DEBUG>("[PlanWalkPath::walk_backwards] is_walking_backwards set to true, velocity_magnitude = ",
                   velocity_magnitude);

        // Walk on spot, then walk backwards
        velocity_magnitude += std::min(velocity_magnitude * cfg.acceleration_multiplier, cfg.max_velocity_magnitude);

        // Step backwards while keeping the forward direction
        return cfg.backwards_vector;
    }

    double PlanWalkPath::accelerate_to_target(double desired_heading) {
        // If we are walking backwards, change direction
        is_walking_backwards = false;
        log<DEBUG>("[PlanWalkPath::accelerate_to_target] is_walking_backwards set to false, velocity_magnitude = ",
                   velocity_magnitude);
        // "Accelerate", assuring velocity is always positive
        velocity_magnitude = std::max(velocity_magnitude + cfg.acceleration, 0.3);
        // Limit the velocity magnitude to the maximum velocity
        velocity_magnitude = std::min(velocity_magnitude, cfg.max_velocity_magnitude);
        // Scale the velocity by angle error to have robot rotate on spot when far away and not facing
        // target [0 at max_angle_error, linearly interpolate between, 1 at min_angle_error]
        const double angle_error_gain =
            std::clamp((cfg.max_angle_error - std::abs(desired_heading)) / (cfg.max_angle_error - cfg.min_angle_error),
                       0.0,
                       1.0);
        return angle_error_gain * velocity_magnitude;
    }

    Eigen::Vector2d PlanWalkPath::adjust_target_direction_for_obstacles(Eigen::Vector2d rDRr,
                                                                        const std::vector<Eigen::Vector2d>& obstacles,
                                                                        double obstacle_radius) {
        log<DEBUG>("Path planning around", obstacles.size(), "obstacles.");

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
        leftmost  = leftmost - perp_direction * obstacle_radius;
        rightmost = rightmost + perp_direction * obstacle_radius;

        // Determine if leftmost or rightmost position has a quicker path
        const double left_distance  = (leftmost - rDRr).norm() + leftmost.norm();
        const double right_distance = (rightmost - rDRr).norm() + rightmost.norm();

        // Scale the perpendicular vector by obstacle_radius to ensure clearance
        return left_distance < right_distance ? leftmost : rightmost;
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
                                                                   const Eigen::Vector2d& rDRr,
                                                                   double obstacle_radius) {
        // If there are no obstacles, return an empty group
        if (all_obstacles.empty())
            return {};

        // Find the first obstacle in the way
        auto it = std::ranges::find_if(all_obstacles, [&](const Eigen::Vector2d& obstacle) {
            const bool in_front        = rDRr.normalized().dot(obstacle.normalized()) > 0.0;
            const bool before_target   = obstacle.norm() < rDRr.norm();
            const bool close_to_target = (obstacle - rDRr).norm() < obstacle_radius;
            const bool intersects =
                intersection_line_and_circle(Eigen::Vector2d::Zero(), rDRr, obstacle, obstacle_radius);

            return in_front && before_target && intersects && !close_to_target;
        });

        // No obstacles in the way
        if (it == all_obstacles.end())
            return {};

        std::vector<Eigen::Vector2d> avoid_obstacles{*it};

        // Find any obstacles close to our first obstacle, as the robot needs to go around the whole group
        for (const Eigen::Vector2d& obstacle : all_obstacles) {
            // If the obstacle is close to the group, add it to the group
            if (std::ranges::any_of(avoid_obstacles, [&](const Eigen::Vector2d& ao) {
                    // 3 represents two obstacles and the robot
                    return (obstacle - ao).norm() < obstacle_radius * 3;
                })) {
                avoid_obstacles.push_back(obstacle);
            }
        }

        return avoid_obstacles;
    }

}  // namespace module::planning
