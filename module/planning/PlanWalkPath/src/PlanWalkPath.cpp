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
#include "message/localisation/Ball.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;
    using message::strategy::StandStill;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    double g(const double x, const double y, const double x0, const double y0) {
        double distance = std::pow(x - x0, 2) + std::pow(y - y0, 2);
        return 1 / (distance * distance);
    }

    double g_target(const double x, const double y, const double x0, const double y0) {
        return std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2));
    }

    // https://www.desmos.com/calculator/exmrdiugxn
    Eigen::Vector2d PlanWalkPath::vector_field(const Eigen::Vector2d& target_position,
                                               const double heading,
                                               std::vector<Eigen::Vector2d> obstacles) {
        double x = 0.0;
        double y = 0.0;

        // Attraction of the target position
        Eigen::Vector2d vector_field_direction(
            (target_position.x() - x) * g(x, y, target_position.x(), target_position.y()) * cfg.target_strength,
            (target_position.y() - y) * g(x, y, target_position.x(), target_position.y()) * cfg.target_strength);

        log<NUClear::INFO>("Target difference",
                           (target_position.x() - x) * g_target(x, y, target_position.x(), target_position.y()),
                           (target_position.y() - y) * g_target(x, y, target_position.x(), target_position.y()));

        // Create point next to target position at given heading
        double x_target = target_position.x() + std::cos(heading);
        double y_target = target_position.y() + std::sin(heading);

        // Repulse the heading point
        vector_field_direction.x() += ((x - x_target) * g(x, y, x_target, y_target)) * cfg.heading_strength;
        vector_field_direction.y() += ((y - y_target) * g(x, y, x_target, y_target)) * cfg.heading_strength;

        log<NUClear::INFO>("Heading difference",
                           (x - x_target) * g(x, y, x_target, y_target),
                           (y - y_target) * g(x, y, x_target, y_target));

        // Repulsion of the obstacles
        for (const auto& obstacle : obstacles) {
            vector_field_direction.x() +=
                ((x - obstacle.x()) * g(x, y, obstacle.x(), obstacle.y())) * cfg.obstacle_strength;
            vector_field_direction.y() +=
                ((y - obstacle.y()) * g(x, y, obstacle.x(), obstacle.y())) * cfg.obstacle_strength;

            log<NUClear::INFO>("Obstacle difference",
                               (x - obstacle.x()) * g(x, y, obstacle.x(), obstacle.y()),
                               (y - obstacle.y()) * g(x, y, obstacle.x(), obstacle.y()));
        }

        // Repulsion of the walls
        // vector_field_direction.x() -=
        //     cfg.bounds_strength
        //     * (std::abs(x - 4.5) * utility::math::sgn(x - 4.5) + std::abs(x + 4.5) * utility::math::sgn(x + 4.5));
        // vector_field_direction.y() -=
        //     cfg.bounds_strength
        //     * (std::abs(y - 3.0) * utility::math::sgn(y - 3.0) + std::abs(y + 3.0) * utility::math::sgn(y + 3.0));

        log<NUClear::INFO>(
            "Bounds difference",
            cfg.bounds_strength
                * (std::abs(x - 4.5) * utility::math::sgn(x - 4.5) + std::abs(x + 4.5) * utility::math::sgn(x + 4.5)),
            cfg.bounds_strength
                * (std::abs(y - 3.0) * utility::math::sgn(y - 3.0) + std::abs(y + 3.0) * utility::math::sgn(y + 3.0)));

        log<NUClear::INFO>("Vector Field", vector_field_direction.transpose(), obstacles.size());

        return vector_field_direction;
    }

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // WalkTo tuning
            cfg.max_translational_velocity_x = config["max_translational_velocity_x"].as<double>();
            cfg.max_translational_velocity_y = config["max_translational_velocity_y"].as<double>();
            max_velocity_magnitude   = std::max(cfg.max_translational_velocity_x, cfg.max_translational_velocity_y);
            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.max_acceleration     = config["max_acceleration"].as<double>();

            cfg.max_align_radius = config["max_align_radius"].as<double>();
            cfg.min_align_radius = config["min_align_radius"].as<double>();
            cfg.max_angle_error  = config["max_angle_error"].as<Expression>();
            cfg.min_angle_error  = config["min_angle_error"].as<Expression>();
            cfg.strafe_gain      = config["strafe_gain"].as<double>();

            // TurnOnSpot tuning
            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            // PivotAroundPoint tuning
            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();

            cfg.target_strength   = config["target_strength"].as<double>();
            cfg.heading_strength  = config["heading_strength"].as<double>();
            cfg.obstacle_strength = config["obstacle_strength"].as<double>();
            cfg.bounds_strength   = config["bounds_strength"].as<double>();

            // Emit empty Robots in case we don't get any
            emit(std::make_unique<Robots>());
        });

        on<Start<WalkTo>>().then([this] {
            log<NUClear::DEBUG>("Starting walk to task");
            // Reset the velocity magnitude to zero
            velocity_command = Eigen::Vector3d::Zero();
        });

        on<Provide<WalkTo>, Uses<Walk>, Optional<With<Robots>>, With<Sensors>>().then(
            [this](const WalkTo& walk_to,
                   const Uses<Walk>& walk,
                   const std::shared_ptr<const Robots>& robots,
                   const Sensors& sensors) {
                std::vector<Eigen::Vector2d> obstacles{};
                if (robots != nullptr) {
                    for (const auto& robot : robots->robots) {
                        obstacles.push_back((sensors.Hrw * robot.rRWw).head(2));
                    }
                }

                // Decompose the target pose into position and orientation
                Eigen::Vector2d rDRr = walk_to.Hrd.translation().head(2);

                // Calculate the angle between the robot and the final desired heading
                double angle_to_final_heading =
                    std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());

                // Get the vector field desired velocity
                Eigen::Vector2d desired_velocity_target = vector_field(rDRr, angle_to_final_heading, obstacles);
                emit(graph("desired_velocity_target ", desired_velocity_target.x(), desired_velocity_target.y()));

                // Calculate the angle between the robot and the target velocity (x, y)
                double angle_to_target = std::atan2(desired_velocity_target.y(), desired_velocity_target.x());

                // Calculate the translational error between the robot and the target point (x, y)
                double translational_error = rDRr.norm();

                // Linearly interpolate between angle to the target and desired heading when inside the align radius
                // region
                double translation_progress = std::clamp(
                    (cfg.max_align_radius - translational_error) / (cfg.max_align_radius - cfg.min_align_radius),
                    0.0,
                    1.0);
                double desired_heading =
                    (1 - translation_progress) * angle_to_target + translation_progress * angle_to_final_heading;
                // Set the desired heading as the velocity command
                velocity_command.z() = desired_heading;

                double desired_velocity_magnitude = 0;
                if (translational_error > cfg.max_align_radius) {
                    // Scale the velocity by angle error to have robot rotate on spot when far away and not facing
                    // target [0 at max_angle_error, linearly interpolate between, 1 at min_angle_error]
                    double angle_error_gain = std::clamp(
                        (cfg.max_angle_error - std::abs(desired_heading)) / (cfg.max_angle_error - cfg.min_angle_error),
                        0.0,
                        1.0);
                    velocity_command.x() = std::min(angle_error_gain * desired_velocity_target.x(),
                                                    velocity_command.x() + cfg.max_acceleration);
                    velocity_command.y() = std::min(angle_error_gain * desired_velocity_target.y(),
                                                    velocity_command.y() + cfg.max_acceleration);
                }
                else {
                    // "Proportional control" to strafe towards the target inside align radius
                    // Normalise error between [0, 1] inside align radius
                    velocity_command.x() = cfg.strafe_gain * rDRr.x() / cfg.max_align_radius;
                    velocity_command.y() = cfg.strafe_gain * rDRr.y() / cfg.max_align_radius;
                }


                // Limit the velocity to the maximum translational and angular velocity
                velocity_command = constrain_velocity(velocity_command,
                                                      cfg.max_translational_velocity_x,
                                                      cfg.max_translational_velocity_y,
                                                      cfg.max_angular_velocity);

                // Emit the walk task with the calculated velocities
                emit<Task>(std::make_unique<Walk>(velocity_command));

                // Emit debugging information for visualization and monitoring
                auto debug_information              = std::make_unique<WalkToDebug>();
                Eigen::Isometry3d Hrd               = Eigen::Isometry3d::Identity();
                Hrd.translation().head(2)           = rDRr;
                Hrd.linear()                        = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, desired_heading));
                debug_information->Hrd              = Hrd;
                debug_information->min_align_radius = cfg.min_align_radius;
                debug_information->max_align_radius = cfg.max_align_radius;
                debug_information->min_angle_error  = cfg.min_angle_error;
                debug_information->max_angle_error  = cfg.max_angle_error;
                debug_information->angle_to_target  = angle_to_target;
                debug_information->angle_to_final_heading = angle_to_final_heading;
                debug_information->translational_error    = translational_error;
                debug_information->velocity_target        = velocity_command;

                // Create vector field for visualization
                for (int i = -5; i < 5; i++) {
                    for (int j = -5; j < 5; j++) {
                        Eigen::Vector2d rDRr                    = Eigen::Vector2d(i / 10.0, j / 10.0);
                        Eigen::Vector2d desired_velocity_target = vector_field(rDRr, angle_to_final_heading, obstacles);
                        debug_information->vector_field.push_back(desired_velocity_target);
                    }
                }

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
}  // namespace module::planning
