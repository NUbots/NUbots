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

#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;
    using message::strategy::StandStill;

    using utility::math::euler::rpy_intrinsic_to_mat;
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
            max_velocity_magnitude   = std::max(cfg.max_translational_velocity_x, cfg.max_translational_velocity_y);
            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.acceleration         = config["acceleration"].as<double>();

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
        });

        on<Start<WalkTo>>().then([this] {
            log<NUClear::DEBUG>("Starting walk to task");

            // Reset the velocity magnitude to zero
            velocity_magnitude = 0;
        });

        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            // Decompose the target pose into position and orientation
            Eigen::Vector3d rDRr = walk_to.Hrd.translation();
            Eigen::Matrix3d Rrd  = walk_to.Hrd.linear();

            // Calculate the translational error between the robot and the target point (x, y)
            double translational_error = rDRr.norm();

            // Calculate the angle between the robot and the target point (x, y)
            double angle_to_target = std::atan2(rDRr.y(), rDRr.x());

            // Calculate the angle between the robot and the desired heading
            double angle_to_desired_heading = std::atan2(Rrd.col(0).y(), Rrd.col(0).x());

            // Linearly interpolate between angle to the target and desired heading when inside the align radius region
            double translation_progress =
                std::clamp((cfg.max_align_radius - translational_error) / (cfg.max_align_radius - cfg.min_align_radius),
                           0.0,
                           1.0);
            double interpolated_angle =
                (1 - translation_progress) * angle_to_target + translation_progress * angle_to_desired_heading;

            // Scale the velocity with angle error
            // [0 at max_angle_error, linearly interpolate between, 1 at min_angle_error]
            double angle_error_velocity_scaling_factor = std::clamp(
                (cfg.max_angle_error - std::abs(interpolated_angle)) / (cfg.max_angle_error - cfg.min_angle_error),
                0.0,
                1.0);

            double scaled_velocity_magnitude = 0;
            if (translational_error > cfg.max_align_radius) {
                // Accelerate
                velocity_magnitude += cfg.acceleration;
                // Scale the velocity based on the angle error
                scaled_velocity_magnitude = velocity_magnitude * angle_error_velocity_scaling_factor;
                // Clamp the velocity magnitude to the maximum velocity
                velocity_magnitude = std::min(velocity_magnitude, max_velocity_magnitude);
            }
            else {
                // "Proportional control" to strafe towards the target inside align radius
                double error              = translational_error / cfg.max_align_radius;
                scaled_velocity_magnitude = cfg.strafe_gain * error * velocity_magnitude;
            }

            // Calculate the target velocity
            Eigen::Vector2d translational_velocity_target = rDRr.head(2).normalized() * scaled_velocity_magnitude;
            Eigen::Vector3d velocity_target;
            velocity_target << translational_velocity_target, interpolated_angle;

            // Limit the velocity to the maximum translational and angular velocity
            velocity_target = constrain_velocity(velocity_target,
                                                 cfg.max_translational_velocity_x,
                                                 cfg.max_translational_velocity_y,
                                                 cfg.max_angular_velocity);

            // Emit the walk task with the calculated velocities
            emit<Task>(std::make_unique<Walk>(velocity_target));

            // Emit debugging information for visualization and monitoring
            auto debug_information              = std::make_unique<WalkToDebug>();
            Eigen::Isometry3d Hrd               = Eigen::Isometry3d::Identity();
            Hrd.translation()                   = rDRr;
            Hrd.linear()                        = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, interpolated_angle));
            debug_information->Hrd              = Hrd;
            debug_information->min_align_radius = cfg.min_align_radius;
            debug_information->max_align_radius = cfg.max_align_radius;
            debug_information->min_angle_error  = cfg.min_angle_error;
            debug_information->max_angle_error  = cfg.max_angle_error;
            debug_information->angle_to_target  = angle_to_target;
            debug_information->angle_to_desired_heading = angle_to_desired_heading;
            debug_information->translational_error      = translational_error;
            debug_information->velocity_target          = velocity_target;
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
