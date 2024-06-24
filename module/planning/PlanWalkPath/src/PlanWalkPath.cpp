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
    using message::planning::WalkDirect;
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

            // Walk tuning
            cfg.max_translational_velocity_x = config["max_translational_velocity_x"].as<double>();
            cfg.max_translational_velocity_y = config["max_translational_velocity_y"].as<double>();
            cfg.max_angular_velocity         = config["max_angular_velocity"].as<double>();

            cfg.acceleration = config["acceleration"].as<double>();

            cfg.rotate_velocity   = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y = config["rotate_velocity_y"].as<double>();

            cfg.pivot_ball_velocity   = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y = config["pivot_ball_velocity_y"].as<double>();

            cfg.align_radius    = config["align_radius"].as<double>();
            cfg.max_angle_error = config["max_angle_error"].as<Expression>();
            cfg.min_angle_error = config["min_angle_error"].as<Expression>();
        });

        on<Start<WalkTo>>().then([this] {
            log<NUClear::DEBUG>("Starting walk to task");
            velocity_magnitude = 0;
        });

        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            Eigen::Isometry3d Hrd           = walk_to.Hrd;
            Eigen::Vector3d rDRr            = walk_to.Hrd.translation();
            double translational_error      = Hrd.translation().norm();
            double angle_to_target          = std::atan2(rDRr.y(), rDRr.x());
            double angle_to_desired_heading = std::atan2(Hrd.linear().col(0).y(), Hrd.linear().col(0).x());

            // When in the align radius, the robot should be decelerating and begin aligning with the target heading
            double translation_progress = std::max(0.0, 1.0 - (translational_error / cfg.align_radius));
            emit(graph("translation_progress", translation_progress));

            // Linearly interpolate between the angle to the target and the angle to the desired heading when close
            double interpolated_angle =
                (1 - translation_progress) * angle_to_target + translation_progress * angle_to_desired_heading;
            emit(graph("interpolated_angle", interpolated_angle));

            // New calculation of angle_progress with smooth transition
            double angle_progress =
                (cfg.max_angle_error - std::abs(interpolated_angle)) / (cfg.max_angle_error - cfg.min_angle_error);
            angle_progress = std::clamp(angle_progress, 0.0, 1.0);
            emit(graph("angle_progress", angle_progress));

            if (translational_error > cfg.align_radius) {
                // Accelerate
                double max_velocity_magnitude =
                    std::max(cfg.max_translational_velocity_x, cfg.max_translational_velocity_y);
                velocity_magnitude += cfg.acceleration;
                velocity_magnitude = std::min(velocity_magnitude, max_velocity_magnitude);
            }
            else {
                // Decelerate
                double min_velocity_magnitude =
                    std::min(cfg.max_translational_velocity_x, cfg.max_translational_velocity_y);
                velocity_magnitude -= cfg.acceleration;
                velocity_magnitude = std::max(velocity_magnitude, min_velocity_magnitude);
            }
            emit(graph("velocity_magnitude", velocity_magnitude));

            // Adjust velocity based on orientation error
            double scaled_velocity_magnitude = velocity_magnitude * angle_progress;

            emit(graph("scaled_velocity_magnitude", scaled_velocity_magnitude));

            Eigen::Vector3d velocity_target = rDRr.normalized() * scaled_velocity_magnitude;
            velocity_target.z()             = interpolated_angle;
            velocity_target                 = constrain_velocity(velocity_target,
                                                 cfg.max_translational_velocity_x,
                                                 cfg.max_translational_velocity_y,
                                                 cfg.max_angular_velocity);
            emit(graph("velocity_target", velocity_target.x(), velocity_target.y(), velocity_target.z()));

            // Emit the walk task with the calculated velocities
            emit<Task>(std::make_unique<Walk>(velocity_target));

            // Emit debugging information for visualization and monitoring
            auto debug_information       = std::make_unique<WalkToDebug>();
            Eigen::Isometry3d Hrd_target = Eigen::Isometry3d::Identity();
            Hrd_target.translation()     = rDRr;
            Hrd_target.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, interpolated_angle));
            debug_information->Hrd       = Hrd_target;
            emit(debug_information);
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Start<WalkDirect>>().then([this] {
            log<NUClear::DEBUG>("Starting walk direct task");
            // Reset the walk direct velocity magnitude to minimum velocity
            // velocity_magnitude = cfg.min_translational_velocity_magnitude;
        });

        on<Provide<WalkDirect>>().then([this](const WalkDirect& walk_direct) {
            if (walk_direct.Hrd.translation().norm() > cfg.align_radius) {
                velocity_magnitude += cfg.acceleration;
            }
            else {
                velocity_magnitude -= cfg.acceleration;
            }
            // Obtain the unit vector to desired target in robot space and scale by magnitude
            Eigen::Vector3d velocity_target = walk_direct.Hrd.translation().normalized() * velocity_magnitude;

            // Set the angular velocity as the angle_to_target to the target and clamp to min and max angular velocity
            if (!walk_direct.dont_align_towards_target) {
                velocity_target.z() = angle_to_target;
            }
            else {
                velocity_target.z() = 0;
            }

            // Constrain the velocity to the maximum translational and angular velocities
            velocity_target = constrain_velocity(velocity_target,
                                                 cfg.max_translational_velocity_x,
                                                 cfg.max_translational_velocity_y,
                                                 cfg.max_angular_velocity);

            emit<Task>(std::make_unique<Walk>(velocity_target));
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
