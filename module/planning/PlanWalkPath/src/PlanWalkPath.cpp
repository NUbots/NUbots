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
            cfg.min_translational_velocity_magnitude = config["min_translational_velocity_magnitude"].as<double>();
            cfg.max_translational_velocity_magnitude = config["max_translational_velocity_magnitude"].as<double>();
            cfg.acceleration                         = config["acceleration"].as<double>();
            cfg.max_angular_velocity                 = config["max_angular_velocity"].as<double>();
            cfg.min_angular_velocity                 = config["min_angular_velocity"].as<double>();
            cfg.rotate_velocity                      = config["rotate_velocity"].as<double>();
            cfg.rotate_velocity_x                    = config["rotate_velocity_x"].as<double>();
            cfg.rotate_velocity_y                    = config["rotate_velocity_y"].as<double>();
            cfg.pivot_ball_velocity                  = config["pivot_ball_velocity"].as<double>();
            cfg.pivot_ball_velocity_x                = config["pivot_ball_velocity_x"].as<double>();
            cfg.pivot_ball_velocity_y                = config["pivot_ball_velocity_y"].as<double>();

            // Thresholds for different walk to tasks
            cfg.rotate_to_thresholds.enter_pos  = config["rotate_to_target"]["enter_pos_threshold"].as<double>();
            cfg.rotate_to_thresholds.enter_ori  = config["rotate_to_target"]["enter_ori_threshold"].as<double>();
            cfg.rotate_to_thresholds.leave_pos  = config["rotate_to_target"]["leave_pos_threshold"].as<double>();
            cfg.rotate_to_thresholds.leave_ori  = config["rotate_to_target"]["leave_ori_threshold"].as<double>();
            cfg.walk_to_thresholds.enter_pos    = config["walk_to_target"]["enter_pos_threshold"].as<double>();
            cfg.walk_to_thresholds.leave_pos    = config["walk_to_target"]["leave_pos_threshold"].as<double>();
            cfg.align_with_thresholds.enter_ori = config["align_with_target"]["enter_ori_threshold"].as<double>();
            cfg.align_with_thresholds.leave_ori = config["align_with_target"]["leave_ori_threshold"].as<double>();

            cfg.approach_radius = config["approach_radius"].as<double>();
        });

        on<Start<WalkTo>>().then([this] {
            log<NUClear::DEBUG>("Starting walk to task");
            // Reset the thresholds to the enter thresholds
            cfg.rotate_to_thresholds.pos  = cfg.rotate_to_thresholds.enter_pos;
            cfg.rotate_to_thresholds.ori  = cfg.rotate_to_thresholds.enter_ori;
            cfg.walk_to_thresholds.pos    = cfg.walk_to_thresholds.enter_pos;
            cfg.align_with_thresholds.ori = cfg.align_with_thresholds.enter_ori;
            velocity_magnitude            = cfg.min_translational_velocity_magnitude;
        });

        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {
            // Calculate the translational error (distance from robot to target) and the target angles
            double translational_error = walk_to.Hrd.translation().norm();
            double angle_to_target     = std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x());
            double angle_to_desired_heading =
                std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());

            // Interpolate between current heading and the desired heading as the robot approaches the target
            double translation_progress = std::max(0.0, 1.0 - (translational_error / cfg.approach_radius));
            double interpolated_angle =
                (1 - translation_progress) * angle_to_target + translation_progress * angle_to_desired_heading;

            // Limit speed based on the angle to the target (only walk forward if aligned with target angle)
            double angle_progress = std::clamp(1 - std::abs(angle_to_target) / M_PI_2, 0.0, 1.0);
            emit(graph("angle_progress", angle_progress));
            // Accelerate
            if (translational_error > cfg.approach_radius) {

                velocity_magnitude += cfg.acceleration;
            }
            else {
                // Decelerate
                velocity_magnitude -= cfg.acceleration;
            }
            // Adjust velocities based on proximity to the target
            double scaled_velocity_magnitude = velocity_magnitude * angle_progress;
            scaled_velocity_magnitude        = std::clamp(scaled_velocity_magnitude,
                                                   cfg.min_translational_velocity_magnitude,
                                                   cfg.max_translational_velocity_magnitude);

            emit(graph("velocity_magnitude", scaled_velocity_magnitude));

            Eigen::Vector3d velocity_target = walk_to.Hrd.translation().normalized() * scaled_velocity_magnitude;
            velocity_target.z() = std::clamp(interpolated_angle, cfg.min_angular_velocity, cfg.max_angular_velocity);
            emit(graph("interpolated_angle", interpolated_angle));

            // Emit the walk task with the calculated velocities
            emit<Task>(std::make_unique<Walk>(velocity_target));

            // Emit debugging information for visualization and monitoring
            auto debug_information = std::make_unique<WalkToDebug>();
            Eigen::Isometry3d Hrd  = Eigen::Isometry3d::Identity();
            Hrd.translation()      = walk_to.Hrd.translation();
            Hrd.linear()           = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, interpolated_angle));
            debug_information->Hrd = Hrd;
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
            if (walk_direct.Hrd.translation().norm() > walk_direct.approach_radius) {
                velocity_magnitude += cfg.acceleration;
            }
            else {
                velocity_magnitude -= cfg.acceleration;
            }
            velocity_magnitude = std::clamp(velocity_magnitude,
                                            cfg.min_translational_velocity_magnitude,
                                            cfg.max_translational_velocity_magnitude);
            // Obtain the unit vector to desired target in robot space and scale by magnitude
            Eigen::Vector3d velocity_target = walk_direct.Hrd.translation().normalized() * velocity_magnitude;

            // Set the angular velocity as the angle_to_target to the target and clamp to min and max angular velocity
            if (!walk_direct.dont_align_towards_target) {
                velocity_target.z() =
                    utility::math::clamp(cfg.min_angular_velocity, angle_to_target, cfg.max_angular_velocity);
            }
            else {
                velocity_target.z() = 0;
            }

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
