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
    using message::planning::AlignWithTarget;
    using message::planning::RotateToTarget;
    using message::planning::StrafeToTarget;
    using message::planning::TurnAroundBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::planning::WalkToTarget;

    using message::skill::Walk;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Walk tuning
            cfg.max_translational_velocity_magnitude = config["max_translational_velocity_magnitude"].as<double>();
            cfg.min_translational_velocity_magnitude = config["min_translational_velocity_magnitude"].as<double>();
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
            cfg.start_rotate_to_target_pos_error_threshold =
                config["rotate_to_target"]["start_pos_error_threshold"].as<double>();
            cfg.start_rotate_to_target_ori_error_threshold =
                config["rotate_to_target"]["start_ori_error_threshold"].as<double>();
            cfg.stop_rotate_to_target_pos_error_threshold =
                config["rotate_to_target"]["stop_pos_error_threshold"].as<double>();
            cfg.stop_rotate_to_target_ori_error_threshold =
                config["rotate_to_target"]["stop_ori_error_threshold"].as<double>();
            cfg.start_walk_to_target_pos_error_threshold =
                config["walk_to_target"]["start_pos_error_threshold"].as<double>();
            cfg.start_walk_to_target_ori_error_threshold =
                config["walk_to_target"]["start_ori_error_threshold"].as<double>();
            cfg.stop_walk_to_target_pos_error_threshold =
                config["walk_to_target"]["stop_pos_error_threshold"].as<double>();
            cfg.stop_walk_to_target_ori_error_threshold =
                config["walk_to_target"]["stop_ori_error_threshold"].as<double>();
            cfg.start_strafe_to_target_pos_error_threshold =
                config["strafe_to_target"]["start_pos_error_threshold"].as<double>();
            cfg.start_strafe_to_target_ori_error_threshold =
                config["strafe_to_target"]["start_ori_error_threshold"].as<double>();
            cfg.stop_strafe_to_target_pos_error_threshold =
                config["strafe_to_target"]["stop_pos_error_threshold"].as<double>();
            cfg.stop_strafe_to_target_ori_error_threshold =
                config["strafe_to_target"]["stop_ori_error_threshold"].as<double>();
            cfg.start_align_with_target_pos_error_threshold =
                config["align_with_target"]["start_pos_error_threshold"].as<double>();
            cfg.start_align_with_target_ori_error_threshold =
                config["align_with_target"]["start_ori_error_threshold"].as<double>();
            cfg.stop_align_with_target_pos_error_threshold =
                config["align_with_target"]["stop_pos_error_threshold"].as<double>();
            cfg.stop_align_with_target_ori_error_threshold =
                config["align_with_target"]["stop_ori_error_threshold"].as<double>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>, Uses<Walk>>().then([this](const WalkTo& walk_to, const Uses<Walk>& walk) {
            // Emit the walk task as non-director task for debugging
            emit(std::make_unique<WalkTo>(walk_to));
            // If we haven't got an active walk task, then reset the velocity to minimum velocity
            if (walk.run_state == GroupInfo::RunState::NO_TASK) {
                velocity_magnitude = cfg.min_translational_velocity_magnitude;
            }

            // Translational error (distance) from robot to target
            translational_error = walk_to.Hrd.translation().norm();
            emit(graph("translational_error", translational_error));

            // Angle between robot and target point
            angle = std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x());
            emit(graph("angle", angle));

            // Angle error (absolute) between robot and target point
            angle_error = std::abs(std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x()));
            emit(graph("angle_error", angle_error));

            // Angle between robot and target heading
            heading = std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());
            emit(graph("heading", heading));

            // Angle error (absolute) between robot and target heading
            heading_error = std::abs(std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x()));
            emit(graph("heading_error", heading_error));


            // Second argument is priority - higher number means higher priority
            // 1. If far away, and not facing the target, then rotate on spot to face the target
            emit<Task>(std::make_unique<RotateToTarget>(walk_to.Hrd), 4);

            // 2. If far away, and facing the target, then walk towards the target directly
            emit<Task>(std::make_unique<WalkToTarget>(walk_to.Hrd), 3);

            // 3. If close to the target, then strafe to target but do not rotate towards the target
            emit<Task>(std::make_unique<StrafeToTarget>(walk_to.Hrd), 2);

            // 4. If close to the target, but not aligned with the target, then rotate on spot to face the target
            emit<Task>(std::make_unique<AlignWithTarget>(walk_to.Hrd), 1);
        });

        on<Start<RotateToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Starting RotateToTarget task");
            rotate_to_target_pos_error_threshold = cfg.start_rotate_to_target_pos_error_threshold;
            rotate_to_target_ori_error_threshold = cfg.start_rotate_to_target_ori_error_threshold;
        });
        on<Provide<RotateToTarget>>().then([this](const RotateToTarget& rotate_to_face_target) {
            // 1. If far away, and not facing the target, then rotate on spot to face the target
            if (translational_error > rotate_to_target_pos_error_threshold
                && angle_error > rotate_to_target_ori_error_threshold) {
                int sign = angle < 0 ? -1 : 1;
                // Turn on the spot
                emit<Task>(std::make_unique<Walk>(
                    Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
                emit(graph("Rotating to face target", true));
            }
            else {
                emit(graph("Rotating to face target", false));
            }
            emit(graph("rotate_to_target_pos_error_threshold", rotate_to_target_pos_error_threshold));
            emit(graph("rotate_to_target_ori_error_threshold", rotate_to_target_ori_error_threshold));
        });
        on<Stop<RotateToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Stopping RotateToTarget task");
            rotate_to_target_pos_error_threshold = cfg.stop_rotate_to_target_pos_error_threshold;
            rotate_to_target_ori_error_threshold = cfg.stop_rotate_to_target_ori_error_threshold;
        });

        on<Start<WalkToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Starting WalkToTarget task");
            walk_to_target_pos_error_threshold = cfg.start_walk_to_target_pos_error_threshold;
            walk_to_target_ori_error_threshold = cfg.start_walk_to_target_ori_error_threshold;
        });
        on<Provide<WalkToTarget>>().then([this](const WalkToTarget& walk_direct_to_target) {
            // 2. If far away, and facing the target, then walk towards the target directly
            // Accelerate to max velocity
            if (translational_error > walk_to_target_pos_error_threshold
                && angle_error > walk_to_target_ori_error_threshold) {
                velocity_magnitude += cfg.acceleration;
                velocity_magnitude = std::max(cfg.min_translational_velocity_magnitude,
                                              std::min(velocity_magnitude, cfg.max_translational_velocity_magnitude));
                // Obtain the unit vector to desired target in robot space and scale by magnitude
                Eigen::Vector3d velocity_target =
                    walk_direct_to_target.Hrd.translation().normalized() * velocity_magnitude;

                // Set the angular velocity as the angle to the target and clamp to min and max angular velocity
                velocity_target.z() = utility::math::clamp(cfg.min_angular_velocity, angle, cfg.max_angular_velocity);

                emit<Task>(std::make_unique<Walk>(velocity_target));
                emit(graph("Walking directly towards target", true));
            }
            else {
                emit(graph("Walking directly towards target", false));
            }
            emit(graph("walk_to_target_pos_error_threshold", walk_to_target_pos_error_threshold));
            emit(graph("walk_to_target_ori_error_threshold", walk_to_target_ori_error_threshold));
        });
        on<Stop<WalkToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Stopping WalkToTarget task");
            walk_to_target_pos_error_threshold = cfg.stop_walk_to_target_pos_error_threshold;
            walk_to_target_ori_error_threshold = cfg.stop_walk_to_target_ori_error_threshold;
        });

        on<Start<StrafeToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Starting StrafeToTarget task");
            strafe_to_target_pos_error_threshold = cfg.start_strafe_to_target_pos_error_threshold;
            strafe_to_target_ori_error_threshold = cfg.start_strafe_to_target_ori_error_threshold;
        });
        on<Provide<StrafeToTarget>>().then([this](const StrafeToTarget& strafe_to_target) {
            // 3. If close to the target, then strafe to target but do not rotate towards the target
            if (translational_error > strafe_to_target_pos_error_threshold
                && angle_error > strafe_to_target_ori_error_threshold) {
                // Decelerate to min velocity
                velocity_magnitude -= cfg.acceleration;
                velocity_magnitude = std::max(velocity_magnitude, cfg.min_translational_velocity_magnitude);

                // Obtain the unit vector to desired target in robot space and scale by magnitude
                Eigen::Vector3d velocity_target = strafe_to_target.Hrd.translation().normalized() * velocity_magnitude;

                // Set the angular velocity to zero so we only strafe
                velocity_target.z() = 0;

                emit<Task>(std::make_unique<Walk>(velocity_target));
                emit(graph("Strafing towards goal", true));
            }
            else {
                emit(graph("Strafing towards goal", false));
            }
            emit(graph("strafe_to_target_pos_error_threshold", strafe_to_target_pos_error_threshold));
            emit(graph("strafe_to_target_ori_error_threshold", strafe_to_target_ori_error_threshold));
        });
        on<Stop<StrafeToTarget>>().then([this]() {
            log<NUClear::DEBUG>("Stopping StrafeToTarget task");
            strafe_to_target_pos_error_threshold = cfg.stop_strafe_to_target_pos_error_threshold;
            strafe_to_target_ori_error_threshold = cfg.stop_strafe_to_target_ori_error_threshold;
        });

        on<Start<AlignWithTarget>>().then([this]() {
            log<NUClear::DEBUG>("Starting AlignWithTarget task");
            align_with_target_pos_error_threshold = cfg.start_align_with_target_pos_error_threshold;
            align_with_target_ori_error_threshold = cfg.start_align_with_target_ori_error_threshold;
        });
        on<Provide<AlignWithTarget>>().then([this](const AlignWithTarget& align_with_target) {
            // 4. If close to the target, but not aligned with the target, then align with target heading
            // Determine the direction of rotation
            if (translational_error > align_with_target_pos_error_threshold
                && heading_error > align_with_target_ori_error_threshold) {
                int sign = heading < 0 ? -1 : 1;

                // Turn on the spot
                emit<Task>(std::make_unique<Walk>(
                    Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
                emit(graph("Aligning with target", true));
            }
            else {
                emit(graph("Aligning with target", false));
            }
            emit(graph("align_with_target_pos_error_threshold", align_with_target_pos_error_threshold));
            emit(graph("align_with_target_ori_error_threshold", align_with_target_ori_error_threshold));
        });
        on<Stop<AlignWithTarget>>().then([this]() {
            log<NUClear::DEBUG>("Stopping AlignWithTarget task");
            align_with_target_pos_error_threshold = cfg.stop_align_with_target_pos_error_threshold;
            align_with_target_ori_error_threshold = cfg.stop_align_with_target_ori_error_threshold;
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
