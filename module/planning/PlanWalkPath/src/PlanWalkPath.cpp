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

#include "message/actuation/LimbsIK.hpp"
#include "message/localisation/Ball.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::localisation::Ball;
    using message::planning::PivotAroundPoint;
    using message::planning::TurnOnSpot;
    using message::planning::WalkDirect;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;

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
            cfg.rotate_to_thresholds.enter_pos  = config["rotate_to_target"]["enter_pos_threshold"].as<double>();
            cfg.rotate_to_thresholds.enter_ori  = config["rotate_to_target"]["enter_ori_threshold"].as<double>();
            cfg.rotate_to_thresholds.leave_pos  = config["rotate_to_target"]["leave_pos_threshold"].as<double>();
            cfg.rotate_to_thresholds.leave_ori  = config["rotate_to_target"]["leave_ori_threshold"].as<double>();
            cfg.walk_to_thresholds.enter_pos    = config["walk_to_target"]["enter_pos_threshold"].as<double>();
            cfg.walk_to_thresholds.enter_ori    = config["walk_to_target"]["enter_ori_threshold"].as<double>();
            cfg.walk_to_thresholds.leave_pos    = config["walk_to_target"]["leave_pos_threshold"].as<double>();
            cfg.walk_to_thresholds.leave_ori    = config["walk_to_target"]["leave_ori_threshold"].as<double>();
            cfg.strafe_to_thresholds.enter_pos  = config["strafe_to_target"]["enter_pos_threshold"].as<double>();
            cfg.strafe_to_thresholds.enter_ori  = config["strafe_to_target"]["enter_ori_threshold"].as<double>();
            cfg.strafe_to_thresholds.leave_pos  = config["strafe_to_target"]["leave_pos_threshold"].as<double>();
            cfg.strafe_to_thresholds.leave_ori  = config["strafe_to_target"]["leave_ori_threshold"].as<double>();
            cfg.align_with_thresholds.enter_pos = config["align_with_target"]["enter_pos_threshold"].as<double>();
            cfg.align_with_thresholds.enter_ori = config["align_with_target"]["enter_ori_threshold"].as<double>();
            cfg.align_with_thresholds.leave_pos = config["align_with_target"]["leave_pos_threshold"].as<double>();
            cfg.align_with_thresholds.leave_ori = config["align_with_target"]["leave_ori_threshold"].as<double>();

            // Initialise the thresholds to the enter thresholds
            cfg.rotate_to_thresholds.pos  = cfg.rotate_to_thresholds.enter_pos;
            cfg.rotate_to_thresholds.ori  = cfg.rotate_to_thresholds.enter_ori;
            cfg.walk_to_thresholds.pos    = cfg.walk_to_thresholds.enter_pos;
            cfg.walk_to_thresholds.ori    = cfg.walk_to_thresholds.enter_ori;
            cfg.strafe_to_thresholds.pos  = cfg.strafe_to_thresholds.enter_pos;
            cfg.strafe_to_thresholds.ori  = cfg.strafe_to_thresholds.enter_ori;
            cfg.align_with_thresholds.pos = cfg.align_with_thresholds.enter_pos;
            cfg.align_with_thresholds.ori = cfg.align_with_thresholds.enter_ori;

            cfg.approach_radius = config["approach_radius"].as<double>();
        });

        // Path to walk to a particular point
        on<Provide<WalkTo>, Uses<Walk>>().then([this](const WalkTo& walk_to, const Uses<Walk>& walk) {
            // If we haven't got an active walk task, then reset the velocity to minimum velocity
            if (walk.run_state == GroupInfo::RunState::NO_TASK) {
                velocity_magnitude = cfg.min_translational_velocity_magnitude;
            }

            // Translational error (distance) from robot to target
            translational_error = walk_to.Hrd.translation().norm();
            emit(graph("translational_error", translational_error));

            // Angle between robot and target point
            angle_to_target = std::atan2(walk_to.Hrd.translation().y(), walk_to.Hrd.translation().x());
            emit(graph("angle_to_target", angle_to_target));

            // Angle between robot and target angle_to_desired_heading
            angle_to_desired_heading = std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());
            emit(graph("angle_to_desired_heading", angle_to_desired_heading));


            // 1. If far away, and not facing the target, then rotate on spot to face the target
            if (translational_error > cfg.rotate_to_thresholds.pos
                && std::abs(angle_to_target) > cfg.rotate_to_thresholds.ori) {
                int sign = angle_to_target < 0 ? -1 : 1;
                // emit<Task>(std::make_unique<TurnOnSpot>(clockwise), 4);
                // Turn on the spot
                emit<Task>(std::make_unique<Walk>(
                    Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));

                cfg.rotate_to_thresholds.pos = cfg.rotate_to_thresholds.enter_pos;
                cfg.rotate_to_thresholds.ori = cfg.rotate_to_thresholds.enter_ori;
                emit(graph("Rotating towards target", true));
                return;
            }
            else {
                // Increase thresholds to prevent oscillation when condition is just met
                cfg.rotate_to_thresholds.pos = cfg.rotate_to_thresholds.leave_pos;
                cfg.rotate_to_thresholds.ori = cfg.rotate_to_thresholds.leave_ori;
                emit(graph("Rotating towards target", false));
            }

            // 2. If far away, and facing the target, then walk towards the target directly
            if (translational_error > cfg.walk_to_thresholds.enter_pos
                && std::abs(angle_to_target) > cfg.walk_to_thresholds.ori) {
                emit<Task>(std::make_unique<WalkDirect>(walk_to.Hrd, cfg.approach_radius));
                cfg.walk_to_thresholds.pos = cfg.walk_to_thresholds.enter_pos;
                cfg.walk_to_thresholds.ori = cfg.walk_to_thresholds.enter_ori;
                emit(graph("Walking towards target", true));
                return;
            }
            else {
                cfg.walk_to_thresholds.pos = cfg.walk_to_thresholds.leave_pos;
                cfg.walk_to_thresholds.ori = cfg.walk_to_thresholds.leave_ori;
                emit(graph("Walking towards target", false));
            }

            //  3. If close to the target, then walk to target but do not rotate towards the target if it goes behind
            if (translational_error > cfg.strafe_to_thresholds.pos
                && std::abs(angle_to_target) > cfg.strafe_to_thresholds.ori) {
                if (walk_to.Hrd.translation().x() < 0) {
                    emit<Task>(std::make_unique<WalkDirect>(walk_to.Hrd, cfg.approach_radius, true));
                }
                else {
                    emit<Task>(std::make_unique<WalkDirect>(walk_to.Hrd, cfg.approach_radius));
                }
                cfg.strafe_to_thresholds.pos = cfg.strafe_to_thresholds.enter_pos;
                cfg.strafe_to_thresholds.ori = cfg.strafe_to_thresholds.enter_ori;
                emit(graph("Strafing towards target", true));
                return;
            }
            else {
                emit(graph("Strafing towards target", false));
                cfg.strafe_to_thresholds.pos = cfg.strafe_to_thresholds.leave_pos;
                cfg.strafe_to_thresholds.ori = cfg.strafe_to_thresholds.leave_ori;
            }

            // 4. If close to the target, but not aligned with the target, then align with target heading
            if (translational_error > cfg.align_with_thresholds.pos
                && std::abs(angle_to_desired_heading) > cfg.align_with_thresholds.ori) {
                // Determine the direction of rotation
                bool clockwise = angle_to_desired_heading < 0 ? true : false;
                emit<Task>(std::make_unique<TurnOnSpot>(clockwise));
                cfg.align_with_thresholds.pos = cfg.align_with_thresholds.enter_pos;
                cfg.align_with_thresholds.ori = cfg.align_with_thresholds.enter_ori;
                emit(graph("Aligning with target heading", true));
            }
            else {
                emit(graph("Aligning with target heading", false));
                cfg.align_with_thresholds.pos = cfg.align_with_thresholds.leave_pos;
                cfg.align_with_thresholds.ori = cfg.align_with_thresholds.leave_ori;
            }

            emit_debug_information(walk_to.Hrd, Eigen::Vector3d(0, 0, 0));
        });

        on<Provide<TurnOnSpot>>().then([this](const TurnOnSpot& turn_on_spot) {
            // Determine the direction of rotation
            int sign = turn_on_spot.clockwise ? -1 : 1;

            // Turn on the spot
            emit<Task>(std::make_unique<Walk>(
                Eigen::Vector3d(cfg.rotate_velocity_x, cfg.rotate_velocity_y, sign * cfg.rotate_velocity)));
        });

        on<Provide<WalkDirect>>().then([this](const WalkDirect& walk_direct) {
            if (walk_direct.Hrd.translation().norm() > walk_direct.approach_radius) {
                velocity_magnitude += cfg.acceleration;
            }
            else {
                velocity_magnitude -= cfg.acceleration;
            }
            velocity_magnitude = std::max(cfg.min_translational_velocity_magnitude,
                                          std::min(velocity_magnitude, cfg.max_translational_velocity_magnitude));
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

    void PlanWalkPath::emit_debug_information(const Eigen::Isometry3d& Hrd, const Eigen::Vector3d& velocity_target) {
        auto walk_to_debug = std::make_unique<WalkToDebug>();
        walk_to_debug->Hrd = Hrd;
        // walk_to_debug->velocity_target                 = velocity_target;
        // walk_to_debug->cfg.rotate_to_thresholds.pos  = cfg.rotate_to_thresholds.pos;
        // walk_to_debug->cfg.rotate_to_thresholds.ori  = cfg.rotate_to_thresholds.ori;
        // walk_to_debug->cfg.walk_to_thresholds.enter_pos    = cfg.walk_to_thresholds.enter_pos;
        // walk_to_debug->cfg.walk_to_thresholds.ori    = cfg.walk_to_thresholds.ori;
        // walk_to_debug->cfg.strafe_to_thresholds.pos  = cfg.strafe_to_thresholds.pos;
        // walk_to_debug->cfg.strafe_to_thresholds.ori  = cfg.strafe_to_thresholds.ori;
        // walk_to_debug->cfg.align_with_thresholds.pos = cfg.align_with_thresholds.pos;
        // walk_to_debug->cfg.align_with_thresholds.ori = cfg.align_with_thresholds.ori;
        // walk_to_debug->translational_error             = translational_error;
        // walk_to_debug->angle_to_target                 = angle_to_target;
        // walk_to_debug->angle_to_desired_headingr       = angle_to_desired_heading;
        emit(walk_to_debug);
    }
}  // namespace module::planning
