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
#include "Walk.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::LeftArm;
    using message::actuation::LeftLegIK;
    using message::actuation::RightArm;
    using message::actuation::RightLegIK;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;
    using utility::input::FrameID;
    using utility::input::ServoID;
    using WalkTask  = message::skill::Walk;
    using WalkState = message::behaviour::state::WalkState;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    Walk::Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Walk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Configure the motion generation options
            cfg.walk_generator_parameters.step_period     = config["walk"]["period"].as<double>();
            cfg.walk_generator_parameters.step_apex_ratio = config["walk"]["step"]["apex_ratio"].as<double>();
            cfg.walk_generator_parameters.step_limits     = config["walk"]["step"]["limits"].as<Expression>();
            cfg.walk_generator_parameters.step_height     = config["walk"]["step"]["height"].as<double>();
            cfg.walk_generator_parameters.step_width      = config["walk"]["step"]["width"].as<double>();
            cfg.walk_generator_parameters.torso_height    = config["walk"]["torso"]["height"].as<double>();
            cfg.walk_generator_parameters.torso_pitch     = config["walk"]["torso"]["pitch"].as<Expression>();
            cfg.walk_generator_parameters.torso_position_offset =
                config["walk"]["torso"]["position_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_offset = config["walk"]["torso"]["sway_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_ratio  = config["walk"]["torso"]["sway_ratio"].as<double>();
            cfg.walk_generator_parameters.torso_final_position_ratio =
                config["walk"]["torso"]["final_position_ratio"].as<Expression>();
            walk_generator.set_parameters(cfg.walk_generator_parameters);

            // Reset the walk engine and last update time
            walk_generator.reset();
            last_update_time = NUClear::clock::now();

            // Configure the arms
            for (auto id : utility::input::LimbID::servos_for_arms()) {
                cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<double>(), 100);
            }
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH,
                                           config["arms"]["right_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH,
                                           config["arms"]["left_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL,
                                           config["arms"]["right_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<double>());
        });

        // Start - Runs every time the Walk provider starts (wasn't running)
        on<Start<WalkTask>>().then([this]() {
            // Reset the last update time and walk engine
            last_update_time = NUClear::clock::now();
            walk_generator.reset();
            // Emit a stopped state as we are not yet walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Stop - Runs every time the Walk task is removed from the director tree
        on<Stop<WalkTask>>().then([this] {
            // Emit a stopped state as we are now not walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Main loop - Updates the walk engine at fixed frequency of UPDATE_FREQUENCY
        on<Provide<WalkTask>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           With<Sensors>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this](const WalkTask& walk_task, const Sensors& sensors) {
                // Compute time since the last update
                auto time_delta =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                Eigen::Vector3d step = Eigen::Vector3d::Zero();
                step.x() = std::max(std::min(walk_task.velocity_target.x() * cfg.walk_generator_parameters.step_period,
                                             cfg.walk_generator_parameters.step_limits.x()),
                                    -cfg.walk_generator_parameters.step_limits.x());
                step.y() = std::max(std::min(walk_task.velocity_target.y() * cfg.walk_generator_parameters.step_period,
                                             cfg.walk_generator_parameters.step_limits.y()),
                                    -cfg.walk_generator_parameters.step_limits.y())
                           + walk_generator.get_foot_width_offset();
                step.z() = std::max(std::min(walk_task.velocity_target.z() * cfg.walk_generator_parameters.step_period,
                                             cfg.walk_generator_parameters.step_limits.z()),
                                    -cfg.walk_generator_parameters.step_limits.z());

                double t = walk_generator.get_time();
                double T = cfg.walk_generator_parameters.step_period - t;

                // Get current desired torso pose
                Eigen::Isometry3d Hpt_t = walk_generator.get_torso_pose();

                // Get the desired torso pose at the end of the step
                Eigen::Isometry3d Hpt_T_desired =
                    walk_generator.get_torso_pose(cfg.walk_generator_parameters.step_period);

                // Get the current torso pose from the sensors
                Eigen::Isometry3d Hpt_t_sensors = walk_generator.is_left_foot_planted()
                                                      ? Eigen::Isometry3d(sensors.Htx[FrameID::L_FOOT_BASE].inverse())
                                                      : Eigen::Isometry3d(sensors.Htx[FrameID::R_FOOT_BASE].inverse());

                // Get the current torso velocity from the sensors
                Eigen::Vector3d vTp_t_sensors = sensors.Hwp.inverse().linear() * sensors.vTw;


                // Get the estimated torso pose at the end of the step using LIPM from sensors initial state
                double w                       = std::sqrt(9.81 / cfg.walk_generator_parameters.torso_height);
                Eigen::Vector3d rTPp_t_sensors = Hpt_t_sensors.translation();
                Eigen::Vector3d rPTt_T_est = rTPp_t_sensors * std::cosh(w * T) + vTp_t_sensors / w * std::sinh(w * T);

                // Get the estimated torso velocity at the end of the step using LIPM from sensors initial state
                Eigen::Vector3d vTp_T_est = rTPp_t_sensors * w * std::sinh(w * T) + vTp_t_sensors * std::cosh(w * T);


                Eigen::Vector3d rTPp_t         = Hpt_t.translation();
                Eigen::Vector3d rTPp_T_desired = Hpt_T_desired.translation();


                // Compute capture point at t and T
                Eigen::Vector3d capture_point_t =
                    vTp_t_sensors * std::sqrt(cfg.walk_generator_parameters.torso_height / 9.81);
                Eigen::Vector3d capture_point_T =
                    vTp_T_est * std::sqrt(cfg.walk_generator_parameters.torso_height / 9.81);
                emit(graph("t: ", t));
                emit(graph("T: ", T));
                emit(graph("Step: ", step.x(), step.y(), step.z()));
                emit(graph("rTPp_t_sensors: ", rTPp_t_sensors.x(), rTPp_t_sensors.y(), rTPp_t_sensors.z()));
                emit(graph("rTPp_t: ", rTPp_t.x(), rTPp_t.y(), rTPp_t.z()));
                emit(graph("rPTt_T_est: ", rPTt_T_est.x(), rPTt_T_est.y(), rPTt_T_est.z()));
                emit(graph("rTPp_T_desired: ", rTPp_T_desired.x(), rTPp_T_desired.y(), rTPp_T_desired.z()));
                emit(graph("vTp_t_sensors: ", vTp_t_sensors.x(), vTp_t_sensors.y(), vTp_t_sensors.z()));
                emit(graph("vTp_T_est: ", vTp_T_est.x(), vTp_T_est.y(), vTp_T_est.z()));
                emit(graph("Capture_point_t: ", capture_point_t.x(), capture_point_t.y(), 0));
                emit(graph("Capture point_T: ", capture_point_T.x(), capture_point_T.y(), 0));

                if (t > cfg.walk_generator_parameters.step_period * 0.8 && walk_task.velocity_target.norm() > 0.0) {
                    // Move the step to the capture point when the step is halfway through
                    // step = step + 0.05 * (capture_point_T - step);
                    // walk_generator.set_step(capture_point_T);
                    step.x() = capture_point_T.x();
                    walk_generator.set_step(step);
                }
                else {
                    walk_generator.set_step(step);
                }


                // Update the walk engine and emit the stability state
                switch (walk_generator.update(time_delta, walk_task.velocity_target, vTp_t_sensors).value) {
                    case WalkState::State::WALKING:
                    case WalkState::State::STOPPING: emit(std::make_unique<Stability>(Stability::DYNAMIC)); break;
                    case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                    case WalkState::State::UNKNOWN:
                    default: NUClear::log<NUClear::WARN>("Unknown state."); break;
                }


                // Compute the goal position time
                const NUClear::clock::time_point goal_time =
                    NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

                // Get desired feet poses in the torso {t} frame from the walk engine
                Eigen::Isometry3d Htl = walk_generator.get_foot_pose(LimbID::LEFT_LEG);
                Eigen::Isometry3d Htr = walk_generator.get_foot_pose(LimbID::RIGHT_LEG);

                // Construct ControlFoot tasks
                emit<Task>(std::make_unique<ControlLeftFoot>(Htl, goal_time));
                emit<Task>(std::make_unique<ControlRightFoot>(Htr, goal_time));

                // Construct Arm IK tasks
                auto left_arm  = std::make_unique<LeftArm>();
                auto right_arm = std::make_unique<RightArm>();
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_ARM)) {
                    right_arm->servos[id] =
                        ServoCommand(goal_time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
                }
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_ARM)) {
                    left_arm->servos[id] =
                        ServoCommand(goal_time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
                }
                emit<Task>(left_arm, 0, true, "Walk left arm");
                emit<Task>(right_arm, 0, true, "Walk right arm");

                // Emit the walk state
                WalkState::Phase phase =
                    walk_generator.is_left_foot_planted() ? WalkState::Phase::LEFT : WalkState::Phase::RIGHT;
                auto walk_state =
                    std::make_unique<WalkState>(walk_generator.get_state(), walk_task.velocity_target, phase);


                // Debugging
                if (log_level <= NUClear::DEBUG) {
                    Eigen::Vector3d thetaTL = mat_to_rpy_intrinsic(Htl.linear());
                    emit(graph("Left foot desired position (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
                    emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));
                    Eigen::Vector3d thetaTR = mat_to_rpy_intrinsic(Htr.linear());
                    emit(graph("Right foot desired position (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
                    emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));
                    Eigen::Isometry3d Hpt   = walk_generator.get_torso_pose();
                    Eigen::Vector3d thetaPT = mat_to_rpy_intrinsic(Hpt.linear());
                    emit(graph("Torso desired position (x,y,z)",
                               Hpt.translation().x(),
                               Hpt.translation().y(),
                               Hpt.translation().z()));
                    emit(graph("Torso desired orientation (r,p,y)", thetaPT.x(), thetaPT.y(), thetaPT.z()));
                }
                emit(walk_state);
            });
    }

}  // namespace module::skill
