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

#include "utility/input/FrameID.hpp"
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
    using WalkTask = message::skill::Walk;
    using message::input::Sensors;
    using WalkState = message::behaviour::state::WalkState;

    using utility::input::FrameID;
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
            cfg.desired_torso_pitch = Eigen::Matrix<double, 1, 1>(cfg.walk_generator_parameters.torso_pitch);
            cfg.walk_generator_parameters.torso_position_offset =
                config["walk"]["torso"]["position_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_offset = config["walk"]["torso"]["sway_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_ratio  = config["walk"]["torso"]["sway_ratio"].as<double>();
            cfg.walk_generator_parameters.torso_final_position_ratio =
                config["walk"]["torso"]["final_position_ratio"].as<Expression>();
            walk_generator.set_parameters(cfg.walk_generator_parameters);
            cfg.walk_generator_parameters.only_switch_when_planted =
                config["walk"]["only_switch_when_planted"].as<bool>();

            // Reset the walk engine and last update time
            walk_generator.reset();
            last_update_time = NUClear::clock::now();

            // Controller gains
            cfg.arm_servo_gain   = config["gains"]["arm_servo_gain"].as<double>();
            cfg.leg_servo_gain   = config["gains"]["leg_servo_gain"].as<double>();
            cfg.torso_pid_gains  = config["gains"]["torso_pid_gains"].as<Expression>();
            cfg.torso_antiwindup = config["gains"]["torso_antiwindup"].as<Expression>();
            cfg.pitch_pid_gains  = config["gains"]["pitch_pid_gains"].as<Expression>();
            cfg.pitch_antiwindup = config["gains"]["pitch_antiwindup"].as<Expression>();

            // Configure torso PID controller
            torso_controller       = utility::math::control::PID<double, 2>(cfg.torso_pid_gains[0],
                                                                      cfg.torso_pid_gains[1],
                                                                      cfg.torso_pid_gains[2],
                                                                      cfg.torso_antiwindup[0],
                                                                      cfg.torso_antiwindup[1]);
            cfg.torso_filter_alpha = config["gains"]["torso_filter_alpha"].as<double>();
            // Configure pitch PID controller
            pitch_controller = utility::math::control::PID<double, 1>(cfg.pitch_pid_gains[0],
                                                                      cfg.pitch_pid_gains[1],
                                                                      cfg.pitch_pid_gains[2],
                                                                      cfg.pitch_antiwindup[0],
                                                                      cfg.pitch_antiwindup[1]);

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

            // Since walk needs a Stability message to run, emit one at the beginning
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
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
           With<Sensors>,
           With<Stability>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this](const WalkTask& walk_task, const Sensors& sensors, const Stability& stability) {
                // Compute time since the last update
                auto time_delta =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                // Update torso pitch controller
                auto measured_pitch =
                    Eigen::Matrix<double, 1, 1>(MatrixToEulerIntrinsic(sensors.Htw.inverse().rotation()).y());
                auto pitch_offset =
                    pitch_controller.update(cfg.desired_torso_pitch, measured_pitch, time_delta)(0, 0) * time_delta;
                cfg.walk_generator_parameters.torso_pitch = cfg.walk_generator_parameters.torso_pitch + pitch_offset;
                walk_generator.set_parameters(cfg.walk_generator_parameters);


                // Update the walk engine and emit the stability state, only if not falling/fallen
                if (stability >= Stability::DYNAMIC) {
                    switch (walk_generator.update(time_delta, walk_task.velocity_target, sensors.planted_foot_phase)
                                .value) {
                        case WalkState::State::STARTING:
                        case WalkState::State::WALKING:
                        case WalkState::State::STOPPING: emit(std::make_unique<Stability>(Stability::DYNAMIC)); break;
                        case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                        case WalkState::State::UNKNOWN:
                        default: NUClear::log<NUClear::WARN>("Unknown state."); break;
                    }
                }

                // Compute the goal position time
                const NUClear::clock::time_point goal_time =
                    NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

                // Construct leg IK tasks
                auto left_leg   = std::make_unique<LeftLegIK>();
                auto right_leg  = std::make_unique<RightLegIK>();
                left_leg->time  = goal_time;
                right_leg->time = goal_time;
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_LEG)) {
                    left_leg->servos[id] = ServoState(cfg.leg_servo_gain, 100);
                }
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                    right_leg->servos[id] = ServoState(cfg.leg_servo_gain, 100);
                }

                // Construct ControlFoot tasks
                emit<Task>(std::make_unique<ControlLeftFoot>(Htl, goal_time));
                emit<Task>(std::make_unique<ControlRightFoot>(Htr, goal_time));

                const Eigen::Isometry3d Htl_measured(sensors.Htx[FrameID::L_FOOT_BASE]);
                const Eigen::Isometry3d Htr_measured(sensors.Htx[FrameID::R_FOOT_BASE]);

                // Set desired foot poses in the torso {t} frame and apply torso offsets to keep the robot balanced
                left_leg->Htl  = walk_generator.is_left_foot_planted()
                                     ? compute_torso_offset(Htl_desired, Htl_measured, time_delta)
                                     : Htl_desired;
                right_leg->Htr = !walk_generator.is_left_foot_planted()
                                     ? compute_torso_offset(Htr_desired, Htr_measured, time_delta)
                                     : Htr_desired;

                emit(graph("control", torso_controller.get_control().x(), torso_controller.get_control().y()));
                emit(graph("control_filter", filtered_torso_offset.x(), filtered_torso_offset.y()));

                emit<Task>(left_leg);
                emit<Task>(right_leg);

                // Construct arm IK tasks
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
                auto walk_state = std::make_unique<WalkState>(walk_generator.get_state(),
                                                              walk_task.velocity_target,
                                                              walk_generator.get_phase());

                // Debugging
                if (log_level <= NUClear::DEBUG) {
                    Eigen::Vector3d thetaTL = mat_to_rpy_intrinsic(Htl.linear());
                    emit(graph("Left foot desired position rLTt (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
                    emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));
                    Eigen::Vector3d thetaTR = mat_to_rpy_intrinsic(Htr.linear());
                    emit(graph("Right foot desired position rRTt (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
                    emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));
                    Eigen::Isometry3d Hpt   = walk_generator.get_torso_pose();
                    Eigen::Vector3d thetaPT = mat_to_rpy_intrinsic(Hpt.linear());
                    emit(graph("Torso desired position rTPt (x,y,z)",
                               Hpt.translation().x(),
                               Hpt.translation().y(),
                               Hpt.translation().z()));
                    emit(graph("Torso desired orientation (r,p,y)", thetaPT.x(), thetaPT.y(), thetaPT.z()));
                    emit(graph("Walk state", int(walk_state->state)));
                    emit(graph("Walk velocity target",
                               walk_task.velocity_target.x(),
                               walk_task.velocity_target.y(),
                               walk_task.velocity_target.z()));
                    emit(graph("Walk phase", int(walk_generator.get_phase())));
                    emit(graph("Walk time", walk_generator.get_time()));

                    // Generate a set of swing foot poses for visually debugging
                    if (walk_task.velocity_target.norm() > 0) {
                        double t = 0;
                        while (t < cfg.walk_generator_parameters.step_period) {
                            auto Hps = walk_generator.get_swing_foot_pose(t);
                            auto Hpt = walk_generator.get_torso_pose(t);
                            walk_state->swing_foot_trajectory.push_back(Hps.translation());
                            walk_state->torso_trajectory.push_back(Hpt.translation());
                            t += cfg.walk_generator_parameters.step_period / 10;
                        }
                    }
                }
                emit(walk_state);
            });
    };
    Eigen::Isometry3d Walk::compute_torso_offset(const Eigen::Isometry3d& Htp_desired,
                                                 const Eigen::Isometry3d& Htp_measured,
                                                 const double time_delta) {
        // Proportional Control
        // u = Kp * (rTPp_desired - rTPp_measured)
        Eigen::Vector2d rTPp_desired  = Htp_desired.inverse().translation().head<2>();
        Eigen::Vector2d rTPp_measured = Htp_measured.inverse().translation().head<2>();
        torso_controller.update(rTPp_desired, rTPp_measured, time_delta);

        // TODO: Add velocity error
        // TODO: Add ZMP error

        // Filter controller using exponential filter (leaky integrator)
        filtered_torso_offset = cfg.torso_filter_alpha * (torso_controller.get_control() * time_delta)
                                + (1 - cfg.torso_filter_alpha) * filtered_torso_offset;
        auto Hpt_offset = Htp_desired.inverse();
        Hpt_offset.translation().head<2>() += filtered_torso_offset;
        return Hpt_offset.inverse();
    }

}  // namespace module::skill
