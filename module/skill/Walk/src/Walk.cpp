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
    using message::skill::SmoothWalk;
    using WalkTask  = message::skill::Walk;
    using WalkState = message::behaviour::state::WalkState;

    using utility::input::FrameID;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    // Same message with a new name!
    struct CurrentWalkTask : WalkTask {
        using WalkTask::WalkTask;
    };

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
            cfg.walk_generator_parameters.only_switch_when_planted =
                config["walk"]["only_switch_when_planted"].as<bool>();

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
            cfg.kick_velocity_x    = config["kick"]["kick_velocity_x"].as<double>();
            cfg.kick_velocity_y    = config["kick"]["kick_velocity_y"].as<double>();
            cfg.kick_timing_offset = config["kick"]["kick_timing_offset"].as<double>();

            // Max acceleration
            cfg.acceleration = config["walk"]["acceleration"].as<Expression>();

            // Since walk needs a Stability message to run, emit one at the beginning
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        // Start - Runs every time the Walk provider starts (wasn't running)
        on<Start<WalkTask>>().then([this]() {
            // Reset the last update time
            last_update_time = NUClear::clock::now();

            // Emit a zero current walk command
            emit(std::make_unique<CurrentWalkTask>());

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
           Needs<ControlLeftFoot>,
           Needs<ControlRightFoot>,
           With<CurrentWalkTask>,
           With<Sensors>,
           With<Stability>,
           Single,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Priority::HIGH>()
            .then([this](const WalkTask& new_walk,
                         const RunReason& run_reason,
                         const CurrentWalkTask& current_walk,
                         const Sensors& sensors,
                         const Stability& stability) {
                // Force the walk to run at a known rate
                if (run_reason != RunReason::OTHER_TRIGGER)
                    return;

                // Compute time since the last update
                auto time_delta =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                // Set the walk locally so the kick can modify it if needed
                auto walk = new_walk;

                // Always enter the kick if its a new kick, otherwise only enter if we're not done yet
                if (walk.kick) {
                    double current_time    = walk_generator.get_time();
                    double full_period     = walk_generator.get_step_period();
                    WalkState::Phase phase = walk_generator.get_phase();

                    // If the current time in the walk generator phase is either the end of the other leg or the start
                    // of the desired leg, start the kick step and increase the velocity of the walk generator when it
                    // is currently executing a kick step and it is at the end of the desired leg, stop.
                    if (!kick_step_in_progress) {
                        // Check if the conditions allow for the kick to start
                        // Todo maybe set offset based on the frequency
                        bool end_of = (current_time + cfg.kick_timing_offset) >= full_period;

                        // If the leg we want to kick with is planted, we can kick at the end of the step
                        bool other_support = walk.leg == LimbID::LEFT_LEG ? phase == WalkState::Phase::LEFT
                                                                          : phase == WalkState::Phase::RIGHT;
                        // If its the end of the kick foot's support, we can kick
                        if (end_of && other_support) {
                            log<INFO>("Kick step started");
                            kick_step_in_progress = true;
                            walk.velocity_target  = Eigen::Vector3d(cfg.kick_velocity_x, cfg.kick_velocity_y, 0.0);
                        }
                    }
                    else {
                        // Check if the step can end
                        bool end_of = (current_time + cfg.kick_timing_offset) >= full_period;
                        // On the correct support foot, so can't be the beginning of the kick
                        bool correct_support = walk.leg == LimbID::LEFT_LEG ? phase == WalkState::Phase::RIGHT
                                                                            : phase == WalkState::Phase::LEFT;
                        // If its not the end of the kick and we're not on the right support, we shouldn't be kicking
                        if ((end_of && correct_support) || (!end_of && !correct_support)) {
                            kick_step_in_progress = false;
                            emit<Task>(std::make_unique<Done>());
                            log<INFO>("Kick step ended");
                            return;
                        }
                        // Otherwise continue to kick
                        walk.velocity_target = Eigen::Vector3d(cfg.kick_velocity_x, cfg.kick_velocity_y, 0.0);
                    }
                }

                if (!kick_step_in_progress) {
                    // If the new walk command is larger than the current one, accelerate towards it
                    auto dv = cfg.acceleration * std::min(time_delta, 1.0);  // never accelerate too much
                    walk.velocity_target =
                        current_walk.velocity_target
                        + (new_walk.velocity_target - current_walk.velocity_target).cwiseMax(-dv).cwiseMin(dv);
                }

                // Update the state for next time
                emit(std::make_unique<CurrentWalkTask>(walk));

                // Update the walk engine and emit the stability state, only if not falling/fallen
                if (stability != Stability::FALLEN) {
                    switch (walk_generator.update(time_delta, walk.velocity_target, sensors.planted_foot_phase).value) {
                        case WalkState::State::STARTING:
                        case WalkState::State::WALKING:
                        case WalkState::State::STOPPING: emit(std::make_unique<Stability>(Stability::DYNAMIC)); break;
                        case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                        case WalkState::State::UNKNOWN:
                        default: NUClear::log<NUClear::LogLevel::WARN>("Unknown state."); break;
                    }
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
                auto walk_state = std::make_unique<WalkState>(walk_generator.get_state(),
                                                              walk.velocity_target,
                                                              walk_generator.get_phase());

                // Debugging
                if (log_level <= DEBUG) {
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
                    emit(graph("Raw walk velocity target",
                               new_walk.velocity_target.x(),
                               new_walk.velocity_target.y(),
                               new_walk.velocity_target.z()));
                    emit(graph("Smooth walk velocity target",
                               walk.velocity_target.x(),
                               walk.velocity_target.y(),
                               walk.velocity_target.z()));
                    emit(graph("Walk phase", int(walk_generator.get_phase())));
                    emit(graph("Walk time", walk_generator.get_time()));
                    emit(graph("Kick step in progress", kick_step_in_progress ? 1 : 0));

                    // Sample trajectory points
                    const int num_samples = 10;
                    for (double t = 0; t <= walk_generator.get_step_period();
                         t += walk_generator.get_step_period() / num_samples) {
                        walk_state->torso_trajectory.push_back(walk_generator.get_torso_pose(t).matrix());
                        walk_state->swing_foot_trajectory.push_back(walk_generator.get_swing_foot_pose(t).matrix());
                    }
                    auto Htp        = walk_generator.get_phase() == WalkState::Phase::RIGHT
                                          ? sensors.Htx[FrameID::R_FOOT_BASE]
                                          : sensors.Htx[FrameID::L_FOOT_BASE];
                    walk_state->Hwp = sensors.Htw.inverse() * Htp;
                }
                emit(walk_state);
            });
    }

}  // namespace module::skill
