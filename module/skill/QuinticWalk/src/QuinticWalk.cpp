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
#include "QuinticWalk.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::KinematicsModel;
    using message::actuation::LeftArm;
    using message::actuation::LeftLegIK;
    using message::actuation::RightArm;
    using message::actuation::RightLegIK;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::skill::Walk;

    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::skill::WalkEngineState;
    using utility::support::Expression;


    /**
     * @brief loads the configuration from config into config
     * @param config A Configuration provided by the Configuration extension
     * @param config The destination configuration that we will write to.
     */
    void QuinticWalk::load_quintic_walk(const Configuration& config, Config& cfg) {
        cfg.params.freq                          = config["walk"]["freq"].as<double>();
        cfg.params.double_support_ratio          = config["walk"]["double_support_ratio"].as<double>();
        cfg.params.first_step_swing_factor       = config["walk"]["first_step_swing_factor"].as<double>();
        cfg.params.foot_distance                 = config["walk"]["foot"]["distance"].as<double>();
        cfg.params.foot_rise                     = config["walk"]["foot"]["rise"].as<double>();
        cfg.params.foot_z_pause                  = config["walk"]["foot"]["z_pause"].as<double>();
        cfg.params.foot_put_down_z_offset        = config["walk"]["foot"]["put_down"]["z_offset"].as<double>();
        cfg.params.foot_put_down_phase           = config["walk"]["foot"]["put_down"]["phase"].as<double>();
        cfg.params.foot_put_down_roll_offset     = config["walk"]["foot"]["put_down"]["roll_offset"].as<double>();
        cfg.params.foot_apex_phase               = config["walk"]["foot"]["apex_phase"].as<double>();
        cfg.params.foot_overshoot_ratio          = config["walk"]["foot"]["overshoot"]["ratio"].as<double>();
        cfg.params.foot_overshoot_phase          = config["walk"]["foot"]["overshoot"]["phase"].as<double>();
        cfg.params.trunk_height                  = config["walk"]["trunk"]["height"].as<double>();
        cfg.params.trunk_pitch                   = config["walk"]["trunk"]["pitch"].as<Expression>();
        cfg.params.trunk_phase                   = config["walk"]["trunk"]["phase"].as<double>();
        cfg.params.trunk_x_offset                = config["walk"]["trunk"]["x_offset"].as<double>();
        cfg.params.trunk_y_offset                = config["walk"]["trunk"]["y_offset"].as<double>();
        cfg.params.trunk_swing                   = config["walk"]["trunk"]["swing"].as<double>();
        cfg.params.trunk_pause                   = config["walk"]["trunk"]["pause"].as<double>();
        cfg.params.trunk_x_offset_p_coef_forward = config["walk"]["trunk"]["x_offset_p_coef"]["forward"].as<double>();
        cfg.params.trunk_x_offset_p_coef_turn    = config["walk"]["trunk"]["x_offset_p_coef"]["turn"].as<double>();
        cfg.params.trunk_pitch_p_coef_forward =
            1.0 + config["walk"]["trunk"]["pitch_p_coef"]["forward"].as<Expression>();
        cfg.params.trunk_pitch_p_coef_turn = 1.0 + config["walk"]["trunk"]["pitch_p_coef"]["turn"].as<Expression>();
        cfg.params.kick_length             = config["walk"]["kick"]["length"].as<double>();
        cfg.params.kick_phase              = config["walk"]["kick"]["phase"].as<double>();
        cfg.params.kick_vel                = config["walk"]["kick"]["vel"].as<double>();
        cfg.params.pause_duration          = config["walk"]["pause"]["duration"].as<double>();

        cfg.max_step.x() = config["max_step"]["x"].as<double>();
        cfg.max_step.y() = config["max_step"]["y"].as<double>();
        cfg.max_step.z() = config["max_step"]["z"].as<double>();
        cfg.max_step_xy  = config["max_step"]["xy"].as<double>();

        cfg.imu_active          = config["imu"]["active"].as<bool>();
        cfg.imu_pitch_threshold = 1.0 + config["imu"]["pitch"]["threshold"].as<double>();
        cfg.imu_roll_threshold  = config["imu"]["roll"]["threshold"].as<double>();

        for (int id = 0; id < ServoID::NUMBER_OF_SERVOS; ++id) {
            // Sets the leg gains
            if ((id >= ServoID::R_HIP_YAW) && (id < ServoID::HEAD_YAW)) {
                cfg.servo_states[id] = ServoState(config["gains"]["legs"].as<double>(), 100);
            }
            // Sets the arm gains
            if (id < ServoID::R_HIP_YAW) {
                cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<double>(), 100);
            }
        }

        cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH, config["arms"]["right_shoulder_pitch"].as<double>());
        cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH, config["arms"]["left_shoulder_pitch"].as<double>());
        cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL, config["arms"]["right_shoulder_roll"].as<double>());
        cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<double>());
        cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<double>());
        cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<double>());
    }

    QuinticWalk::QuinticWalk(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        imu_reaction = on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            Eigen::Vector3d RPY = utility::math::euler::mat_to_rpy_intrinsic(sensors.Htw.rotation().cast<double>());

            // compute the pitch offset to the currently wanted pitch of the engine
            double wanted_pitch =
                current_cfg.params.trunk_pitch
                + current_cfg.params.trunk_pitch_p_coef_forward * walk_engine.get_footstep().get_next().x()
                + current_cfg.params.trunk_pitch_p_coef_turn * std::abs(walk_engine.get_footstep().get_next().z());
            RPY.y() += wanted_pitch;

            // threshold pitch and roll
            if (std::abs(RPY.x()) > current_cfg.imu_roll_threshold) {
                log<WARN>(fmt::format("Robot roll exceeds threshold - {} > {}",
                                      std::abs(RPY.x()),
                                      current_cfg.imu_roll_threshold));
                walk_engine.request_pause();
            }
            else if (std::abs(RPY.y()) > current_cfg.imu_pitch_threshold) {
                log<WARN>(fmt::format("Robot pitch exceeds threshold - {} > {}",
                                      std::abs(RPY.y()),
                                      current_cfg.imu_pitch_threshold));
                walk_engine.request_pause();
            }
        });

        on<Configuration>("QuinticWalk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            load_quintic_walk(config, normal_cfg);

            // Make sure the walk engine has the parameters at least once
            if (first_cfg) {
                // Send these parameters to the walk engine
                walk_engine.set_parameters(current_cfg.params);

                imu_reaction.enable(current_cfg.imu_active);

                first_cfg = false;
            }
        });

        on<Configuration>("goalie/QuinticWalk.yaml").then([this](const Configuration& config) {
            load_quintic_walk(config, goalie_cfg);
        });

        // Runs every time the Walk provider starts (wasn't running)
        on<Start<Walk>, With<Behaviour::State>, With<KinematicsModel>>().then(
            [this](const Behaviour::State& behaviour, const KinematicsModel& model) {
                kinematicsModel = model;
                first_run       = true;
                current_orders.setZero();
                is_left_support  = true;
                last_update_time = NUClear::clock::now();
                walk_engine.reset();
                if (behaviour == Behaviour::State::GOALIE_WALK) {
                    current_cfg = goalie_cfg;
                }
                else {
                    current_cfg = normal_cfg;
                }
                // Send these parameters to the walk engine
                walk_engine.set_parameters(current_cfg.params);

                imu_reaction.enable(current_cfg.imu_active);

                // Update the walking state
                emit(std::make_unique<WalkState>(WalkState::State::WALKING, Eigen::Vector3d::Zero()));
            });

        // Runs every time the Walk task is removed from the director tree
        on<Stop<Walk>>().then([this] {
            imu_reaction.enable(false);
            // Update the walking state
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // MAIN LOOP
        on<Provide<Walk>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this](const Walk& walk) {
                // Set the robot stability value based on the walk engine state
                if (walk_engine.get_state() == WalkEngineState::IDLE) {
                    emit(std::make_unique<Stability>(Stability::STANDING));
                }
                else {
                    emit(std::make_unique<Stability>(Stability::DYNAMIC));
                }
                // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is
                // a double step factor 2 since the order distance is only for a single step, not double step
                const double factor            = (1.0 / (current_cfg.params.freq)) * 0.5;
                const Eigen::Vector3d& command = walk.velocity_target.cast<double>() * factor;

                // Clamp velocity command
                current_orders =
                    command.array().max(-current_cfg.max_step.array()).min(current_cfg.max_step.array()).matrix();

                // translational orders (x+y) should not exceed combined limit. scale if necessary
                if (current_cfg.max_step_xy != 0) {
                    double scaling_factor =
                        1.0 / std::max(1.0, (current_orders.x() + current_orders.y()) / current_cfg.max_step_xy);
                    current_orders.cwiseProduct(Eigen::Vector3d(scaling_factor, scaling_factor, 1.0));
                }

                // warn user that speed was limited
                if (command.x() != current_orders.x() || command.y() != current_orders.y()
                    || command.z() != current_orders.z()) {
                    log<WARN>(fmt::format(
                        "Speed command was x: {} y: {} z: {} xy: {} but maximum is x: {} y: {} z: {} xy: {}",
                        command.x(),
                        command.y(),
                        command.z(),
                        command.x() + command.y(),
                        current_cfg.max_step.x() / factor,
                        current_cfg.max_step.y() / factor,
                        current_cfg.max_step.z() / factor,
                        current_cfg.max_step_xy / factor));
                }

                const double dt = get_time_delta();
                // If the walk engine is still running, call the calculate_joint_goals function,
                // which will emit the tasks for the limbs.
                // If there are no new goals, no tasks are emitted, removing child tasks from the Director tree.
                if (walk_engine.update_state(dt, current_orders.cast<float>())) {
                    calculate_joint_goals();
                }
                // Update the walking state
                emit(std::make_unique<WalkState>(WalkState::State::WALKING, walk.velocity_target));
            });

        // Stand Reaction - Sets walk_engine commands to zero, checks walk_engine state, Sets stability state
        on<Provide<Walk>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Causing<Stability, Stability::STANDING>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this] {
                // Stop the walk engine
                const double dt = get_time_delta();
                current_orders.setZero();
                walk_engine.update_state(dt, current_orders.cast<float>());
                // Check if we are in the IDLE state
                if (walk_engine.get_state() == WalkEngineState::IDLE) {
                    emit(std::make_unique<Stability>(Stability::STANDING));
                }
                calculate_joint_goals();

                // Update the walking state
                emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            });
    }


    double QuinticWalk::get_time_delta() {
        // compute time delta depended if we are currently in simulation or reality
        const auto current_time = NUClear::clock::now();
        double dt =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update_time).count() / 1000.0;

        if (dt == 0.0) {
            dt = 0.001;
        }

        // time is wrong when we run it for the first time
        if (first_run) {
            first_run = false;
            dt        = 0.0001;
        }

        last_update_time = current_time;
        return dt;
    }

    void QuinticWalk::calculate_joint_goals() {
        /*
        This method computes the next motor goals and emits limb tasks.
        */
        // Position of trunk {t} relative to support foot {s}
        Eigen::Vector3f rTSs = Eigen::Vector3f::Zero();
        // Euler angles [Roll, Pitch, Yaw] of trunk {t} relative to support foot {s}
        Eigen::Vector3f thetaST = Eigen::Vector3f::Zero();
        // Position of flying foot {f} relative to support foot {s}
        Eigen::Vector3f rFSs = Eigen::Vector3f::Zero();
        // Euler angles [Roll, Pitch, Yaw] of flying foot {f} relative to support foot {s}
        Eigen::Vector3f thetaSF = Eigen::Vector3f::Zero();
        // Read the cartesian positions and orientations for trunk and fly foot
        std::tie(rTSs, thetaST, rFSs, thetaSF, is_left_support) = walk_engine.compute_cartesian_position();

        // Change goals from support foot based coordinate system to trunk based coordinate system
        // Trunk {t} from support foot {s}
        Eigen::Isometry3f Hst;
        Hst.linear()      = rpy_intrinsic_to_mat(thetaST);
        Hst.translation() = rTSs;

        // Flying foot {f} from support foot {s}
        Eigen::Isometry3f Hsf;
        Hsf.linear()      = rpy_intrinsic_to_mat(thetaSF);
        Hsf.translation() = rFSs;

        // Support foot {s} from trunk {t}
        const Eigen::Isometry3f Hts = Hst.inverse();

        // Flying foot {f} from trunk {t}
        const Eigen::Isometry3f Htf = Hts * Hsf;

        // Get desired transform for left foot {l}
        const Eigen::Isometry3f Htl = walk_engine.get_footstep().is_left_support() ? Hts : Htf;

        // Get desired transform for right foot {r}
        const Eigen::Isometry3f Htr = walk_engine.get_footstep().is_left_support() ? Htf : Hts;

        // ****Create limb tasks****
        const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

        //  Legs
        auto left_leg   = std::make_unique<LeftLegIK>();
        auto right_leg  = std::make_unique<RightLegIK>();
        left_leg->time  = time;
        right_leg->time = time;
        left_leg->Htl   = Htl.cast<double>();
        right_leg->Htr  = Htr.cast<double>();
        // Arms
        auto left_arm  = std::make_unique<LeftArm>();
        auto right_arm = std::make_unique<RightArm>();

        // Loop to set the servo states
        for (int id = 0; id < ServoID::NUMBER_OF_SERVOS - 2; ++id) {
            // Set the legs
            if ((id >= ServoID::R_HIP_YAW) && (id % 2 == 0)) {  // right legs
                right_leg->servos[id] = current_cfg.servo_states[ServoID(id)];
            }
            else if ((id >= ServoID::R_HIP_YAW) && (id % 2 == 1)) {  // left legs

                left_leg->servos[id] = current_cfg.servo_states[ServoID(id)];
            }
            else if ((id < ServoID::R_HIP_YAW) && (id % 2 == 0)) {  // right arms
                right_arm->servos[id] = ServoCommand(time,
                                                     current_cfg.arm_positions[ServoID(id)].second,
                                                     current_cfg.servo_states[ServoID(id)]);
            }
            else if ((id < ServoID::R_HIP_YAW) && (id % 2 == 1)) {  // left arms
                left_arm->servos[id] = ServoCommand(time,
                                                    current_cfg.arm_positions[ServoID(id)].second,
                                                    current_cfg.servo_states[ServoID(id)]);
            }
        }

        emit<Task>(left_leg, 0, false, "quintic left leg");
        emit<Task>(right_leg, 0, false, "quintic right leg");

        emit<Task>(left_arm, 0, true, "quintic left arm");
        emit<Task>(right_arm, 0, true, "quintic right arm");

        // Plot graphs of desired trajectories
        if (log_level <= DEBUG) {
            Eigen::Vector3f thetaTL = mat_to_rpy_intrinsic(Htl.linear());
            emit(graph("Left foot desired position (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
            emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));

            Eigen::Vector3f thetaTR = mat_to_rpy_intrinsic(Htr.linear());
            emit(graph("Right foot desired position (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
            emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));

            emit(graph("Trunk desired position (x,y,z)", Hst(0, 3), Hst(1, 3), Hst(2, 3)));
            emit(graph("Trunk desired orientation (r,p,y)", thetaST.x(), thetaST.y(), thetaST.z()));
        }
    }

}  // namespace module::skill
