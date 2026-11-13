/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/format.h>
#include <iomanip>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/actuation/Servos.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterModeState.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::Servos;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::BoosterModeState;
    using message::booster::K1Mode;
    using message::platform::RawSensors;
    using WalkTask = message::skill::Walk;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    namespace {

        constexpr double TWO_PI = 6.283185307179586;

        // Booster SDK RotateHead limits: pitch down-positive [-0.3, 1.0], yaw [-0.785, 0.785]
        constexpr double HEAD_PITCH_MIN = -0.3;
        constexpr double HEAD_PITCH_MAX = 1.0;
        constexpr double HEAD_YAW_LIMIT = 0.785;

        // Bounds on the measured loop period used to advance the gait phase
        constexpr double MIN_TICK_DT = 0.005;
        constexpr double MAX_TICK_DT = 0.100;

        // The training control period, and the error in it past which a tick counts as off rate
        constexpr double NOMINAL_TICK_DT   = 0.02;
        constexpr double TICK_DT_TOLERANCE = 0.005;

        // How often the loop-rate warning may be logged
        constexpr std::chrono::seconds TIMING_REPORT_PERIOD{2};

        // How often to re-ask for CUSTOM while the robot is not in it
        constexpr std::chrono::seconds MODE_RETRY_PERIOD{1};

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("Walk.yaml: ") + key + " must have " + std::to_string(N)
                                         + " entries (JointIndexK1 order), got " + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

        /// Servo feedback in JointIndexK1 order (ankles pre-converted to serial by the platform)
        std::array<const RawSensors::Servo*, Walk::JOINT_COUNT> servos_of(const RawSensors& raw) {
            return {&raw.servo.head_pan,         &raw.servo.head_tilt,       &raw.servo.l_shoulder_pitch,
                    &raw.servo.l_shoulder_roll,  &raw.servo.l_elbow,         &raw.servo.l_elbow_yaw,
                    &raw.servo.r_shoulder_pitch, &raw.servo.r_shoulder_roll, &raw.servo.r_elbow,
                    &raw.servo.r_elbow_yaw,      &raw.servo.l_hip_pitch,     &raw.servo.l_hip_roll,
                    &raw.servo.l_hip_yaw,        &raw.servo.l_knee,          &raw.servo.l_ankle_pitch,
                    &raw.servo.l_ankle_roll,     &raw.servo.r_hip_pitch,     &raw.servo.r_hip_roll,
                    &raw.servo.r_hip_yaw,        &raw.servo.r_knee,          &raw.servo.r_ankle_pitch,
                    &raw.servo.r_ankle_roll};
        }

        template <typename Shape>
        std::size_t numel(const Shape& shape) {
            return std::accumulate(shape.begin(), shape.end(), std::size_t{1}, [](std::size_t n, auto d) {
                return n * static_cast<std::size_t>(d);
            });
        }

    }  // namespace

    void Walk::reset_policy_state() {
        std::fill(last_action.begin(), last_action.end(), 0.0f);
        history.clear();
        gait_phase     = 0.0;
        have_last_tick = false;

        off_rate_ticks     = 0;
        director_runs      = 0;
        min_tick_dt        = std::numeric_limits<double>::infinity();
        max_tick_dt        = 0.0;
        timing_report_tick = tick;
        last_timing_report = NUClear::clock::now();
    }

    Walk::Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration, Sync<Walk>>("Walk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path             = config["model_path"].as<std::string>();
            cfg.device                 = config["device"].as<std::string>();
            cfg.history_window         = config["history_window"].as<std::size_t>();
            cfg.gait_clock             = config["gait_clock"].as<bool>();
            cfg.gait_period            = config["gait_period"].as<double>();
            cfg.command_threshold      = config["command_threshold"].as<double>();
            cfg.handoff_blend          = config["handoff_blend"].as<double>();
            cfg.head_policy_controlled = config["head"]["policy_controlled"].as<bool>();
            cfg.head_kp                = config["head"]["kp"].as<double>();
            cfg.head_kd                = config["head"]["kd"].as<double>();
            cfg.kick_velocity          = Eigen::Vector3d(config["kick"]["velocity"].as<Expression>());
            cfg.kick_duration          = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick"]["duration"].as<double>()));

            if (cfg.history_window < 1) {
                throw std::runtime_error("Walk.yaml: history_window must be >= 1");
            }
            if (cfg.gait_clock && cfg.gait_period <= 0.0) {
                throw std::runtime_error("Walk.yaml: gait_period must be > 0 when gait_clock is set");
            }

            // The policy joints must be distinct and in range
            cfg.policy_joints = config["policy_joints"].as<std::vector<std::size_t>>();
            std::set<std::size_t> seen{};
            for (const std::size_t j : cfg.policy_joints) {
                if (j >= JOINT_COUNT || !seen.insert(j).second) {
                    throw std::runtime_error("Walk.yaml: policy_joints must be distinct JointIndexK1 indices in [0, "
                                             + std::to_string(JOINT_COUNT) + "), got " + std::to_string(j));
                }
            }
            // The policy can only drive the head if it has head actions
            if (cfg.head_policy_controlled && (seen.count(HEAD_YAW) == 0 || seen.count(HEAD_PITCH) == 0)) {
                throw std::runtime_error(
                    "Walk.yaml: head.policy_controlled is set but the head joints are "
                    "not in policy_joints, so the policy has no head action to apply");
            }

            cfg.kp                    = load_joint_array<JOINT_COUNT>(config, "kp");
            cfg.kd                    = load_joint_array<JOINT_COUNT>(config, "kd");
            cfg.action_scale_joint    = load_joint_array<JOINT_COUNT>(config, "action_scale_joint");
            cfg.default_pose          = load_joint_array<JOINT_COUNT>(config, "default_pose");
            cfg.joint_lower           = load_joint_array<JOINT_COUNT>(config, "joint_lower");
            cfg.joint_upper           = load_joint_array<JOINT_COUNT>(config, "joint_upper");
            const double action_scale = config["action_scale"].as<double>();
            for (double& s : cfg.action_scale_joint) {
                s *= action_scale;
            }

            last_action.assign(cfg.policy_joints.size(), 0.0f);
            reset_policy_state();
            load_model();

            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        // Cached rather than a With<>, as BoosterModeState is not emitted in NUSim
        on<Trigger<BoosterModeState>, Sync<Walk>>().then([this](const BoosterModeState& state) {  //
            last_mode = int(state.mode);
        });

        // Cache the head target the look skills last asked for
        on<Trigger<BoosterHeadRot>, Sync<Walk>>().then([this](const BoosterHeadRot& head) {
            head_target.x() = std::clamp(head.rot.x(), -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT);
            head_target.y() = std::clamp(head.rot.y(), HEAD_PITCH_MIN, HEAD_PITCH_MAX);
        });

        on<Start<WalkTask>, Sync<Walk>>().then([this]() {
            // Without a policy to stream joint commands the robot would collapse in CUSTOM mode
            if (!model_loaded) {
                log<ERROR>("Walk task started but no walk policy is loaded; staying out of CUSTOM mode");
                return;
            }
            tick = 0;
            reset_policy_state();
            walk_since = NUClear::clock::now();

            // Low-level joint commands are only honoured in CUSTOM mode
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));

            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state   = int(WalkState::State::STOPPED);
            last_walk_command = Eigen::Vector3d::Zero();
        });

        on<Stop<WalkTask>, Sync<Walk>>().then([this] {
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state   = int(WalkState::State::STOPPED);
            last_walk_command = Eigen::Vector3d::Zero();
        });

        // 50 Hz inference loop, matching the training control rate. High priority so a busy thread
        // pool does not run the tick late.
        on<Provide<WalkTask>,
           Every<50, Per<std::chrono::seconds>>,
           With<RawSensors>,
           With<Stability>,
           Single,
           Sync<Walk>,
           Priority::HIGH>()
            .then([this](const WalkTask& walk,
                         const RunReason& run_reason,
                         const RawSensors& raw,
                         const Stability& stability) {
                if (!model_loaded) {
                    return;
                }

                // Step the policy only on the timer. The Director also runs this provider when the Walk
                // task is re-emitted, and stepping then would break the trained 50 Hz period.
                if (run_reason != RunReason::OTHER_TRIGGER) {
                    if (walk.kick && run_reason == RunReason::NEW_TASK) {
                        kick_start_time = NUClear::clock::now();
                        log<INFO>("Walk starting in-walk kick");
                    }
                    ++director_runs;
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }
                // The get-up skill owns the servos while fallen, and the history must not span a fall
                if (stability == Stability::FALLEN) {
                    reset_policy_state();
                    return;
                }

                // Re-request CUSTOM until the robot reports it, as the Walk task can start before the
                // model has loaded, in which case Start<WalkTask> did not request it
                if (last_mode != int(K1Mode::CUSTOM) && NUClear::clock::now() - last_mode_request > MODE_RETRY_PERIOD) {
                    last_mode_request = NUClear::clock::now();
                    auto mode         = std::make_unique<BoosterMode>();
                    mode->mode        = K1Mode::CUSTOM;
                    emit(std::move(mode));
                }

                // The policy has no kick, so emulate one as a velocity burst held for kick_duration
                Eigen::Vector3d cmd = walk.velocity_target;
                if (walk.kick) {
                    if (NUClear::clock::now() - kick_start_time > cfg.kick_duration) {
                        log<INFO>("Walk in-walk kick complete");
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    cmd = cfg.kick_velocity;
                    emit<Task>(std::make_unique<Continue>());
                }

                // Measured control period, which the gait phase advances on
                const auto now = NUClear::clock::now();
                double tick_dt = NOMINAL_TICK_DT;
                if (have_last_tick) {
                    tick_dt = std::chrono::duration<double>(now - last_tick_time).count();
                    off_rate_ticks += std::abs(tick_dt - NOMINAL_TICK_DT) > TICK_DT_TOLERANCE ? 1 : 0;
                    min_tick_dt = std::min(min_tick_dt, tick_dt);
                    max_tick_dt = std::max(max_tick_dt, tick_dt);
                    tick_dt     = utility::math::clamp(MIN_TICK_DT, tick_dt, MAX_TICK_DT);
                }
                last_tick_time = now;
                have_last_tick = true;

                // Report ticks that missed the trained period
                if (now - last_timing_report > TIMING_REPORT_PERIOD) {
                    if (off_rate_ticks > 0) {
                        log<WARN>(
                            fmt::format("Walk missed the trained 50 Hz on {} of the last {} ticks "
                                        "(period {:.1f}-{:.1f} ms, {} Director runs skipped); the policy is "
                                        "seeing the wrong dynamics",
                                        off_rate_ticks,
                                        tick - timing_report_tick,
                                        min_tick_dt * 1e3,
                                        max_tick_dt * 1e3,
                                        director_runs));
                    }
                    off_rate_ticks     = 0;
                    director_runs      = 0;
                    min_tick_dt        = std::numeric_limits<double>::infinity();
                    max_tick_dt        = 0.0;
                    timing_report_tick = tick;
                    last_timing_report = now;
                }

                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Policy command", cmd.x(), cmd.y(), cmd.z()));
                    emit(graph("Policy loop period (s)", tick_dt));
                }

                const auto servos = servos_of(raw);

                // Observation frame, see README.md
                std::vector<float> frame{};
                frame.reserve(frame_dim());

                // [0:3] gyro, body frame
                frame.push_back(raw.gyroscope.x());
                frame.push_back(raw.gyroscope.y());
                frame.push_back(raw.gyroscope.z());

                // [3:6] projected gravity: world (0, 0, -1) in the body frame
                const Eigen::Matrix3d Rwt =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                const Eigen::Vector3d gravity = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                frame.push_back(static_cast<float>(gravity.x()));
                frame.push_back(static_cast<float>(gravity.y()));
                frame.push_back(static_cast<float>(gravity.z()));

                // q - default_pose, then dq, over the policy joints
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(static_cast<float>(servos[j]->present_position - cfg.default_pose[j]));
                }
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(servos[j]->present_velocity);
                }

                // Previous raw network output
                frame.insert(frame.end(), last_action.begin(), last_action.end());

                // Command [vx, vy, wz]
                frame.push_back(static_cast<float>(cmd.x()));
                frame.push_back(static_cast<float>(cmd.y()));
                frame.push_back(static_cast<float>(cmd.z()));

                // Gait clock, (0, 0) while the command is below the threshold
                if (cfg.gait_clock) {
                    const double command_magnitude = cmd.head<2>().norm() + std::abs(cmd.z());
                    const bool clock_active        = command_magnitude > cfg.command_threshold;
                    frame.push_back(clock_active ? static_cast<float>(std::sin(TWO_PI * gait_phase)) : 0.0f);
                    frame.push_back(clock_active ? static_cast<float>(std::cos(TWO_PI * gait_phase)) : 0.0f);
                }

                // Observation window, seeded by repeating the first frame as in training
                if (history.empty()) {
                    history.assign(cfg.history_window, frame);
                }
                else {
                    history.push_back(frame);
                    while (history.size() > cfg.history_window) {
                        history.pop_front();
                    }
                }
                std::vector<float> input{};
                input.reserve(cfg.history_window * frame_dim());
                for (const auto& f : history) {
                    input.insert(input.end(), f.begin(), f.end());
                }

                // Inference
                const std::vector<float> action = onnx_rt->infer(input);
                std::copy_n(action.begin(), last_action.size(), last_action.begin());

                // The phase advances regardless of the command, only the observed clock is gated
                if (cfg.gait_clock) {
                    gait_phase = std::fmod(gait_phase + tick_dt / cfg.gait_period, 1.0);
                }

                // Observation trace, with the tick, mode and time to segment a capture on
                if (log_level <= NUClear::LogLevel::TRACE) {
                    std::ostringstream line;
                    line << "WALKOBS " << tick << " mode=" << last_mode
                         << " t=" << std::chrono::duration<double>(now.time_since_epoch()).count() << " dt=" << tick_dt;
                    for (const float v : frame) {
                        line << ' ' << v;
                    }
                    log<TRACE>(line.str());
                }
                ++tick;

                // Joint targets: action offsets on the default pose for the policy joints
                std::array<double, JOINT_COUNT> target = cfg.default_pose;
                std::array<double, JOINT_COUNT> kp     = cfg.kp;
                std::array<double, JOINT_COUNT> kd     = cfg.kd;
                for (std::size_t k = 0; k < cfg.policy_joints.size(); ++k) {
                    const std::size_t j = cfg.policy_joints[k];
                    target[j]           = cfg.default_pose[j] + cfg.action_scale_joint[j] * last_action[k];
                }
                // The look skills own the head unless the policy is configured to
                if (!cfg.head_policy_controlled) {
                    target[HEAD_YAW]   = head_target.x();
                    target[HEAD_PITCH] = head_target.y();
                    kp[HEAD_YAW] = kp[HEAD_PITCH] = cfg.head_kp;
                    kd[HEAD_YAW] = kd[HEAD_PITCH] = cfg.head_kd;
                }

                // Blend from the measured pose into the policy target after a walk starts
                const double blend_age = std::chrono::duration<double>(now - walk_since).count();
                const double alpha     = cfg.handoff_blend > 0.0 ? std::min(1.0, blend_age / cfg.handoff_blend) : 1.0;

                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                std::array<double, JOINT_COUNT> cmd_q{};
                // Number of targets the joint-range clamp moved this tick
                int clamped = 0;
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const double blended = (1.0 - alpha) * servos[j]->present_position + alpha * target[j];
                    const double q       = std::clamp(blended, cfg.joint_lower[j], cfg.joint_upper[j]);
                    clamped += q != blended ? 1 : 0;

                    auto& motor  = low->motor_cmd[j];
                    motor.mode   = 1;
                    motor.q      = static_cast<float>(q);
                    motor.dq     = 0.0f;
                    motor.tau    = 0.0f;
                    motor.kp     = static_cast<float>(kp[j]);
                    motor.kd     = static_cast<float>(kd[j]);
                    motor.weight = 0.0f;
                    cmd_q[j]     = q;
                }
                if (clamped > 0) {
                    log<DEBUG>("Clamped", clamped, "joint targets to the joint limits");
                }

                // A Servos subtask rather than a raw BoosterLowCmd, so the Director can give the
                // servos to a higher priority kick or get-up
                auto servos_task     = std::make_unique<Servos>();
                servos_task->command = *low;
                emit<Task>(std::move(servos_task));

                const auto state = cmd.isZero() ? WalkState::State::STOPPED : WalkState::State::WALKING;
                if (state == WalkState::State::WALKING && log_level <= NUClear::LogLevel::DEBUG) {
                    std::ostringstream out;
                    out << std::fixed << std::setprecision(4);
                    out << "Walk sim2real"
                        << " cmd=[" << cmd.x() << ',' << cmd.y() << ',' << cmd.z() << ']';
                    if (cfg.gait_clock) {
                        out << " phase=" << gait_phase;
                    }
                    out << " clamped=" << clamped;
                    std::ostringstream action_stream;
                    std::ostringstream joint_pos_rel_stream;
                    std::ostringstream joint_vel_stream;
                    std::ostringstream cmd_q_stream;
                    for (std::size_t k = 0; k < cfg.policy_joints.size(); ++k) {
                        const std::size_t j = cfg.policy_joints[k];
                        if (k != 0) {
                            action_stream << ',';
                            joint_pos_rel_stream << ',';
                            joint_vel_stream << ',';
                            cmd_q_stream << ',';
                        }
                        action_stream << last_action[k];
                        joint_pos_rel_stream << (servos[j]->present_position - cfg.default_pose[j]);
                        joint_vel_stream << servos[j]->present_velocity;
                        cmd_q_stream << cmd_q[j];
                    }
                    out << " action=[" << action_stream.str() << ']' << " joint_pos_rel=[" << joint_pos_rel_stream.str()
                        << ']' << " joint_vel=[" << joint_vel_stream.str() << ']' << " cmd_q=[" << cmd_q_stream.str()
                        << ']';
                    log<DEBUG>(out.str());
                }

                // Emit on a command change too, as others report WalkState's velocity as the walk command
                if (int(state) != last_walk_state || cmd != last_walk_command) {
                    emit(std::make_unique<WalkState>(state, cmd));
                    last_walk_state   = int(state);
                    last_walk_command = cmd;
                }
            });
    }

    void Walk::load_model() {
        model_loaded = false;
        onnx_rt.reset();

        const std::size_t expected_in  = cfg.history_window * frame_dim();
        const std::size_t expected_out = cfg.policy_joints.size();
        const auto check               = [&](std::size_t in, std::size_t out) {
            if (in != expected_in || out != expected_out) {
                throw std::runtime_error("ONNX I/O is " + std::to_string(in) + " -> " + std::to_string(out)
                                         + " but the configured contract is " + std::to_string(cfg.history_window)
                                         + " x " + std::to_string(frame_dim()) + " = " + std::to_string(expected_in)
                                         + " -> " + std::to_string(expected_out));
            }
        };

        // Only a failure to set up the GPU falls back to the CPU. FP16 is off, see Walk.yaml.
        try {
            std::string device = cfg.device;
            try {
                onnx_rt = std::make_unique<utility::onnx::ONNXRuntime>(cfg.model_path, device, false);
            }
            catch (const std::exception& e) {
                if (device != "gpu") {
                    throw;
                }
                log<INFO>("TensorRT unavailable, falling back to the CPU:", e.what());
                device  = "cpu";
                onnx_rt = std::make_unique<utility::onnx::ONNXRuntime>(cfg.model_path, device, false);
            }
            check(numel(onnx_rt->input_shape()), numel(onnx_rt->output_shape()));
            model_loaded = true;
            log<INFO>("Loaded walk policy on", device, cfg.model_path);
        }
        catch (const std::exception& e) {
            onnx_rt.reset();
            log<ERROR>("Failed to load walk policy", cfg.model_path, e.what());
        }
    }

}  // namespace module::skill
