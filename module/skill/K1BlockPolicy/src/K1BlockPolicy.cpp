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
#include "K1BlockPolicy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

#include "extension/Configuration.hpp"

#include "message/actuation/K1Servos.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/skill/Block.hpp"

#include "utility/math/euler.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::K1Servos;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::K1Mode;
    using message::platform::RawSensors;
    using BlockTask = message::skill::Block;

    using utility::math::euler::rpy_intrinsic_to_mat;

    namespace {

        // Head command limits, shared with K1WalkPolicy
        constexpr double HEAD_PITCH_MIN = -0.3;
        constexpr double HEAD_PITCH_MAX = 1.0;
        constexpr double HEAD_YAW_LIMIT = 0.785;

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("K1BlockPolicy.yaml: ") + key + " must have " + std::to_string(N)
                                         + " entries (JointIndexK1 order), got " + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

        /// Servo feedback in JointIndexK1 order (ankles pre-converted to serial by the platform)
        std::array<const RawSensors::Servo*, K1BlockPolicy::JOINT_COUNT> servos_of(const RawSensors& raw) {
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

    K1BlockPolicy::K1BlockPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1BlockPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path          = config["model_path"].as<std::string>();
            cfg.device              = config["device"].as<std::string>();
            cfg.history_window      = config["history_window"].as<std::size_t>();
            cfg.handoff_blend       = config["handoff_blend"].as<double>();
            cfg.dy_limit            = config["command_limits"]["dy"].as<double>();
            cfg.time_to_arrival_max = config["command_limits"]["time_to_arrival"].as<double>();
            cfg.ball_speed_max      = config["command_limits"]["ball_speed"].as<double>();
            cfg.head_kp             = config["head"]["kp"].as<double>();
            cfg.head_kd             = config["head"]["kd"].as<double>();

            if (cfg.history_window < 1) {
                throw std::runtime_error("K1BlockPolicy.yaml: history_window must be >= 1");
            }

            // The policy joints must be distinct, in range, and exclude the head (vision owns it)
            cfg.policy_joints = config["policy_joints"].as<std::vector<std::size_t>>();
            std::set<std::size_t> seen{};
            for (const std::size_t j : cfg.policy_joints) {
                if (j >= JOINT_COUNT || j == HEAD_YAW || j == HEAD_PITCH || !seen.insert(j).second) {
                    throw std::runtime_error(
                        "K1BlockPolicy.yaml: policy_joints must be distinct non-head "
                        "JointIndexK1 indices, got "
                        + std::to_string(j));
                }
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
            history.clear();
            load_model();
        });

        // The policy does not own the head: track whatever the look skills last asked for
        on<Trigger<BoosterHeadRot>>().then([this](const BoosterHeadRot& head) {
            head_target.x() = std::clamp(head.rot.x(), -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT);
            head_target.y() = std::clamp(head.rot.y(), HEAD_PITCH_MIN, HEAD_PITCH_MAX);
        });

        on<Start<BlockTask>>().then([this] {
            if (!model_loaded) {
                log<ERROR>("Block task started but no block policy is loaded; staying out of CUSTOM mode");
                return;
            }
            log<INFO>("Blocking (policy)...");
            std::fill(last_action.begin(), last_action.end(), 0.0f);
            history.clear();
            block_since = NUClear::clock::now();
            tick        = 0;

            // Low-level joint commands are only honoured in CUSTOM mode
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));
        });

        on<Stop<BlockTask>>().then([this] { log<INFO>("Stopped blocking (policy)"); });

        // 50 Hz inference loop, matching the training control rate (0.02 s)
        on<Provide<BlockTask>, Every<50, Per<std::chrono::seconds>>, With<RawSensors>, Single>().then(
            [this](const BlockTask& block, const RawSensors& raw) {
                if (!model_loaded) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                const auto servos = servos_of(raw);

                // --- observation frame (contract v0) ---
                std::vector<float> frame{};
                frame.reserve(frame_dim());

                frame.push_back(raw.gyroscope.x());
                frame.push_back(raw.gyroscope.y());
                frame.push_back(raw.gyroscope.z());

                const Eigen::Matrix3d Rwt =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                const Eigen::Vector3d gravity = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                frame.push_back(static_cast<float>(gravity.x()));
                frame.push_back(static_cast<float>(gravity.y()));
                frame.push_back(static_cast<float>(gravity.z()));

                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(static_cast<float>(servos[j]->present_position - cfg.default_pose[j]));
                }
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(servos[j]->present_velocity);
                }
                frame.insert(frame.end(), last_action.begin(), last_action.end());

                // An inactive command is all zeros, exactly as in training
                if (block.active) {
                    frame.push_back(1.0f);
                    frame.push_back(static_cast<float>(std::clamp(block.dy, -cfg.dy_limit, cfg.dy_limit)));
                    frame.push_back(
                        static_cast<float>(std::clamp(block.time_to_arrival, 0.0, cfg.time_to_arrival_max)));
                    frame.push_back(static_cast<float>(std::clamp(block.ball_speed, 0.0, cfg.ball_speed_max)));
                }
                else {
                    frame.insert(frame.end(), COMMAND_DIM, 0.0f);
                }

                // --- observation window: seed by repeating the first frame, as the training-side
                // circular buffer backfills on reset ---
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

                // --- inference ---
                const std::vector<float> action = infer(input);
                std::copy_n(action.begin(), last_action.size(), last_action.begin());

                if (log_level <= TRACE) {
                    std::ostringstream trace{};
                    trace << "BLOCKOBS " << tick;
                    for (const float v : frame) {
                        trace << ' ' << v;
                    }
                    log<TRACE>(trace.str());
                }
                ++tick;

                // --- action -> joint targets: offsets on the default pose for the policy joints, the
                // latest look target for the head, the default pose for anything else ---
                std::array<double, JOINT_COUNT> target = cfg.default_pose;
                std::array<double, JOINT_COUNT> kp     = cfg.kp;
                std::array<double, JOINT_COUNT> kd     = cfg.kd;
                for (std::size_t k = 0; k < cfg.policy_joints.size(); ++k) {
                    const std::size_t j = cfg.policy_joints[k];
                    target[j]           = cfg.default_pose[j] + cfg.action_scale_joint[j] * last_action[k];
                }
                target[HEAD_YAW]   = head_target.x();
                target[HEAD_PITCH] = head_target.y();
                kp[HEAD_YAW] = kp[HEAD_PITCH] = cfg.head_kp;
                kd[HEAD_YAW] = kd[HEAD_PITCH] = cfg.head_kd;

                // Cross-fade from the measured pose into the policy target: training always starts at
                // the default pose, deployment starts from whatever the previous skill left
                const double blend_age = std::chrono::duration<double>(NUClear::clock::now() - block_since).count();
                const double alpha     = cfg.handoff_blend > 0.0 ? std::min(1.0, blend_age / cfg.handoff_blend) : 1.0;

                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                int clamped = 0;
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const double blended = (1.0 - alpha) * servos[j]->present_position + alpha * target[j];
                    // A saturated action can command past the mechanical stops, which the simulator
                    // absorbs silently and the robot does not
                    const double q = std::clamp(blended, cfg.joint_lower[j], cfg.joint_upper[j]);
                    clamped += q != blended ? 1 : 0;

                    auto& motor  = low->motor_cmd[j];
                    motor.mode   = 1;
                    motor.q      = static_cast<float>(q);
                    motor.dq     = 0.0f;
                    motor.tau    = 0.0f;
                    motor.kp     = static_cast<float>(kp[j]);
                    motor.kd     = static_cast<float>(kd[j]);
                    motor.weight = 0.0f;
                }
                if (clamped > 0) {
                    log<DEBUG>("Clamped", clamped, "joint targets to the joint limits");
                }

                // Emit as a Director-arbitrated K1Servos subtask so the Director owns the low-level
                // channel; the subtask keeps the Block task alive, so no Continue is needed
                auto k1_servos     = std::make_unique<K1Servos>();
                k1_servos->command = *low;
                emit<Task>(std::move(k1_servos));
            });
    }

    void K1BlockPolicy::load_model() {
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

        // A contract mismatch is fatal on any device: falling back would only load the same wrong
        // graph on the CPU. FP16 is off, see K1BlockPolicy.yaml.
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
            log<INFO>("Loaded block policy on", device, cfg.model_path);
        }
        catch (const std::exception& e) {
            onnx_rt.reset();
            log<ERROR>("Failed to load block policy", cfg.model_path, e.what());
        }
    }

    std::vector<float> K1BlockPolicy::infer(const std::vector<float>& input) {
        return onnx_rt->infer(input);
    }

}  // namespace module::skill
