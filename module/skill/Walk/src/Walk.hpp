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
#ifndef MODULE_SKILL_WALK_HPP
#define MODULE_SKILL_WALK_HPP

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs an mjlab K1 velocity-tracking walk policy at 50 Hz and emits the joint targets as a
    /// Director-arbitrated Servos subtask (honoured in CUSTOM mode).
    ///
    /// The observation contract is configured. One frame is
    ///
    ///     gyro(3) | projected gravity(3) | q - default(N) | dq(N) | last action(N) | command(3)
    ///     [ | gait clock(2), when gait_clock is set ]
    ///
    /// over the `policy_joints` joints, and the ONNX input is the last `history_window` frames,
    /// oldest first. Observation normalisation is baked into the graph. See README.md.
    class Walk : public ::extension::behaviour::BehaviourReactor {
    public:
        /// All K1 joints, in the Booster SDK JointIndexK1 serial order
        static constexpr std::size_t JOINT_COUNT = 22;
        /// Head joints in JointIndexK1 order
        static constexpr std::size_t HEAD_YAW   = 0;
        static constexpr std::size_t HEAD_PITCH = 1;
        /// Velocity command length: [vx, vy, wz]
        static constexpr std::size_t COMMAND_DIM = 3;
        /// Gait clock length: [sin(2*pi*phase), cos(2*pi*phase)]
        static constexpr std::size_t CLOCK_DIM = 2;

        explicit Walk(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Inference device: "cpu", or "gpu" for TensorRT with a CPU fallback
            std::string device = "cpu";
            /// Number of frames in the observation window (1 = no history)
            std::size_t history_window = 1;
            /// JointIndexK1 indices the policy observes and acts on, in policy order
            std::vector<std::size_t> policy_joints{};
            /// Whether the observation frame ends with the [sin, cos] gait clock
            bool gait_clock = false;
            /// Gait cycle duration (s) of the observed clock, as in training
            double gait_period = 0.6;
            /// Command magnitude (|v_xy| + |wz|) at or below which the observed clock is (0, 0)
            double command_threshold = 0.05;
            /// Time (s) to blend from the measured pose into the policy target after a walk starts
            double handoff_blend = 0.3;
            /// Whether the policy drives the head. Otherwise the head tracks BoosterHeadRot.
            bool head_policy_controlled = false;
            /// Head gains used when the head tracks BoosterHeadRot
            double head_kp = 10.0;
            double head_kd = 0.5;
            /// Velocity command applied while performing an in-walk kick
            Eigen::Vector3d kick_velocity = Eigen::Vector3d::Zero();
            /// How long to drive the kick velocity before reporting the kick as done
            NUClear::clock::duration kick_duration{};
            /// Training-time actuation, in JointIndexK1 order
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
            /// Joint ranges the commanded position is clamped to, in JointIndexK1 order
            std::array<double, JOINT_COUNT> joint_lower{};
            std::array<double, JOINT_COUNT> joint_upper{};
        } cfg;

        /// Length of one observation frame
        [[nodiscard]] std::size_t frame_dim() const {
            return 6 + 3 * cfg.policy_joints.size() + COMMAND_DIM + (cfg.gait_clock ? CLOCK_DIM : 0);
        }

        /// Loads the ONNX and checks its input and output sizes against the configured contract
        void load_model();

        /// Clears the observation history, the last action and the gait phase, for when the next
        /// tick does not continue the previous one
        void reset_policy_state();

        /// ONNX Runtime session for the policy, nullptr until a model loads
        std::unique_ptr<utility::onnx::ONNXRuntime> onnx_rt{};
        bool model_loaded = false;

        /// Previous raw policy output, in policy order
        std::vector<float> last_action{};
        /// Observation window, oldest frame first
        std::deque<std::vector<float>> history{};

        /// Gait clock phase in [0, 1), advanced on the measured loop period
        double gait_phase = 0.0;

        /// Time of the previous policy tick
        bool have_last_tick = false;
        NUClear::clock::time_point last_tick_time{};

        /// Loop-rate statistics since the last report: ticks off the trained 50 Hz, the tick the
        /// window started at, the shortest and longest period, and Director runs skipped
        std::uint64_t off_rate_ticks     = 0;
        std::uint64_t timing_report_tick = 0;
        double min_tick_dt               = std::numeric_limits<double>::infinity();
        double max_tick_dt               = 0.0;
        std::uint64_t director_runs      = 0;
        NUClear::clock::time_point last_timing_report{};

        /// When the current walk task started, for the hand-off blend
        NUClear::clock::time_point walk_since{};

        /// Tick counter and the robot's last reported mode, logged with every observation
        std::uint64_t tick = 0;
        int last_mode      = -1;

        /// When CUSTOM mode was last requested from the tick loop
        NUClear::clock::time_point last_mode_request{};

        /// Latest clamped head target (yaw, pitch) from BoosterHeadRot
        Eigen::Vector2d head_target = Eigen::Vector2d::Zero();

        /// Time the current in-walk kick started
        NUClear::clock::time_point kick_start_time{};

        /// Last emitted walk state and command, so WalkState is only emitted on a change
        int last_walk_state               = -1;
        Eigen::Vector3d last_walk_command = Eigen::Vector3d::Zero();
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
