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
#ifndef MODULE_SKILL_K1BLOCKPOLICY_HPP
#define MODULE_SKILL_K1BLOCKPOLICY_HPP

#include <Eigen/Core>
#include <array>
#include <deque>
#include <memory>
#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs the K1 block (save) policy trained in the mjlab goalkeeper task at 50 Hz and streams
    /// joint targets to the platform as a Director-arbitrated K1Servos subtask (CUSTOM mode), the
    /// same low-level path as K1KickPolicy / K1WalkPolicy.
    ///
    /// The policy stays on its feet: it holds a ready stance while the Block task is inactive and
    /// shuffles/steps/extends a limb to put a body part on the ball's path while it is active. It
    /// is driven by a command, not by raw ball state, so the save planner owns the prediction and
    /// the policy's capability envelope is measured over exactly the quantities the planner
    /// commands. See README.md for the observation/action contract (v0).
    class K1BlockPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        /// All K1 joints, in the Booster SDK JointIndexK1 serial order
        static constexpr std::size_t JOINT_COUNT = 22;
        /// Head joints in JointIndexK1 order; never policy controlled (vision owns the head)
        static constexpr std::size_t HEAD_YAW   = 0;
        static constexpr std::size_t HEAD_PITCH = 1;
        /// Block command length: [active, dy, time_to_arrival, ball_speed]
        static constexpr std::size_t COMMAND_DIM = 4;

        explicit K1BlockPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Inference device: "gpu" (ONNX Runtime's TensorRT execution provider, falling back to
            /// the CPU when it cannot be set up) or "cpu"
            std::string device = "cpu";
            /// Number of frames in the observation window fed to the ONNX (1 = no history). The
            /// input is time-major, oldest frame first: [1, history_window * frame_dim]
            std::size_t history_window = 1;
            /// JointIndexK1 indices of the policy-controlled joints, in policy order
            std::vector<std::size_t> policy_joints{};
            /// Commanded pose is blended from the current pose into the policy target over this
            /// window (s), since deployment starts from wherever the previous skill left the robot
            double handoff_blend = 0.3;
            /// Command clipping, matching the training ranges
            double dy_limit            = 1.5;
            double time_to_arrival_max = 3.0;
            double ball_speed_max      = 6.0;
            /// Head tracking gains (the head follows the latest BoosterHeadRot)
            double head_kp = 10.0;
            double head_kd = 0.5;
            /// Training-time actuation, all in JointIndexK1 order
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
            std::array<double, JOINT_COUNT> joint_lower{};
            std::array<double, JOINT_COUNT> joint_upper{};
        } cfg;

        /// Length of one observation frame: gyro(3) + gravity(3) + 3 * n_policy_joints + command
        [[nodiscard]] std::size_t frame_dim() const {
            return 6 + 3 * cfg.policy_joints.size() + COMMAND_DIM;
        }

        /// Load the ONNX and check its input/output sizes against the configured contract
        void load_model();

        /// Run the network on a flat observation window, returning n_policy_joints actions
        std::vector<float> infer(const std::vector<float>& input);

        /// ONNX Runtime session for the policy, nullptr until a model loads
        std::unique_ptr<utility::onnx::ONNXRuntime> onnx_rt{};
        bool model_loaded = false;

        /// Previous raw policy output (policy order), fed back as the last-action observation
        std::vector<float> last_action{};
        /// Observation window, oldest frame first
        std::deque<std::vector<float>> history{};
        /// Latest head target [yaw, pitch] from BoosterHeadRot
        Eigen::Vector2d head_target = Eigen::Vector2d::Zero();
        /// When the current Block task started (drives the hand-off blend)
        NUClear::clock::time_point block_since{};
        /// Monotonic tick counter for the BLOCKOBS trace
        uint64_t tick = 0;
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1BLOCKPOLICY_HPP
