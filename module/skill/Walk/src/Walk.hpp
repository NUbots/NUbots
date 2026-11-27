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
#include <optional>
#include <string>
#include <vector>

#include "ball_skill.hpp"
#include "ball_skill_policy.hpp"

#include "extension/Behaviour.hpp"

#include "message/input/Sensors.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs the mjlab K1 policies at 50 Hz and emits the joint targets as a Director-arbitrated Servos
    /// subtask (honoured in CUSTOM mode): the velocity-tracking walk (provides skill::Walk) and the six ball
    /// skills (provides skill::BallSkill). Both live in one reactor so a switch between them can carry the
    /// last joint targets over as the next policy's previous action, and a switch between ball skills keeps
    /// their shared observation history.
    ///
    /// The observation contract is configured. One frame is
    ///
    ///     gyro(3) | projected gravity(3) | q - default(N) | dq(N) | last action(N) | command(3)
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
            return 6 + 3 * cfg.policy_joints.size() + COMMAND_DIM;
        }

        /// Loads the ONNX and checks its input and output sizes against the configured contract
        void load_model();

        /// Clears the observation history and the last action, for when the next
        /// tick does not continue the previous one
        void reset_policy_state();

        /// Which policy last sent joint targets
        enum class TargetSource { NONE, WALK, BALL_SKILL };

        /// Emits joint targets as the Servos subtask and records them for a hand-over
        void send_targets(const std::array<double, JOINT_COUNT>& q,
                          const std::array<double, JOINT_COUNT>& kp,
                          const std::array<double, JOINT_COUNT>& kd,
                          TargetSource source);

        /// Whether `source` sent targets recently enough for the next policy to continue from them
        [[nodiscard]] bool handing_over_from(TargetSource source) const;

        /// Requests CUSTOM mode
        void request_custom_mode();

        /// Registers the ball-skill reactions, see BallSkills.cpp
        void setup_ball_skills();

        /// Loads the ball-skill ONNX files
        void load_ball_skills();

        /// The ball observation (x, y, z, visible) in the policies' camera frame for this tick
        [[nodiscard]] std::array<float, ball_skill::BALL_DIM> ball_observation(const message::input::Sensors& sensors);

        /// Ball-skill configuration, from BallSkills.yaml
        struct BallConfig {
            /// ONNX file of each skill, in BallSkill::Type order
            std::array<std::string, ball_skill::KIND_COUNT> model_paths{};
            std::string device = "cpu";
            /// Gains used if a file's metadata gains are placeholders, JointIndexK1 order
            std::array<double, JOINT_COUNT> fallback_kp{};
            std::array<double, JOINT_COUNT> fallback_kd{};
            /// Where the ball observation comes from
            enum class Source { VISION, LOCALISATION } source = Source::LOCALISATION;
            /// Ball radius (m), for placing a vision detection's ray on the ball centre
            double ball_radius = 0.0785;
            /// A ball estimate older than this is not seen this tick
            NUClear::clock::duration max_age{};
            /// Localisation only: advance the estimate by its velocity over its age
            bool predict = true;
            ball_skill::Camera camera{};
            ball_skill::CommandLimits limits{};
            /// Head joint speed limit (rad/s), as the ground kick can exceed the head motor's rating
            double head_rate_limit = 7.85;
            /// The last targets of the other policy count as a hand-over for this long
            NUClear::clock::duration handover_window{};
        } ball_cfg;

        /// The loaded ball-skill policies, in BallSkill::Type order; empty where the file failed to load
        std::array<std::unique_ptr<ball_skill::Policy>, ball_skill::KIND_COUNT> ball_policies{};

        /// Ball-skill observation history, kept across switches between ball skills
        ball_skill::History ball_history{};
        /// Previous raw ball-skill output, JointIndexK1 order
        std::array<float, JOINT_COUNT> ball_last_action{};
        /// The ball observation last seen, held while the ball is out of view
        std::array<float, ball_skill::BALL_DIM> last_ball_seen = ball_skill::NO_BALL;
        /// Latest ball estimate in the world {w}, and when it was measured
        std::optional<Eigen::Vector3d> ball_rBWw{};
        Eigen::Vector3d ball_vBw = Eigen::Vector3d::Zero();
        NUClear::clock::time_point ball_time{};
        /// The ball skill running now, -1 for none
        int ball_kind = -1;
        /// When the current ball-skill task started, and whether it blends in from the measured pose
        NUClear::clock::time_point ball_since{};
        bool ball_blend = false;
        /// Ball-skill tick counter, for the trace
        std::uint64_t ball_tick = 0;

        /// The joint targets last sent, by which policy and when
        std::array<double, JOINT_COUNT> last_target{};
        TargetSource last_target_source = TargetSource::NONE;
        NUClear::clock::time_point last_target_time{};

        /// ONNX Runtime session for the policy, nullptr until a model loads
        std::unique_ptr<utility::onnx::ONNXRuntime> onnx_rt{};
        bool model_loaded = false;

        /// Previous raw policy output, in policy order
        std::vector<float> last_action{};
        /// Observation window, oldest frame first
        std::deque<std::vector<float>> history{};

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
