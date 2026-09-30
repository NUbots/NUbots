#ifndef MODULE_SKILL_K1WALKPOLICY_HPP
#define MODULE_SKILL_K1WALKPOLICY_HPP

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <nuclear>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "utility/vision/TensorRT.hpp"

namespace module::skill {

    /// Runs the mjlab K1 velocity-tracking walk policy at 50 Hz and streams the resulting joint
    /// targets to the platform as a Director-arbitrated K1Servos subtask (CUSTOM mode), the same
    /// low-level path as K1BlockPolicy / K1GetUpPolicy. This replaces skill::K1Walk's Move() RPC
    /// path: locomotion inference lives here, and the robot/simulator only tracks joint commands.
    ///
    /// The policy is an observation-history model (mjlab.rl.obs_history): the ONNX takes a flat
    /// window of the last `history_window` observation frames, time-major and oldest first, and a
    /// TCN inside the graph encodes it. One frame is byte-for-byte the actor observation vector, so
    /// this module keeps a single ring buffer of the frame it already builds. Both observation
    /// normalizers are baked into the exported graph -- nothing here normalizes.
    ///
    /// The observation leads with the base linear velocity, in the body frame, which comes from the
    /// Booster controller's odometry twist (rt/odom, BoosterOdometryTwist). See README.md for the
    /// full contract.
    class K1WalkPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        /// All K1 joints, in the Booster SDK JointIndexK1 serial order
        static constexpr std::size_t JOINT_COUNT = 22;
        /// Head joints in JointIndexK1 order; never policy controlled (vision owns the head)
        static constexpr std::size_t HEAD_YAW   = 0;
        static constexpr std::size_t HEAD_PITCH = 1;
        /// Velocity command length: [vx, vy, wz]
        static constexpr std::size_t COMMAND_DIM = 3;

        explicit K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Run inference with TensorRT (GPU), falling back to OpenVINO CPU when false or when
            /// the engine cannot be built
            bool use_tensorrt = false;
            /// Number of frames in the observation window fed to the ONNX (1 = no history). The
            /// input is time-major, oldest frame first: [1, history_window * frame_dim]
            std::size_t history_window = 1;
            /// JointIndexK1 indices of the policy-controlled joints, in policy order
            std::vector<std::size_t> policy_joints{};
            /// Oldest odometry twist (s) observed as the base linear velocity. Older, or none at
            /// all, is observed as zero and warned about.
            double linear_velocity_max_age = 0.1;
            /// Commanded pose is blended from the current pose into the policy target over this
            /// window (s), since deployment enters CUSTOM from wherever the previous mode left the
            /// robot while training always starts at the default pose
            double handoff_blend = 0.3;
            /// Head tracking gains (the head follows the latest BoosterHeadRot)
            double head_kp = 10.0;
            double head_kd = 0.5;
            /// @brief Velocity command applied while performing an in-walk kick
            Eigen::Vector3d kick_velocity = Eigen::Vector3d::Zero();
            /// @brief How long to drive the kick velocity before reporting the kick as done
            NUClear::clock::duration kick_duration{};
            /// Training-time actuation, all in JointIndexK1 order (see K1WalkPolicy.yaml)
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
            /// @brief Hard joint ranges the commanded position is clamped to, JointIndexK1 order.
            /// MuJoCo's joint constraint silently absorbs an out-of-range target, but on the robot
            /// it is a leg torquing into a mechanical stop.
            std::array<double, JOINT_COUNT> joint_lower{};
            std::array<double, JOINT_COUNT> joint_upper{};
        } cfg;

        /// Length of one observation frame: gyro(3) + gravity(3)
        /// + 3 * n_policy_joints + command(3)
        [[nodiscard]] std::size_t frame_dim() const {
            return 6 + 3 * cfg.policy_joints.size() + COMMAND_DIM;
        }

        /// Load the ONNX and check its input/output sizes against the configured contract
        void load_model();

        /// Run the network on a flat observation window, returning n_policy_joints actions
        std::vector<float> infer(const std::vector<float>& input);

        /// Drop the history window and the action feedback. Called whenever the
        /// next tick cannot continue the previous one: a new walk task, or a resumption after the
        /// get-up policy owned the low-level channel.
        void reset_policy_state();

        /// TensorRT engine, nullptr when running on the OpenVINO fallback
        std::unique_ptr<utility::vision::TensorRT> trt{};
        ov::Core core{};
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        bool model_loaded = false;

        /// Previous raw policy output (policy order), fed back as the last-action observation
        std::vector<float> last_action{};
        /// Observation window, oldest frame first
        std::deque<std::vector<float>> history{};

        /// Latest base linear velocity from the controller's odometry twist (body frame, m/s) and
        /// when it arrived, guarded against the tick loop reading it mid-write
        std::mutex linear_velocity_mutex{};
        Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
        NUClear::clock::time_point linear_velocity_time{};
        bool have_linear_velocity = false;
        /// When a missing or stale velocity was last warned about
        NUClear::clock::time_point last_linear_velocity_warning{};

        /// Wall-clock of the previous policy tick, to measure the loop period against the trained
        /// 0.02 s.
        bool have_last_tick = false;
        NUClear::clock::time_point last_tick_time{};

        /// Ticks since the last loop-rate summary whose period missed the trained 50 Hz, and the
        /// tick/time the current summary window started at
        std::uint64_t off_rate_ticks     = 0;
        std::uint64_t timing_report_tick = 0;
        NUClear::clock::time_point last_timing_report{};

        /// When the current walk task entered CUSTOM (drives the hand-off blend)
        NUClear::clock::time_point walk_since{};

        /// Monotonic tick counter and the robot's last reported motion mode, both logged with every
        /// observation: statistics over a log that mixes CUSTOM-mode walking with frozen
        /// non-CUSTOM ticks describe a policy shouting at a robot that isn't listening.
        std::uint64_t tick = 0;
        int last_mode      = -1;

        /// When CUSTOM was last requested from the tick loop, so a robot that will not enter the
        /// mode is not sent a mode-change RPC every 20 ms
        NUClear::clock::time_point last_mode_request{};

        /// Latest clamped head target (yaw, pitch) from BoosterHeadRot
        Eigen::Vector2d head_target = Eigen::Vector2d::Zero();

        /// Time the current in-walk kick started
        NUClear::clock::time_point kick_start_time{};

        /// Last emitted walk state and command, to avoid re-emitting an unchanged WalkState at 50 Hz
        int last_walk_state               = -1;
        Eigen::Vector3d last_walk_command = Eigen::Vector3d::Zero();
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1WALKPOLICY_HPP
