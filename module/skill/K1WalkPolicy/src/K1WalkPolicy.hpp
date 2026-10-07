#ifndef MODULE_SKILL_K1WALKPOLICY_HPP
#define MODULE_SKILL_K1WALKPOLICY_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <Eigen/Core>
#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"
#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs an mjlab K1 velocity-tracking walk policy at 50 Hz and streams the resulting joint
    /// targets to the platform as a Director-arbitrated K1Servos subtask (CUSTOM mode), the same
    /// low-level path as K1BlockPolicy / K1GetUpPolicy. This replaces skill::K1Walk's Move() RPC
    /// path: locomotion inference lives here, and the robot/simulator only tracks joint commands.
    ///
    /// The observation contract is configured rather than hardcoded, because the two checkpoints
    /// that have flown on this robot do not share one. A frame is
    ///
    ///     gyro(3) | projected gravity(3) | q - default(N) | dq(N) | last action(N) | command(3)
    ///     [ | gait clock(2), when gait_clock is set ]
    ///
    /// over the `policy_joints` joints, and the ONNX input is the last `history_window` frames
    /// flattened time-major, oldest first (history_window 1 = no history, just the frame). The
    /// currently shipped policy is single-frame with no gait clock and all 22 joints; the previous
    /// one was a 25-frame observation-history model (mjlab.rl.obs_history) with a gait clock over
    /// 20 joints. Every observation normalizer is baked into the exported graph -- nothing here
    /// normalizes -- and load_model() refuses any graph whose I/O does not match the config.
    ///
    /// The observation carries no base linear velocity: there is no measured base linear velocity
    /// on the real K1 in CUSTOM mode, so it is a critic-only privileged quantity in training and
    /// the deployment side has nothing to estimate. See README.md for the full contract.
    class K1WalkPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        /// All K1 joints, in the Booster SDK JointIndexK1 serial order
        static constexpr std::size_t JOINT_COUNT = 22;
        /// Head joints in JointIndexK1 order. Whether the policy drives them or whether they
        /// track BoosterHeadRot is head.policy_controlled; see the config.
        static constexpr std::size_t HEAD_YAW   = 0;
        static constexpr std::size_t HEAD_PITCH = 1;
        /// Velocity command length: [vx, vy, wz]
        static constexpr std::size_t COMMAND_DIM = 3;
        /// Gait clock length: [sin(2*pi*phase), cos(2*pi*phase)]
        static constexpr std::size_t CLOCK_DIM = 2;

        explicit K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Inference device: "gpu" (ONNX Runtime's TensorRT execution provider, falling back to
            /// the CPU when it cannot be set up) or "cpu"
            std::string device = "cpu";
            /// Number of frames in the observation window fed to the ONNX (1 = no history). The
            /// input is time-major, oldest frame first: [1, history_window * frame_dim]
            std::size_t history_window = 1;
            /// JointIndexK1 indices of the joints the policy observes and acts on, in policy order
            std::vector<std::size_t> policy_joints{};
            /// Whether the observation frame carries the [sin, cos] gait clock. Only true for
            /// policies trained against an explicit clock; the current one has none.
            bool gait_clock = false;
            /// Full gait-cycle duration (s) of the observed clock; must match the training
            /// GAIT_PERIOD, since the same clock drove the swing-height and contact rewards.
            /// Unused when gait_clock is false.
            double gait_period = 0.6;
            /// Command magnitude (|v_xy| + |wz|) at or below which the observed clock collapses to
            /// (0, 0) -- the distinct "standing" input the policy was trained to see
            double command_threshold = 0.05;
            /// Commanded pose is blended from the current pose into the policy target over this
            /// window (s), since deployment enters CUSTOM from wherever the previous mode left the
            /// robot while training always starts at the default pose
            double handoff_blend = 0.3;
            /// Whether the policy's own head actions drive the head. When false the head is
            /// overridden with the latest BoosterHeadRot and the gains below, so vision owns it;
            /// the policy still observes the measured head state and its own head action.
            bool head_policy_controlled = false;
            /// Head tracking gains used when the head follows BoosterHeadRot rather than the policy
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

        /// Length of one observation frame: gyro(3) + gravity(3) + 3 * n_policy_joints
        /// + command(3) [+ gait clock(2)]
        [[nodiscard]] std::size_t frame_dim() const {
            return 6 + 3 * cfg.policy_joints.size() + COMMAND_DIM + (cfg.gait_clock ? CLOCK_DIM : 0);
        }

        /// Load the ONNX and check its input/output sizes against the configured contract
        void load_model();

        /// Run the network on a flat observation window, returning n_policy_joints actions
        std::vector<float> infer(const std::vector<float>& input);

        /// Drop the history window, the action feedback and the gait phase. Called whenever the
        /// next tick cannot continue the previous one: a new walk task, or a resumption after the
        /// get-up policy owned the low-level channel.
        void reset_policy_state();

        /// ONNX Runtime session for the policy, nullptr until a model loads
        std::unique_ptr<utility::onnx::ONNXRuntime> onnx_rt{};
        bool model_loaded = false;

        /// Previous raw policy output (policy order), fed back as the last-action observation
        std::vector<float> last_action{};
        /// Observation window, oldest frame first
        std::deque<std::vector<float>> history{};

        /// Gait clock phase in [0, 1), unused when gait_clock is false. Training indexes it on
        /// episode time at a fixed 50 Hz; here it advances on the measured loop period so the gait
        /// keeps its trained wall-clock rate when the loop runs slow.
        double gait_phase = 0.0;

        /// Wall-clock of the previous policy tick, so the gait phase advances on the period that
        /// actually elapsed rather than on a hardcoded 0.02 s.
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
