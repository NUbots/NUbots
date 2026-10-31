#ifndef MODULE_SKILL_SIMK1WALK_HPP
#define MODULE_SKILL_SIMK1WALK_HPP

#include <Eigen/Core>
#include <array>
#include <memory>
#include <nuclear>
#include <string>

#include "extension/Behaviour.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs the mujoco_playground K1 joystick walk policy (79-obs / 22-action ONNX, see
    /// NUSim docs/OBS_ACTION_CONTRACT.md) on the robot side and streams the resulting
    /// joint targets to the platform as BoosterLowCmd (rt/joint_ctrl, CUSTOM mode). This
    /// replaces skill::Walk's Move() RPC path: locomotion inference lives here, and the
    /// robot/simulator only has to track servo joint commands.
    ///
    /// The observation carries no base linear velocity. It used to (82 obs), sourced from a
    /// finite-difference of rt/odometer_state, but there is no measured linear velocity on
    /// the real K1 in CUSTOM mode -- so the policy is now trained with linvel as a
    /// critic-only privileged quantity and the deployment side has nothing to estimate.
    class SimK1Walk : public ::extension::behaviour::BehaviourReactor {
    public:
        static constexpr std::size_t JOINT_COUNT = 22;  // SDK JointIndexK1 serial order
        static constexpr std::size_t OBS_DIM     = 79;

        explicit SimK1Walk(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Inference device: "cpu", or "gpu" for ONNX Runtime's TensorRT EP (fp16). gpu falls
            /// back to cpu when it cannot load (no CUDA device, driver mismatch, ...).
            std::string device = "cpu";
            /// @brief Hz, gait phase advance rate (training randomizes U(1.25, 1.75))
            double gait_frequency = 1.5;
            /// @brief command norm below which the phase observation pins to [pi, pi]
            double stand_threshold = 0.01;
            /// @brief PD gains for the two head joints (the policy does not own the head)
            double head_kp = 10.0;
            double head_kd = 0.5;
            /// @brief Velocity command applied while performing an in-walk kick
            Eigen::Vector3d kick_velocity = Eigen::Vector3d::Zero();
            /// @brief How long to drive the kick velocity before reporting the kick as done
            NUClear::clock::duration kick_duration{};
            /// @brief Training-time actuation, JointIndexK1 order (see SimK1Walk.yaml)
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
            /// @brief Hard joint ranges of the trained MuJoCo model, JointIndexK1 order. The
            /// commanded position is clamped to these: MuJoCo's joint constraint silently
            /// absorbs an out-of-range target, but on the robot it is a leg torquing into a
            /// mechanical stop.
            std::array<double, JOINT_COUNT> joint_lower{};
            std::array<double, JOINT_COUNT> joint_upper{};
        } cfg;

        /// ONNX Runtime inference backend
        std::unique_ptr<utility::onnx::ONNXRuntime> policy{};
        bool model_loaded = false;

        /// Previous raw network output (part of the observation)
        std::array<float, JOINT_COUNT> last_action{};
        /// Per-foot gait phase, initialized anti-phase [0, pi]
        std::array<double, 2> phase{0.0, 3.141592653589793};

        /// Wall-clock of the previous policy tick, so the gait phase advances on the period
        /// that actually elapsed rather than on a hardcoded 0.02 s. Every<50, Per<seconds>>
        /// is best-effort on an Orin also running YOLO: at a true 40 Hz a nominal 1.5 Hz
        /// gait actually runs at 1.20 Hz, below the U(1.25, 1.75) training range.
        bool have_last_tick = false;
        NUClear::clock::time_point last_tick_time{};

        /// Monotonic tick counter and the robot's last reported motion mode, both logged with
        /// every observation: statistics over a log that mixes CUSTOM-mode walking with
        /// frozen non-CUSTOM ticks describe a policy shouting at a robot that isn't listening.
        std::uint64_t tick = 0;
        int last_mode      = -1;

        /// Latest clamped head target (yaw, pitch) from BoosterHeadRot
        Eigen::Vector2d head_target = Eigen::Vector2d::Zero();

        /// Time the current in-walk kick started
        NUClear::clock::time_point kick_start_time{};

        /// Last emitted walk state, to avoid re-emitting unchanged states at 50 Hz
        int last_walk_state = -1;

        void reset_policy_state();
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SIMK1WALK_HPP
