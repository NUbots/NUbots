#ifndef MODULE_SKILL_K1WALKPOLICY_HPP
#define MODULE_SKILL_K1WALKPOLICY_HPP

#include <array>
#include <Eigen/Core>
#include <nuclear>
#include <openvino/openvino.hpp>
#include <string>

#include "extension/Behaviour.hpp"

namespace module::skill {

    /// Runs the mujoco_playground K1 joystick walk policy (82-obs / 22-action ONNX, see
    /// NUSim docs/OBS_ACTION_CONTRACT.md) on the robot side and streams the resulting
    /// joint targets to the platform as BoosterLowCmd (rt/joint_ctrl, CUSTOM mode). This
    /// replaces skill::K1Walk's Move() RPC path: locomotion inference lives here, and the
    /// robot/simulator only has to track servo joint commands.
    class K1WalkPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        static constexpr std::size_t JOINT_COUNT = 22;  // SDK JointIndexK1 serial order
        static constexpr std::size_t OBS_DIM     = 82;

        explicit K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// @brief Hz, gait phase advance rate (training randomizes U(1.25, 1.75))
            double gait_frequency = 1.5;
            /// @brief command norm below which the phase observation pins to [pi, pi]
            double stand_threshold = 0.01;
            /// @brief low-pass factor for the odometry-differentiated linear velocity estimate
            double linvel_alpha = 0.3;
            /// @brief PD gains for the two head joints (the policy does not own the head)
            double head_kp = 10.0;
            double head_kd = 0.5;
            /// @brief Velocity command applied while performing an in-walk kick
            Eigen::Vector3d kick_velocity = Eigen::Vector3d::Zero();
            /// @brief How long to drive the kick velocity before reporting the kick as done
            NUClear::clock::duration kick_duration{};
            /// @brief Training-time actuation, JointIndexK1 order (see K1WalkPolicy.yaml)
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
        } cfg;

        /// OpenVINO inference plumbing (CPU device; the model is a small MLP)
        ov::Core core{};
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        bool model_loaded = false;

        /// Previous raw network output (part of the observation)
        std::array<float, JOINT_COUNT> last_action{};
        /// Per-foot gait phase, initialized anti-phase [0, pi]
        std::array<double, 2> phase{0.0, 3.141592653589793};

        /// Odometry-differentiated body-frame linear velocity estimate
        bool have_last_odom = false;
        Eigen::Vector3d last_odom = Eigen::Vector3d::Zero();  // x, y, theta (world)
        NUClear::clock::time_point last_odom_time{};
        Eigen::Vector2d linvel_body = Eigen::Vector2d::Zero();

        /// Latest clamped head target (yaw, pitch) from BoosterHeadRot
        Eigen::Vector2d head_target = Eigen::Vector2d::Zero();

        /// Time the current in-walk kick started
        NUClear::clock::time_point kick_start_time{};

        /// Last emitted walk state, to avoid re-emitting unchanged states at 50 Hz
        int last_walk_state = -1;

        void reset_policy_state();
        void update_linvel(const Eigen::Vector3d& odom_now);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1WALKPOLICY_HPP
