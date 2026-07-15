#ifndef MODULE_SKILL_K1GETUPPOLICY_HPP
#define MODULE_SKILL_K1GETUPPOLICY_HPP

#include <array>
#include <Eigen/Core>
#include <nuclear>
#include <openvino/openvino.hpp>
#include <string>

#include "extension/Behaviour.hpp"

namespace module::skill {

    /// Runs the mujoco_playground K1Getup fall-recovery policy (72-obs / 22-action ONNX)
    /// at 50 Hz and streams joint targets to the platform as BoosterLowCmd, the same
    /// low-level path skill::K1WalkPolicy uses. Replaces skill::K1GetUp's GetUpWithMode()
    /// RPC. Unlike the walk policy, actions are offsets on the *current* joint
    /// configuration (the Go1Getup convention the task was trained with), and the policy
    /// owns the head.
    class K1GetUpPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        static constexpr std::size_t JOINT_COUNT = 22;  // SDK JointIndexK1 serial order
        static constexpr std::size_t OBS_DIM     = 72;  // gyro+gravity+q+dq+last_action

        explicit K1GetUpPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// @brief |roll| and |pitch| below this count as upright (rad)
            double upright_angle = 0.35;
            /// @brief how long the robot must stay upright before the get-up is Done (s)
            double upright_time = 1.0;
            /// @brief both knees must be flexed less than this to count as standing (rad)
            double knee_extended_angle = 0.9;
            /// @brief PD gains + per-joint action scale (training-time values, shared
            /// with the walk policy since both train against the same model)
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
        } cfg;

        ov::Core core{};
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        bool model_loaded = false;

        std::array<float, JOINT_COUNT> last_action{};

        /// Time we first observed the robot upright (reset when it tips again)
        NUClear::clock::time_point upright_since{};
        bool was_upright = false;
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1GETUPPOLICY_HPP
