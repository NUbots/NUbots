#ifndef MODULE_SKILL_K1KICKPOLICY_HPP
#define MODULE_SKILL_K1KICKPOLICY_HPP

#include <array>
#include <memory>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <string>

#include "extension/Behaviour.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill {

    /// Runs the mujoco_playground K1Kick policy (v5 side-sweep, 80-obs / 22-action ONNX) at
    /// 50 Hz and streams joint targets to the platform as BoosterLowCmd, the same low-level
    /// path skill::K1WalkPolicy uses. Like the walk policy (and unlike K1GetUpPolicy),
    /// actions are offsets on the *default* pose, not the current configuration. The policy
    /// owns all 22 joints including the head.
    ///
    /// v5 obs layout (80): gyro(3), projected gravity(3), commanded kick direction in the
    /// torso frame (2), ball xy in the torso frame (2), gait phase [cos_L,cos_R,sin_L,sin_R]
    /// (4), q - default_pose(22), dq(22), last_action(22). The gait phase is a self-run clock
    /// (same convention as K1WalkPolicy) so the walk<->kick handoff shares a phase. The kick
    /// direction is a lateral (±robot-left) sweep vector: the policy sweeps the ball sideways
    /// with the inside foot. The incoming Kick task's target/direction/leg fields are ignored
    /// (the task acts as a trigger); the sweep side is chosen from the ball's lateral offset.
    class K1KickPolicy : public ::extension::behaviour::BehaviourReactor {
    public:
        static constexpr std::size_t JOINT_COUNT = 22;  // SDK JointIndexK1 serial order
        static constexpr std::size_t OBS_DIM = 80;  // gyro+grav+kick_dir+ball+phase+q+dq+last_action

        explicit K1KickPolicy(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            std::string model_path;
            /// Inference device: "gpu" (ONNX Runtime's TensorRT execution provider, falling back to
            /// the CPU when it cannot be set up, e.g. no CUDA device) or "cpu"
            std::string device = "cpu";
            /// @brief how long to run the policy before the Kick Task reports Done (s)
            double kick_duration = 4.0;
            /// @brief ignore ball estimates below this localisation confidence
            double ball_confidence_threshold = 0.1;
            /// @brief treat the ball obs as absent if the last estimate is older than this (s)
            double ball_stale_timeout = 1.0;
            /// @brief gait-phase advance rate (Hz), shared with the walk policy for a
            /// phase-continuous walk<->kick handoff
            double gait_frequency = 1.5;
            /// @brief walk->kick handoff cross-fade duration (s): the commanded pose is
            /// blended from the robot's current stance into the policy output over this
            /// window so the kick does not snap the mid-walk robot to the default crouch
            /// (training always starts from the default pose; deployment does not)
            double handoff_blend = 0.3;
            /// @brief default commanded lateral sweep direction in the torso frame (unit xy),
            /// used when no ball is visible to pick a side
            std::array<double, 2> kick_direction{0.0, 1.0};
            /// @brief PD gains + per-joint action scale + home pose (training-time values,
            /// shared with the walk policy since both train against the same model)
            std::array<double, JOINT_COUNT> kp{};
            std::array<double, JOINT_COUNT> kd{};
            std::array<double, JOINT_COUNT> action_scale_joint{};
            std::array<double, JOINT_COUNT> default_pose{};
        } cfg;

        /// ONNX Runtime session for the policy, nullptr until a model loads
        std::unique_ptr<utility::onnx::ONNXRuntime> onnx_rt{};
        bool model_loaded = false;

        std::array<float, JOINT_COUNT> last_action{};

        /// Gait-phase clock [phase_L, phase_R], advanced each control step (init {0, pi}).
        std::array<double, 2> phase{0.0, M_PI};
        /// Commanded lateral sweep direction in the torso frame (unit xy), fixed for the
        /// duration of a kick once a fresh ball fixes the side.
        std::array<double, 2> kick_dir_robot{0.0, 1.0};
        /// Whether the sweep side has been chosen for the current kick
        bool kick_dir_set = false;

        /// Latest ball estimate in world space {W}, refreshed from localisation::Ball
        Eigen::Vector3d ball_rBWw           = Eigen::Vector3d::Zero();
        bool ball_seen                      = false;
        double ball_confidence              = 0.0;
        NUClear::clock::time_point ball_time{};

        /// When the current Kick Task started (used to end after cfg.kick_duration)
        NUClear::clock::time_point kick_since{};
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_K1KICKPOLICY_HPP
