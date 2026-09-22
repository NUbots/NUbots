#ifndef MODULE_SKILL_RLWALK_HPP
#define MODULE_SKILL_RLWALK_HPP

#include <Eigen/Core>
#include <chrono>
#include <nuclear>
#include <openvino/openvino.hpp>

#include "extension/Behaviour.hpp"

namespace module::skill {

    /// @brief Observation vector sizes
    static constexpr int ACC_SIZE       = 3;
    static constexpr int GYRO_SIZE      = 3;
    static constexpr int GRAVITY_SIZE   = 3;
    static constexpr int JOINT_POS_SIZE = 20;
    static constexpr int COMMAND_SIZE   = 3;
    static constexpr int PHASE_SIZE     = 2;
    static constexpr int TOTAL_OBS_SIZE =
        GYRO_SIZE + GRAVITY_SIZE + JOINT_POS_SIZE + JOINT_POS_SIZE + JOINT_POS_SIZE + COMMAND_SIZE + PHASE_SIZE;  // 71

    /// @brief Fixed-size observation vector type
    using ObservationVector = Eigen::Matrix<double, TOTAL_OBS_SIZE, 1>;
    /// @brief Fixed-size joint vector type
    using JointVector = Eigen::Matrix<double, JOINT_POS_SIZE, 1>;
    /// @brief Fixed-size command vector type
    using CommandVector = Eigen::Matrix<double, COMMAND_SIZE, 1>;

    class RLWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the ONNX model file
            std::string model_path;
            /// @brief Path to the PyTorch model file for normalisation
            std::string pt_model_path;
            /// @brief Device to run inference on (CPU, GPU, etc.)
            std::string device;
            /// @brief Input tensor name in the ONNX model
            std::string input_name;
            /// @brief Output tensor name in the ONNX model
            std::string output_name;
            /// @brief Number of joints in the model output
            int num_joints;
            /// @brief Size of the observation vector
            int obs_size;
            /// @brief Servo torque value to send to nusense
            float servo_torque;
            /// @brief Servo proportional gain for leg and hip joints
            float leg_servo_gain;
            /// @brief Servo proportional gain for head joints
            float head_servo_gain;
            /// @brief Servo proportional gain for arm joints
            float arm_servo_gain;
            /// @brief Scale factor to convert inference outputs to joint angles
            double nugus_action_scale;
            /// @brief Gait period used in the phase calculation
            double gait_period;
            /// @brief Command speed below which the default pose is held instead of the policy's offsets
            double command_velocity_threshold;
        } cfg;

        /// @brief OpenVINO compiled model and inference request
        ov::CompiledModel compiled_model{};
        ov::InferRequest infer_request{};

        /// @brief Current phase of the walk (0-1)
        double phase;

        /// @brief Whether the model is initialized
        bool model_initialized;

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 50;

        /// @brief Fixed control timestep (seconds) corresponding to UPDATE_FREQUENCY
        static constexpr double STEP_DT = 1.0 / UPDATE_FREQUENCY;

        /// @brief Tick gap (s) treated as a pause, after which the timing diagnostics re-baseline
        static constexpr double MAX_TICK_GAP = STEP_DT * 5.0;

        /// @brief Control steps since the walk started; drives the gait phase
        uint64_t control_step = 0;

        /// @brief Last action taken by the model
        JointVector last_action;

        /// @brief Default pose for the robot
        JointVector default_pose;

        /// @brief Per-servo position limits (rad, NUbots order) the commands are clipped to
        JointVector servo_limit_min;
        JointVector servo_limit_max;

        /// @brief Last joint positions for inference
        JointVector previous_pose;

        // Control-loop timing diagnostics
        /// @brief Whether a baseline tick has been sampled
        bool have_timing_sample = false;
        /// @brief Previous tick on each clock
        NUClear::clock::time_point last_tick_nuclear{};
        std::chrono::steady_clock::time_point last_tick_steady{};
        /// @brief Baseline tick on each clock, for the gait-clock drift
        NUClear::clock::time_point walk_start_nuclear{};
        std::chrono::steady_clock::time_point walk_start_steady{};
        /// @brief control_step at the baseline tick
        uint64_t timing_control_step_start = 0;
        /// @brief When the rolling summary was last logged
        NUClear::clock::time_point last_timing_report{};
        /// @brief Running statistics on the NUClear-clock tick period (s) since the baseline
        uint64_t timing_samples     = 0;
        double timing_period_sum    = 0.0;
        double timing_period_sq_sum = 0.0;
        double timing_period_min    = 0.0;
        double timing_period_max    = 0.0;

    public:
        /// @brief Called by the powerplant to build and setup the RLWalk reactor.
        explicit RLWalk(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Initialize the OpenVINO model
        void initialize_model();

        /// @brief Run inference with the current observation
        /// @param observation The current observation vector
        /// @return The model's output (joint angles)
        JointVector run_inference(const ObservationVector& observation);

        /// @brief Graph and log the control-loop period and gait-clock drift against the NUClear and steady clocks
        void debug_loop_timing();

        /// @brief Reset the timing diagnostics, when a walk starts
        void reset_loop_timing();
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_RLWALK_HPP
