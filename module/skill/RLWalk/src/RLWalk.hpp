#ifndef MODULE_SKILL_RLWALK_HPP
#define MODULE_SKILL_RLWALK_HPP

#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <mutex>
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
            /// @brief Alpha value for the action smoothing filter
            float action_alpha;
            /// @brief Servo torque value used for policy-generated commands
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
        } cfg;

        /// @brief OpenVINO model and inference request
        enum class ModelState { UNINITIALISED, READY, FAILED };
        mutable std::mutex model_mutex{};
        ov::Core core{};

        ov::CompiledModel compiled_model{};
        ov::InferRequest infer_request{};
        ModelState model_state = ModelState::UNINITIALISED;

        std::chrono::steady_clock::time_point next_retry{};
        std::chrono::milliseconds retry_backoff{1000};

        bool initialise_model_locked();
        void invalidate_model_locked();

        /// @brief Current phase of the walk (0-1)
        double phase;

        /// @brief Whether the model is initialized
        bool model_initialized;

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 50;

        /// @brief Last action taken by the model
        JointVector last_action;

        /// @brief Flag used by the action smoothing filter
        bool have_last_action = false;

        /// @brief Default pose for the robot
        JointVector default_pose;

        /// @brief Last joint positions for velocity estimation
        JointVector previous_pose;
        bool have_previous_pose = false;
        NUClear::clock::time_point last_update_time;


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
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_RLWALK_HPP
