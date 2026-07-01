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

    /// @brief Extra policy output beyond the joint targets: the per-step gait phase delta emitted by
    /// the clock_learned policy. It is appended after the 20 joint targets, so it is the last output.
    static constexpr int PHASE_DELTA_SIZE = 1;
    /// @brief Total policy output width: the 20 joint targets plus the phase delta.
    static constexpr int TOTAL_ACTION_SIZE = JOINT_POS_SIZE + PHASE_DELTA_SIZE;  // 21

    /// @brief Total observation width. The `actions` (last-action) term feeds back the *full* policy
    /// output — the joint targets plus the phase delta — so it is TOTAL_ACTION_SIZE wide, not
    /// JOINT_POS_SIZE. Layout: gyro, gravity, joint pos, joint vel, last action, command, phase.
    static constexpr int TOTAL_OBS_SIZE = GYRO_SIZE + GRAVITY_SIZE + JOINT_POS_SIZE + JOINT_POS_SIZE
                                          + TOTAL_ACTION_SIZE + COMMAND_SIZE + PHASE_SIZE;  // 72

    /// @brief Fixed-size observation vector type
    using ObservationVector = Eigen::Matrix<double, TOTAL_OBS_SIZE, 1>;
    /// @brief Fixed-size joint vector type
    using JointVector = Eigen::Matrix<double, JOINT_POS_SIZE, 1>;
    /// @brief Fixed-size command vector type
    using CommandVector = Eigen::Matrix<double, COMMAND_SIZE, 1>;

    /// @brief Raw policy output: the joint targets plus the gait phase delta.
    struct InferenceOutput {
        /// @brief Raw per-joint action outputs (mjlab order, no scaling/offset applied).
        JointVector joint_actions;
        /// @brief Raw per-step gait phase delta output (clock_learned variant). A value of 1
        /// advances the phase one nominal step (STEP_DT / gait_period).
        double phase_delta;
    };

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
            /// @brief Command velocity magnitude below which the policy joint offsets are zeroed,
            /// holding the default pose. Hack to avoid unwanted policy behaviour at ~zero command.
            double command_velocity_threshold;
            /// @brief Command magnitude (|linear| + |angular|) below which the policy-owned gait phase
            /// is frozen at zero and the phase observation collapses to the origin. Mirrors the
            /// command gate on PhaseDeltaAction/gait_clock in mjlab training (default 0.05).
            double phase_command_threshold;
        } cfg;

        /// @brief OpenVINO compiled model and inference request
        ov::CompiledModel compiled_model{};
        ov::InferRequest infer_request{};

        /// @brief Policy-owned gait phase in [0, 1). Accumulated from the per-step phase delta the
        /// policy emits as an extra action output (clock_learned variant), rather than from wall-clock
        /// time. Reset to zero on walk start and whenever the command drops below the phase gate.
        double policy_phase = 0.0;

        /// @brief Whether the model is initialized
        bool model_initialized;

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 50;

        /// @brief Fixed control timestep (seconds) corresponding to UPDATE_FREQUENCY
        static constexpr double STEP_DT = 1.0 / UPDATE_FREQUENCY;

        /// @brief Gap (seconds) since the previous timing tick beyond which the control loop is treated
        /// as having paused (e.g. the walk was pre-empted by a get-up). On resume the timing diagnostics
        /// re-baseline instead of reporting a spurious low frequency / huge drift across the gap.
        static constexpr double MAX_TICK_GAP = STEP_DT * 5.0;  // 0.1s, i.e. ~5 missed control steps

        /// @brief Number of control steps executed since the walk started. Used to advance the
        /// gait phase deterministically as control_step * STEP_DT rather than from wall-clock time.
        uint64_t control_step = 0;

        /// @brief Previous smoothed/clipped joint action, kept only as the recurrence state for the EMA
        /// smoothing filter and rate limiter that shape the servo command. This is NOT what is fed back
        /// in the observation (see last_action_raw).
        JointVector last_action;

        /// @brief Previous raw joint action straight from the policy (mjlab order, no scaling, offset,
        /// rate-clip, or smoothing). This is fed back as the joint part of the last-action observation
        /// so it matches training, where the `actions` observation echoes env.action_manager.action —
        /// the unprocessed policy output — rather than the post-processed servo command.
        JointVector last_action_raw;

        /// @brief Last raw gait phase delta emitted by the policy. Fed back as the final element of the
        /// last-action observation, matching the phase_delta action term appended in training. Like the
        /// raw joint actions it is echoed unprocessed, mirroring the raw action stored by the action manager.
        double last_phase_delta = 0.0;

        /// @brief Flag used by the action smoothing filter
        bool have_last_action = false;

        /// @brief Default pose for the robot
        JointVector default_pose;

        /// @brief Lower per-servo position limits used to clip the final commanded servo positions as a safety measure
        JointVector servo_limit_min;
        /// @brief Upper per-servo position limits used to clip the final commanded servo positions as a safety measure
        JointVector servo_limit_max;

        /// @brief Last joint positions for velocity estimation
        JointVector previous_pose;
        bool have_previous_pose = false;
        NUClear::clock::time_point last_update_time;

        // Control-loop timing diagnostics params
        /// @brief Whether a previous tick has been sampled (false until the first stable tick).
        bool have_timing_sample = false;
        /// @brief Timestamps of the previous tick on each clock, used to compute the per-tick period.
        NUClear::clock::time_point last_tick_nuclear{};
        std::chrono::steady_clock::time_point last_tick_steady{};
        /// @brief Timestamps at the first stable tick, used to compute cumulative gait-clock drift.
        NUClear::clock::time_point walk_start_nuclear{};
        std::chrono::steady_clock::time_point walk_start_steady{};
        /// @brief Value of control_step when the current timing baseline was established. Gait-clock drift
        /// is measured from here so it stays meaningful after a re-baseline following a pause.
        uint64_t timing_control_step_start = 0;
        /// @brief Time of the last rolling-summary log, used to throttle the summary to timing_report_period.
        NUClear::clock::time_point last_timing_report{};
        /// @brief Running statistics on the NUClear-clock per-tick period (seconds), since the walk started.
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
        /// @return The model's raw output: joint targets plus the gait phase delta
        InferenceOutput run_inference(const ObservationVector& observation);

        /// @brief Function to measure the actual control-loop frequency according to the nuclear clock and steady wall
        /// clock
        void debug_loop_timing();

        /// @brief Reset all control-loop timing diagnostic state. Called when a walk starts.
        void reset_loop_timing();
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_RLWALK_HPP
