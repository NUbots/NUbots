#include "RLWalk.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/actuation/Servos.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using extension::behaviour::RunReason;
    using message::actuation::Limbs;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using utility::support::Expression;
    using WalkTask = message::skill::Walk;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    std::vector<std::pair<int, ServoID>> joint_map = {
        {0, ServoID::R_SHOULDER_PITCH}, {1, ServoID::L_SHOULDER_PITCH}, {2, ServoID::R_SHOULDER_ROLL},
        {3, ServoID::L_SHOULDER_ROLL},  {4, ServoID::R_ELBOW},          {5, ServoID::L_ELBOW},
        {6, ServoID::R_HIP_YAW},        {7, ServoID::L_HIP_YAW},        {8, ServoID::R_HIP_ROLL},
        {9, ServoID::L_HIP_ROLL},       {10, ServoID::R_HIP_PITCH},     {11, ServoID::L_HIP_PITCH},
        {12, ServoID::R_KNEE},          {13, ServoID::L_KNEE},          {14, ServoID::R_ANKLE_PITCH},
        {15, ServoID::L_ANKLE_PITCH},   {16, ServoID::R_ANKLE_ROLL},    {17, ServoID::L_ANKLE_ROLL},
        {18, ServoID::NECK_YAW},        {19, ServoID::HEAD_PITCH}};

    struct JointStateVectors {
        Eigen::Matrix<double, 20, 1> position;
        Eigen::Matrix<double, 20, 1> velocity;
    };

    inline JointStateVectors sensors_to_joint_state(const std::unique_ptr<Sensors>& sensors) {
        JointStateVectors state;
        for (const auto& [index, servo_id] : joint_map) {
            state.position(index, 0) = sensors->servo.at(servo_id).present_position;
            state.velocity(index, 0) = sensors->servo.at(servo_id).present_velocity;
        }
        return state;
    }

    // Comes from the XML on the mjlab training side.
    std::vector<int> mj_to_nubots_order = {7, 9, 11, 13, 15, 17, 6, 8, 10, 12, 14, 16, 18, 19, 1, 3, 5, 0, 2, 4};
    inline Eigen::Matrix<double, 20, 1> mjlab_to_nubots(const Eigen::Matrix<double, 20, 1>& mjlab_joint_offsets) {
        Eigen::Matrix<double, 20, 1> nubots_joint_offsets = Eigen::Matrix<double, 20, 1>::Zero();
        // Convert from MJlabs order to NUbots order
        for (int i = 0; i < mjlab_joint_offsets.size(); ++i) {
            nubots_joint_offsets(mj_to_nubots_order[i]) = mjlab_joint_offsets(i);
        }
        return nubots_joint_offsets;
    }

    // The inverse mapping of the above
    std::vector<int> nubots_to_mj_order = {17, 14, 18, 15, 19, 16, 6, 0, 7, 1, 8, 2, 9, 3, 10, 4, 11, 5, 12, 13};
    inline Eigen::Matrix<double, 20, 1> nubots_to_mjlab(const Eigen::Matrix<double, 20, 1>& nubots_joints) {
        Eigen::Matrix<double, 20, 1> mjlab_joints = Eigen::Matrix<double, 20, 1>::Zero();
        // Convert from NUbots order to Mjlab's expected order
        for (int i = 0; i < nubots_joints.size(); ++i) {
            mjlab_joints(nubots_to_mj_order[i]) = nubots_joints(i);
        }
        return mjlab_joints;
    }

    RLWalk::RLWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("RLWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RLWalk.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Load model configuration
            cfg.model_path         = config["model"]["path"].as<std::string>();
            cfg.device             = config["model"]["device"].as<std::string>();
            cfg.input_name         = config["model"]["input_name"].as<std::string>();
            cfg.output_name        = config["model"]["output_name"].as<std::string>();
            cfg.num_joints         = config["model"]["num_joints"].as<int>();
            cfg.obs_size           = config["model"]["obs_size"].as<int>();
            cfg.action_alpha       = config["model"]["action_alpha"].as<float>();
            cfg.servo_torque       = config["servos"]["torque"].as<float>();
            cfg.head_servo_gain    = config["servos"]["head_gains"].as<float>();
            cfg.leg_servo_gain     = config["servos"]["leg_gains"].as<float>();
            cfg.arm_servo_gain     = config["servos"]["arm_gains"].as<float>();
            cfg.nugus_action_scale = config["model"]["action_scale"].as<double>();  // Defined in mjlab training.
            cfg.gait_period        = config["model"]["gait_period"].as<double>();   // Defined in mjlab training.

            // Command velocity magnitude below which the policy joint offsets are zeroed so the
            // robot holds the default pose. Hack to avoid unwanted policy behaviour at ~zero command.
            cfg.command_velocity_threshold = config["command_velocity_threshold"].as<double>();

            // Command magnitude below which the policy-owned gait phase is gated off (frozen at zero).
            cfg.phase_command_threshold = config["phase_command_threshold"].as<double>();

            // Initialize vectors
            last_action      = JointVector::Zero();
            last_action_raw  = JointVector::Zero();
            have_last_action = false;

            default_pose = JointVector(config["default_pose"].as<Expression>());

            // Per-servo position limits (radians, NUbots joint order) used to clip the final
            // commanded servo positions as a safety measure against physically infeasible commands.
            servo_limit_min    = JointVector(config["servo_limits"]["min"].as<Expression>());
            servo_limit_max    = JointVector(config["servo_limits"]["max"].as<Expression>());
            previous_pose      = default_pose;
            have_previous_pose = false;
            last_update_time   = NUClear::clock::now();


            // Walk-related behaviours rely on an initial stability message
            emit(std::make_unique<Stability>(Stability::UNKNOWN));

            // Compile the model and create inference request object
            try {
                log<INFO>("Loading RLWalk model from: ", cfg.model_path);
                log<INFO>("Using device: ", cfg.device);

                ov::Core core{};

                // Try to fallback to CPU if GPU fails
                try {
                    compiled_model = core.compile_model(cfg.model_path, cfg.device);
                }
                catch (const std::exception& e) {
                    if (cfg.device == "GPU") {
                        log<WARN>("Failed to compile model on GPU, falling back to CPU: ", e.what());
                        compiled_model = core.compile_model(cfg.model_path, "CPU");
                    }
                    else {
                        log<WARN>("Failed to compile RLWalk model: ", e.what());
                        throw;
                    }
                }

                infer_request = compiled_model.create_infer_request();
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log<ERROR>("Failed to load RLWalk model: ", e.what());
                throw;
            }
        });

        // Start - Runs every time the Walk provider starts
        on<Start<WalkTask>>().then([this]() {
            // Reset the control step counter so the gait phase starts from zero on each walk start
            control_step = 0;
            // Reset the policy-owned gait phase and the last-action feedback (raw joints + phase delta)
            // so each walk starts from a clean, zeroed history, matching the action-history reset mjlab
            // performs on episode reset.
            policy_phase     = 0.0;
            last_phase_delta = 0.0;
            last_action_raw  = JointVector::Zero();
            // If debugging, reset the loop-timing diagnostics so each walk is measured from a clean baseline
            if (log_level <= DEBUG) {
                reset_loop_timing();
            }
            // Emit a stopped state as we are not yet walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Stop - Runs every time the Walk task is removed
        on<Stop<WalkTask>>().then([this] {
            // Emit a stopped state as we are now not walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Main loop - Updates the walk engine at fixed frequency
        on<Provide<WalkTask>,
           With<Sensors>,
           With<Stability>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single,
           Priority::HIGH>()
            .then([this](const WalkTask& walk_task,
                         const RunReason& run_reason,
                         const Sensors& sensors,
                         const Stability& stability) {
                // Keep policy updates at deterministic frequency (50 Hz)
                if (run_reason != RunReason::OTHER_TRIGGER) {
                    return;
                }

                // Only run if we're in a stable state
                if (stability >= Stability::DYNAMIC) {
                    // Debug the actual loop frequency
                    if (log_level <= DEBUG) {
                        debug_loop_timing();
                    }

                    // Construct observation vector
                    ObservationVector observation;
                    int idx = 0;

                    // Gyro data (3)
                    observation.segment<GYRO_SIZE>(idx) = sensors.gyroscope;
                    if (log_level <= DEBUG) {
                        emit(graph("Sensors gyroscope values",
                                   sensors.gyroscope.x(),
                                   sensors.gyroscope.y(),
                                   sensors.gyroscope.z()));
                    };
                    idx += GYRO_SIZE;

                    // Gravity/Accelerometer data in body frame (3)
                    const Eigen::Vector3d g_world(0.0, 0.0, -1.0);
                    Eigen::Vector3d gravity                = sensors.Htw.rotation() * g_world;
                    observation.segment<GRAVITY_SIZE>(idx) = gravity;
                    if (log_level <= DEBUG) {
                        emit(
                            graph("Sensors accelerometer in body frame values", gravity.x(), gravity.y(), gravity.z()));
                    };
                    idx += GRAVITY_SIZE;

                    // Joint relative positions and velocities (40) in Mujoco's Tree-Traversal order
                    auto temp_sensors             = std::make_unique<Sensors>();
                    temp_sensors->servo           = sensors.servo;
                    JointStateVectors joint_state = sensors_to_joint_state(temp_sensors);

                    // Positions
                    JointVector current_joints_rel_mj        = nubots_to_mjlab(joint_state.position - default_pose);
                    observation.segment<JOINT_POS_SIZE>(idx) = current_joints_rel_mj;
                    if (log_level <= DEBUG) {
                        emit(graph("Joint current positions (mjlab)", current_joints_rel_mj.transpose()));
                    };
                    idx += JOINT_POS_SIZE;

                    // Velocities
                    JointVector current_velocities_mj        = nubots_to_mjlab(joint_state.velocity);
                    observation.segment<JOINT_POS_SIZE>(idx) = current_velocities_mj;
                    if (log_level <= DEBUG) {
                        emit(graph("Joint velocities (mjlab)", current_velocities_mj.transpose()));
                    };
                    idx += JOINT_POS_SIZE;

                    // Last action (21) (includes the new phase delta)
                    observation.segment<JOINT_POS_SIZE>(idx) = last_action_raw;
                    idx += JOINT_POS_SIZE;
                    observation(idx) = last_phase_delta;
                    idx += PHASE_DELTA_SIZE;

                    // Command (3)
                    observation.segment<COMMAND_SIZE>(idx) = walk_task.velocity_target;
                    if (log_level <= DEBUG) {
                        emit(graph("Walk velocity target",
                                   walk_task.velocity_target.x(),
                                   walk_task.velocity_target.y(),
                                   walk_task.velocity_target.z()));
                    }
                    idx += COMMAND_SIZE;

                    // Phase (2). Policy-owned gait clock (clock_learned variant): the phase is
                    // accumulated from the per-step phase delta the policy emits as an extra action
                    // output, not from wall-clock time. Here we only *observe* the phase accumulated
                    // from previous steps; it is advanced by this step's delta after inference below.
                    //
                    // Standing gate: when the commanded velocity magnitude (|linear| + |angular|) is
                    // below the threshold, the phase is treated as zero and the observation collapses
                    // to the origin, matching the command gate on gait_clock/PhaseDeltaAction in
                    // training so standing presents a distinct signal from walking.
                    const double command_magnitude =
                        walk_task.velocity_target.head<2>().norm() + std::abs(walk_task.velocity_target.z());
                    const bool phase_active = command_magnitude > cfg.phase_command_threshold;
                    observation.segment<PHASE_SIZE>(idx) =
                        phase_active
                            ? Eigen::Vector2d(std::sin(2 * M_PI * policy_phase), std::cos(2 * M_PI * policy_phase))
                            : Eigen::Vector2d::Zero();
                    idx += PHASE_SIZE;
                    // Count the control step for the loop-timing diagnostics.
                    ++control_step;

                    // Run inference
                    const InferenceOutput inference  = run_inference(observation);
                    JointVector inference_output_raw = inference.joint_actions;
                    if (log_level <= DEBUG) {
                        emit(graph("Raw joint action from inference", inference_output_raw.transpose()));
                    };

                    // Advance the policy-owned gait phase using the phase delta the policy just emitted.
                    // delta = (STEP_DT / gait_period) * raw_output, so a raw output of 1 advances one
                    // nominal step; the accumulated phase is wrapped into [0, 1). When the command is
                    // below the phase gate the phase is frozen at zero, matching PhaseDeltaAction.
                    if (phase_active) {
                        const double phase_delta = (STEP_DT / cfg.gait_period) * inference.phase_delta;
                        policy_phase             = std::fmod(policy_phase + phase_delta, 1.0);
                        if (policy_phase < 0.0) {
                            policy_phase += 1.0;
                        }
                    }
                    else {
                        policy_phase = 0.0;
                    }
                    if (log_level <= DEBUG) {
                        emit(graph("Policy gait phase", policy_phase, inference.phase_delta));
                    }

                    // Filter the raw policy action with an exponential moving average. The mjlab policy
                    // is trained without a per-step rate limit, so the raw output is used directly here;
                    // the only safety measure is the final absolute per-servo position-limit clamp below.
                    JointVector filtered_action_raw = inference_output_raw;
                    if (have_last_action) {
                        filtered_action_raw =
                            cfg.action_alpha * inference_output_raw + (1.0 - cfg.action_alpha) * last_action;
                    }

                    // Store the filtered action (mjlab order, no scaling/offset) as the EMA recurrence
                    // state for the next step's servo smoothing. Separately store the RAW joint output
                    // and RAW phase delta, which feed back into the next observation so it matches
                    // training's unprocessed action echo.
                    last_action      = filtered_action_raw;
                    last_action_raw  = inference_output_raw;
                    last_phase_delta = inference.phase_delta;
                    have_last_action = true;

                    const JointVector filtered_joint_offsets_mj = filtered_action_raw * cfg.nugus_action_scale;


                    // Convert into NUbots order, apply scaling
                    JointVector joint_offsets_scaled_nubots = mjlab_to_nubots(filtered_joint_offsets_mj);

                    // Hack: when the command velocity is below a threshold, zero the offsets so the
                    // robot holds the default pose. This side-steps unwanted policy behaviour at
                    // ~zero command. TODO: replace with an improved policy trained for zero command.
                    if (walk_task.velocity_target.norm() < cfg.command_velocity_threshold) {
                        joint_offsets_scaled_nubots = JointVector::Zero();
                    }

                    if (log_level <= DEBUG) {
                        emit(graph("Scaled joint offsets in NUbots order", joint_offsets_scaled_nubots.transpose()));
                    }

                    // Emit servo commands for the limbs only. The policy outputs 20 joints, but the
                    // head (indices 18, 19) is owned by skill::Look — emitting a full Body task would
                    // conflict on the Head resource and cause the Director to deny the whole task.
                    auto limbs = std::make_unique<Limbs>();
                    for (int i = 0; i < 18; ++i) {
                        auto servo  = std::make_unique<ServoCommand>();
                        servo->time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);
                        // Apply the policy offset to the default pose, then safety-clip the result to
                        // the physical servo limits so the policy cannot command an infeasible position.
                        servo->position  = std::clamp(default_pose[i] + joint_offsets_scaled_nubots[i],
                                                      servo_limit_min[i],
                                                      servo_limit_max[i]);
                        const float gain = i < 6 ? cfg.arm_servo_gain : cfg.leg_servo_gain;
                        servo->state     = ServoState(gain, cfg.servo_torque);
                        limbs->servos[joint_map[i].second] = *servo;
                    }
                    emit<Task>(limbs);

                    // Emit walk state
                    auto walk_state = std::make_unique<WalkState>(WalkState::State::WALKING,
                                                                  walk_task.velocity_target,
                                                                  0.0);  // Phase not used
                    emit(walk_state);

                    if (log_level <= DEBUG) {
                        emit(graph("Walk velocity target",
                                   walk_task.velocity_target.x(),
                                   walk_task.velocity_target.y(),
                                   walk_task.velocity_target.z()));
                    }
                }
            });
    }

    InferenceOutput RLWalk::run_inference(const ObservationVector& observation) {
        if (log_level <= DEBUG) {
            emit(graph("Observation", observation.transpose()));
        }

        std::vector<float> input_data(TOTAL_OBS_SIZE);
        for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
            input_data[i] = static_cast<float>(observation[i]);
        }

        // Create & set input tensor
        ov::Shape input_shape = {1, static_cast<size_t>(TOTAL_OBS_SIZE)};
        ov::Tensor input_tensor(ov::element::f32, input_shape, input_data.data());
        infer_request.set_input_tensor(input_tensor);

        // Run inference
        try {
            infer_request.infer();
        }
        catch (const std::exception& e) {
            log<ERROR>("Inference failed: ", e.what());
            return InferenceOutput{JointVector::Zero(), 0.0};
        }

        // Get output tensor. The policy emits TOTAL_ACTION_SIZE values: the JOINT_POS_SIZE joint
        // targets followed by the single gait phase delta (clock_learned variant).
        auto output_tensor = infer_request.get_output_tensor();
        float* output_data = output_tensor.data<float>();

        InferenceOutput result;
        for (int i = 0; i < JOINT_POS_SIZE; ++i) {
            result.joint_actions[i] = static_cast<double>(output_data[i]);
        }
        result.phase_delta = static_cast<double>(output_data[JOINT_POS_SIZE]);

        if (log_level <= DEBUG) {
            emit(graph("Raw action output values from inference", result.joint_actions.transpose()));
            emit(graph("Raw phase delta output from inference", result.phase_delta));
        }

        return result;
    }

    void RLWalk::reset_loop_timing() {
        have_timing_sample        = false;
        timing_samples            = 0;
        timing_period_sum         = 0.0;
        timing_period_sq_sum      = 0.0;
        timing_period_min         = 0.0;
        timing_period_max         = 0.0;
        timing_control_step_start = control_step;
    }

    void RLWalk::debug_loop_timing() {
        // Sample both clocks as close together as possible.
        const NUClear::clock::time_point now_nuclear           = NUClear::clock::now();
        const std::chrono::steady_clock::time_point now_steady = std::chrono::steady_clock::now();

        // Detect a discontinuity: this loop only ticks while walking, so if the walk is pre-empted by
        // another task (e.g. a get-up) the timing baseline freezes and the gap since the previous tick
        // far exceeds the control period. Measuring period/frequency/drift across that gap would report
        // broken information, so treat the current tick as a fresh baseline instead.
        const bool gap_detected =
            have_timing_sample && std::chrono::duration<double>(now_nuclear - last_tick_nuclear).count() > MAX_TICK_GAP;

        // First stable tick of this walk, or the first tick after a pause: (re)establish the baseline,
        // resetting the running statistics so they describe only the current continuous run. Nothing to
        // compare against yet, so return without emitting a sample.
        if (!have_timing_sample || gap_detected) {
            have_timing_sample        = true;
            last_tick_nuclear         = now_nuclear;
            last_tick_steady          = now_steady;
            walk_start_nuclear        = now_nuclear;
            walk_start_steady         = now_steady;
            last_timing_report        = now_nuclear;
            timing_control_step_start = control_step;
            timing_samples            = 0;
            timing_period_sum         = 0.0;
            timing_period_sq_sum      = 0.0;
            timing_period_min         = 0.0;
            timing_period_max         = 0.0;
            return;
        }

        // Per-tick period on each clock (seconds).
        const double dt_nuclear = std::chrono::duration<double>(now_nuclear - last_tick_nuclear).count();
        const double dt_steady  = std::chrono::duration<double>(now_steady - last_tick_steady).count();
        last_tick_nuclear       = now_nuclear;
        last_tick_steady        = now_steady;

        // Metrics. Gait-clock elapsed is measured from the control step at the current baseline so it
        // stays aligned with the wall-clock elapsed below after a re-baseline.
        const double model_elapsed        = static_cast<double>(control_step - timing_control_step_start) * STEP_DT;
        const double elapsed_nuclear      = std::chrono::duration<double>(now_nuclear - walk_start_nuclear).count();
        const double elapsed_steady       = std::chrono::duration<double>(now_steady - walk_start_steady).count();
        const double drift_nuclear        = model_elapsed - elapsed_nuclear;
        const double drift_steady         = model_elapsed - elapsed_steady;
        const double drift_nuclear_steady = elapsed_nuclear - elapsed_steady;

        // Update running statistics on the NUClear-clock period.
        ++timing_samples;
        timing_period_sum += dt_nuclear;
        timing_period_sq_sum += dt_nuclear * dt_nuclear;
        timing_period_min = (timing_samples == 1) ? dt_nuclear : std::min(timing_period_min, dt_nuclear);
        timing_period_max = (timing_samples == 1) ? dt_nuclear : std::max(timing_period_max, dt_nuclear);

        // Emit graphs
        emit(graph("RLWalk loop period (s)", dt_nuclear, dt_steady));
        emit(graph("RLWalk loop frequency (Hz)", 1.0 / dt_nuclear, 1.0 / dt_steady));
        emit(graph("RLWalk gait clock drift (s)", drift_nuclear, drift_steady));
        emit(graph("NUClear clock drift from steady clock (s)", drift_nuclear_steady));

        // Periodic rolling summary
        const double timing_report_period = 2.0;  // seconds
        const double since_report         = std::chrono::duration<double>(now_nuclear - last_timing_report).count();
        if (since_report >= timing_report_period) {
            const double mean = timing_period_sum / static_cast<double>(timing_samples);
            const double var  = std::max(0.0, timing_period_sq_sum / static_cast<double>(timing_samples) - mean * mean);
            const double jitter = std::sqrt(var);
            log<DEBUG>(
                fmt::format("| ticks: {:>6d} "
                            "| dt: {:7.5f}s ({:6.2f}Hz) "
                            "| jitter: {:7.5f}s "
                            "| min: {:7.5f}s "
                            "| max: {:7.5f}s "
                            "| drift vs nuclear: {:+8.5f}s "
                            "| vs wall: {:+8.5f}s "
                            "| NUClear clock vs Wall clock: {:+8.5f}s |",
                            timing_samples,
                            mean,
                            1.0 / mean,
                            jitter,
                            timing_period_min,
                            timing_period_max,
                            drift_nuclear,
                            drift_steady,
                            drift_nuclear_steady));
            last_timing_report = now_nuclear;
        }
    }

}  // namespace module::skill
