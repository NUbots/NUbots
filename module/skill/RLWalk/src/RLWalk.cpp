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

            // Initialize vectors
            last_action      = JointVector::Zero();
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

                    // Last action (20) in Mujoco's Tree-Traversal order without scaling or offsets applied
                    observation.segment<JOINT_POS_SIZE>(idx) = last_action;
                    idx += JOINT_POS_SIZE;

                    // Command (3)
                    auto vel_targ = walk_task.velocity_target;
                    observation.segment<COMMAND_SIZE>(idx) = vel_targ;
                    if (log_level <= DEBUG) {
                        emit(graph("Walk velocity target",
                                   vel_targ.x(),
                                   vel_targ.y(),
                                   vel_targ.z()));
                    }
                    idx += COMMAND_SIZE;

                    // Phase (2). Advance the gait phase from the control-step count and fixed control
                    // timestep (control_step * STEP_DT) rather than wall-clock time. This is
                    // deterministic, monotonic, and immune to clock jitter/jumps and scheduling delays.
                    const double elapsed = static_cast<double>(control_step) * STEP_DT;
                    const double phase   = std::fmod(elapsed / cfg.gait_period, 1.0);
                    observation.segment<PHASE_SIZE>(idx) =
                        Eigen::Vector2d(std::sin(2 * M_PI * phase), std::cos(2 * M_PI * phase));
                    idx += PHASE_SIZE;
                    // Advance the gait clock by one control step for the next update
                    ++control_step;

                    // Run inference
                    JointVector inference_output_raw = run_inference(observation);
                    if (log_level <= DEBUG) {
                        emit(graph("Raw joint action from inference", inference_output_raw.transpose()));
                    };

                    // Clip the output here as a safety measure to prevent extremely fast movements. TODO: FIXME
                    const double max_joint_offset = 0.2;  // 0.2 radians per update (50 Hz) = ~600 degrees per second.

                    // Convert raw action to physical joint offsets (mjlab order), then clip per-joint
                    // against the current measured offsets to limit command rate.
                    const JointVector desired_joint_offsets_mj = inference_output_raw * cfg.nugus_action_scale;
                    JointVector clipped_joint_offsets_mj       = desired_joint_offsets_mj;

                    for (int i = 0; i < cfg.num_joints; ++i) {
                        const double lower          = current_joints_rel_mj[i] - max_joint_offset;
                        const double upper          = current_joints_rel_mj[i] + max_joint_offset;
                        clipped_joint_offsets_mj[i] = std::clamp(desired_joint_offsets_mj[i], lower, upper);
                    }

                    JointVector clipped_action_raw = inference_output_raw;
                    if (std::abs(cfg.nugus_action_scale) > 1e-9) {
                        clipped_action_raw = clipped_joint_offsets_mj / cfg.nugus_action_scale;
                    }

                    // Filter the actions using an exponential moving average
                    JointVector filtered_action_raw = clipped_action_raw;
                    if (have_last_action) {
                        filtered_action_raw =
                            cfg.action_alpha * clipped_action_raw + (1.0 - cfg.action_alpha) * last_action;
                    }

                    // Store the filtered action in mjlab order without scaling or offsets.
                    last_action      = filtered_action_raw;
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

    JointVector RLWalk::run_inference(const ObservationVector& observation) {
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
            return JointVector::Zero();
        }

        // Get output tensor
        auto output_tensor = infer_request.get_output_tensor();
        float* output_data = output_tensor.data<float>();

        JointVector joint_angles_raw;
        for (int i = 0; i < JOINT_POS_SIZE; ++i) {
            joint_angles_raw[i] = static_cast<double>(output_data[i]);
        }

        if (log_level <= DEBUG) {
            emit(graph("Raw action output values from inference", joint_angles_raw.transpose()));
        }

        return joint_angles_raw;
    }

    void RLWalk::reset_loop_timing() {
        have_timing_sample   = false;
        timing_samples       = 0;
        timing_period_sum    = 0.0;
        timing_period_sq_sum = 0.0;
        timing_period_min    = 0.0;
        timing_period_max    = 0.0;
    }

    void RLWalk::debug_loop_timing() {
        // Sample both clocks as close together as possible.
        const NUClear::clock::time_point now_nuclear           = NUClear::clock::now();
        const std::chrono::steady_clock::time_point now_steady = std::chrono::steady_clock::now();

        // First stable tick of this walk: establish the baseline, nothing to compare against yet.
        if (!have_timing_sample) {
            have_timing_sample = true;
            last_tick_nuclear  = now_nuclear;
            last_tick_steady   = now_steady;
            walk_start_nuclear = now_nuclear;
            walk_start_steady  = now_steady;
            last_timing_report = now_nuclear;
            return;
        }

        // Per-tick period on each clock (seconds).
        const double dt_nuclear = std::chrono::duration<double>(now_nuclear - last_tick_nuclear).count();
        const double dt_steady  = std::chrono::duration<double>(now_steady - last_tick_steady).count();
        last_tick_nuclear       = now_nuclear;
        last_tick_steady        = now_steady;

        // Metrics
        const double model_elapsed        = static_cast<double>(control_step) * STEP_DT;
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
