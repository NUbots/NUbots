#include "RLWalk.hpp"

#include <fstream>
#include <iomanip>

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

    using message::actuation::Body;
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

    inline Eigen::Matrix<double, 20, 1> sensors_to_configuration(const std::unique_ptr<Sensors>& sensors) {
        Eigen::Matrix<double, 20, 1> q = Eigen::Matrix<double, 20, 1>::Zero();
        q.resize(sensors->servo.size(), 1);
        for (const auto& [index, servo_id] : joint_map) {
            q(index, 0) = sensors->servo.at(servo_id).present_position;
        }
        return q;
    }

    RLWalk::RLWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("RLWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RLWalk.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Load model configuration
            cfg.model_path  = config["model"]["path"].as<std::string>();
            cfg.device      = config["model"]["device"].as<std::string>();
            cfg.input_name  = config["model"]["input_name"].as<std::string>();
            cfg.output_name = config["model"]["output_name"].as<std::string>();
            cfg.num_joints  = config["model"]["num_joints"].as<int>();
            cfg.obs_size    = config["model"]["obs_size"].as<int>();

            // Initialize vectors
            last_action = JointVector::Zero();

            default_pose = JointVector(config["default_pose"].as<Expression>());

            previous_pose      = JointVector(config["previous_pose"].as<Expression>());
            have_previous_pose = false;
            last_update_time   = NUClear::clock::now();

            // Initialize the model
            initialize_model();
        });

        // Load normalisation parameters from YAML
        on<Configuration>("normalisation_params.yaml").then([this](const Configuration& norm_config) {
            auto mean_vec = norm_config["obs_mean"].as<std::vector<double>>();
            auto std_vec  = norm_config["obs_std"].as<std::vector<double>>();
            auto var_vec  = norm_config["obs_var"].as<std::vector<double>>();

            if (mean_vec.size() != TOTAL_OBS_SIZE || std_vec.size() != TOTAL_OBS_SIZE
                || var_vec.size() != TOTAL_OBS_SIZE) {
                log<ERROR>("Normalization params size mismatch! Expected: ",
                           TOTAL_OBS_SIZE,
                           " Got mean: ",
                           mean_vec.size(),
                           " std: ",
                           std_vec.size(),
                           " var: ",
                           var_vec.size());
            }
            else {
                for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
                    _mean[i] = mean_vec[i];
                    _std[i]  = std_vec[i];
                    _var[i]  = var_vec[i];
                }
                normalisation_loaded = true;
                log<INFO>("Loaded normalization parameters from normalisation_params.yaml");
            }
        });

        // Start - Runs every time the Walk provider starts
        on<Start<WalkTask>>().then([this]() {
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
            .then([this](const WalkTask& walk_task, const Sensors& sensors, const Stability& stability) {
                // Only run if we're in a stable state
                if (stability >= Stability::DYNAMIC) {
                    // Construct observation vector
                    ObservationVector observation;
                    int idx = 0;

                    // Accelerometer data in body frame (3)
                    observation.segment<ACC_SIZE>(idx) = sensors.accelerometer;
                    // log<DEBUG>("Accelerometer: ", observation.segment<ACC_SIZE>(idx).transpose());
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Sensors accelerometer values",
                                   sensors.accelerometer.x(),
                                   sensors.accelerometer.y(),
                                   sensors.accelerometer.z()));
                    };
                    idx += ACC_SIZE;

                    // Gyro data (3)
                    observation.segment<GYRO_SIZE>(idx) = sensors.gyroscope;  // Convert to radians/s
                    // log<DEBUG>("Gyro: ", observation.segment<GYRO_SIZE>(idx).transpose());
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Sensors gyroscope values",
                                   sensors.gyroscope.x(),
                                   sensors.gyroscope.y(),
                                   sensors.gyroscope.z()));
                    };
                    idx += GYRO_SIZE;

                    // Gravity/Accelerometer data in world frame (3)
                    Eigen::Vector3d gravity = sensors.Htw.inverse().rotation() * sensors.accelerometer;
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Sensors accelerometer in world frame values",
                                   gravity.x(),
                                   gravity.y(),
                                   gravity.z()));
                    };
                    // log<DEBUG>("Gravity: ", gravity.transpose());
                    // log<DEBUG>("Gravity magnitude: ", gravity.norm());
                    observation.segment<GRAVITY_SIZE>(idx) = gravity;
                    // log<DEBUG>("Accelerometer in worldframe: ",
                    // observation.segment<GRAVITY_SIZE>(idx).transpose());
                    idx += GRAVITY_SIZE;

                    // Joint positions relative to default pose (20)
                    // Note: We need to map the joint positions from sensors to the correct order
                    // This is a placeholder - you'll need to map the actual joint positions TODO
                    auto temp_sensors                        = std::make_unique<Sensors>();
                    temp_sensors->servo                      = sensors.servo;
                    JointVector current_joints               = sensors_to_configuration(temp_sensors);
                    observation.segment<JOINT_POS_SIZE>(idx) = current_joints - default_pose;
                    idx += JOINT_POS_SIZE;
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Joint current positions", current_joints.transpose()));
                    };
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Servo default pose", default_pose.transpose()));
                    };

                    // Joint velocities relative to default pose (20)
                    // Estimate using finite differences with exponential smoothing to reduce noise
                    const auto now                   = NUClear::clock::now();
                    JointVector joint_vel            = JointVector::Zero();
                    static const double alpha        = 0.1;
                    static JointVector filtered_vel  = JointVector::Zero();
                    static bool filtered_initialised = false;

                    if (have_previous_pose) {
                        std::chrono::duration<double> elapsed = now - last_update_time;
                        double dt                             = elapsed.count();
                        if (dt > 0.0) {
                            JointVector raw_vel = (current_joints - previous_pose) / dt;
                            // Convert to degrees/s for the inference
                            raw_vel = raw_vel * (180.0 / M_PI);
                            if (!filtered_initialised) {
                                filtered_vel         = raw_vel;
                                filtered_initialised = true;
                            }
                            else {
                                filtered_vel = alpha * raw_vel + (1 - alpha) * filtered_vel;
                            }
                            joint_vel = filtered_vel;
                        }
                        else if (dt < 0.0) {
                            log<ERROR>("Negative dt for joint velocity estimation: ", dt, ". Assuming zero velocity");
                        }
                    }
                    else {
                        have_previous_pose   = true;
                        filtered_initialised = false;
                    }
                    // joint_vel                                = JointVector::Zero();  // DEBUGGING removeme
                    previous_pose                            = current_joints;
                    last_update_time                         = now;
                    observation.segment<JOINT_POS_SIZE>(idx) = joint_vel;
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Joint velocity", joint_vel.transpose()));
                    };
                    idx += JOINT_POS_SIZE;

                    // Last action (20)
                    observation.segment<JOINT_POS_SIZE>(idx) = last_action;
                    idx += JOINT_POS_SIZE;

                    // Command (3)
                    observation.segment<COMMAND_SIZE>(idx) = walk_task.velocity_target;
                    // log<DEBUG>("Velocity target: ", observation.segment<COMMAND_SIZE>(idx).transpose());
                    idx += COMMAND_SIZE;

                    // Run inference
                    JointVector joint_angles_deg = run_inference(observation);
                    // Assume output is in degrees, convert to radians
                    JointVector joint_angles_rad = joint_angles_deg * (M_PI / 180.0);
                    // log<DEBUG>("Joint angles: ", joint_angles.transpose());
                    if (log_level <= DEBUG) {
                        emit(graph("DEBUG: Joint offset goal positions from inference", joint_angles_rad.transpose()));
                    };

                    // Save example data to file
                    std::ofstream data_file("recordings/example_data.json");
                    data_file << std::fixed << std::setprecision(6);
                    data_file << "{\n";
                    data_file << "  \"observation\": [";
                    for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
                        data_file << observation[i];
                        if (i < TOTAL_OBS_SIZE - 1)
                            data_file << ", ";
                    }
                    data_file << "],\n";
                    data_file << "  \"action\": [";
                    for (int i = 0; i < JOINT_POS_SIZE; ++i) {
                        data_file << joint_angles_rad[i];
                        if (i < JOINT_POS_SIZE - 1)
                            data_file << ", ";
                    }
                    data_file << "]\n";
                    data_file << "}\n";
                    data_file.close();
                    // log<DEBUG>("Saved example data to example_data.json");

                    // Store the last action
                    last_action = joint_angles_deg;

                    // Emit servo commands
                    auto body = std::make_unique<Body>();
                    for (int i = 0; i < cfg.num_joints; ++i) {
                        auto servo  = std::make_unique<ServoCommand>();
                        servo->time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);
                        // Apply the joint angles from the policy as offsets to the default pose
                        servo->position                   = default_pose[i] + joint_angles_rad[i];
                        servo->state                      = ServoState(1.0, 100);  // Default gains
                        body->servos[joint_map[i].second] = *servo;
                    }
                    emit<Task>(body);

                    // Emit walk state
                    auto walk_state = std::make_unique<WalkState>(WalkState::State::WALKING,
                                                                  walk_task.velocity_target,
                                                                  0.0);  // Phase not used
                    emit(walk_state);

                    // Debug output
                    if (log_level <= DEBUG) {
                        emit(graph("Walk velocity target",
                                   walk_task.velocity_target.x(),
                                   walk_task.velocity_target.y(),
                                   walk_task.velocity_target.z()));
                    }
                }
            });
    }

    void RLWalk::initialize_model() {
        try {
            // Create OpenVINO Core
            ov::Core core;

            // Read the model
            auto model = core.read_model(cfg.model_path);

            // Log model information
            log<INFO>("Model loaded from: ", cfg.model_path);
            log<INFO>("Model device: ", cfg.device);

            // Log input information
            for (const auto& input : model->inputs()) {
                log<INFO>("Input: ", input.get_any_name());
                log<INFO>("  Shape: ", input.get_shape());
                log<INFO>("  Type: ", input.get_element_type());
            }

            // Log output information
            for (const auto& output : model->outputs()) {
                log<INFO>("Output: ", output.get_any_name());
                log<INFO>("  Shape: ", output.get_shape());
                log<INFO>("  Type: ", output.get_element_type());
            }

            // Log available devices
            log<INFO>("Available devices:");
            for (const auto& device : core.get_available_devices()) {
                log<INFO>("  ", device);
            }

            // Compile the model
            compiled_model = core.compile_model(model, cfg.device);

            // Create inference request
            infer_request = compiled_model.create_infer_request();

            // Log compilation info
            log<INFO>("Model compiled successfully");

            // Validate input shape
            auto input_shape = model->input().get_shape();
            if (input_shape[1] != TOTAL_OBS_SIZE) {
                log<ERROR>("Model input size mismatch. Expected: ", TOTAL_OBS_SIZE, " Got: ", input_shape[1]);
                model_initialized = false;
                return;
            }

            // Validate output shape
            auto output_shape = model->output().get_shape();
            if (output_shape[1] != JOINT_POS_SIZE) {
                log<ERROR>("Model output size mismatch. Expected: ", JOINT_POS_SIZE, " Got: ", output_shape[1]);
                model_initialized = false;
                return;
            }

            model_initialized = true;
            log<INFO>("RLWalk model initialized successfully");
        }
        catch (const std::exception& e) {
            log<ERROR>("Failed to initialize RLWalk model: ", e.what());
            model_initialized = false;
        }
    }

    JointVector RLWalk::run_inference(const ObservationVector& observation) {
        if (!model_initialized) {
            log<ERROR>("Cannot run inference: model not initialized");
            log<INFO>("Attempting to reinitialize model...");
            initialize_model();
            return JointVector::Zero();
        }

        if (!normalisation_loaded) {
            log<ERROR>("Cannot run inference: normalisation parameters not loaded");
            return JointVector::Zero();
        }

        try {
            // Normalise observation
            emit(graph("DEBUG: NON-normalized observation", observation.transpose()));
            ObservationVector norm_observation = (observation - _mean).cwiseQuotient(_std + EPS);
            emit(graph("DEBUG: Normalized observation", norm_observation.transpose()));

            // Convert normalised observation to float array
            std::vector<float> input_data(TOTAL_OBS_SIZE);
            for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
                input_data[i] = static_cast<float>(norm_observation[i]);
            }

            // Create input tensor
            ov::Shape input_shape = {1, static_cast<size_t>(TOTAL_OBS_SIZE)};
            ov::Tensor input_tensor(ov::element::f32, input_shape, input_data.data());

            // Set input tensor
            infer_request.set_input_tensor(input_tensor);

            // Run inference
            infer_request.infer();

            // Get output tensor
            auto output_tensor = infer_request.get_output_tensor();
            float* output_data = output_tensor.data<float>();

            // Convert output to Eigen vector
            JointVector joint_angles_deg_out;
            for (int i = 0; i < JOINT_POS_SIZE; ++i) {
                joint_angles_deg_out[i] = static_cast<double>(output_data[i]);
            }

            return joint_angles_deg_out;
        }
        catch (const std::exception& e) {
            log<ERROR>("Inference failed: ", e.what());
            return JointVector::Zero();
        }
    }

}  // namespace module::skill
