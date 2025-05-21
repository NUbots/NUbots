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
        {0, ServoID::L_HIP_YAW},        {1, ServoID::L_HIP_ROLL},     {2, ServoID::L_HIP_PITCH},
        {3, ServoID::L_KNEE},           {4, ServoID::L_ANKLE_PITCH},  {5, ServoID::L_ANKLE_ROLL},
        {6, ServoID::R_HIP_YAW},        {7, ServoID::R_HIP_ROLL},     {8, ServoID::R_HIP_PITCH},
        {9, ServoID::R_KNEE},           {10, ServoID::R_ANKLE_PITCH}, {11, ServoID::R_ANKLE_ROLL},
        {12, ServoID::HEAD_YAW},        {13, ServoID::HEAD_PITCH},    {14, ServoID::L_SHOULDER_PITCH},
        {15, ServoID::L_SHOULDER_ROLL}, {16, ServoID::L_ELBOW},       {17, ServoID::R_SHOULDER_PITCH},
        {18, ServoID::R_SHOULDER_ROLL}, {19, ServoID::R_ELBOW}};

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

            // Initialize the model
            initialize_model();
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

                    // Gyro data (3)
                    observation.segment<GYRO_SIZE>(idx) = sensors.gyroscope;
                    log<INFO>("Gyro: ", observation.segment<GYRO_SIZE>(idx).transpose());
                    idx += GYRO_SIZE;

                    // Gravity/Accelerometer data (3)
                    observation.segment<GRAVITY_SIZE>(idx) = sensors.accelerometer;
                    log<INFO>("Accelerometer: ", observation.segment<GRAVITY_SIZE>(idx).transpose());
                    idx += GRAVITY_SIZE;

                    // Command (3)
                    observation.segment<COMMAND_SIZE>(idx) = walk_task.velocity_target;
                    log<INFO>("Velocity target: ", observation.segment<COMMAND_SIZE>(idx).transpose());
                    idx += COMMAND_SIZE;

                    // Joint positions relative to default pose (20)
                    // Note: We need to map the joint positions from sensors to the correct order
                    // This is a placeholder - you'll need to map the actual joint positions
                    auto temp_sensors                        = std::make_unique<Sensors>();
                    temp_sensors->servo                      = sensors.servo;
                    JointVector current_joints               = sensors_to_configuration(temp_sensors);
                    observation.segment<JOINT_POS_SIZE>(idx) = current_joints - default_pose;
                    idx += JOINT_POS_SIZE;

                    // Last action (20)
                    observation.segment<JOINT_POS_SIZE>(idx) = last_action;
                    idx += JOINT_POS_SIZE;

                    // Run inference
                    JointVector joint_angles = run_inference(observation);
                    log<INFO>("Joint angles: ", joint_angles.transpose());

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
                        data_file << joint_angles[i];
                        if (i < JOINT_POS_SIZE - 1)
                            data_file << ", ";
                    }
                    data_file << "]\n";
                    data_file << "}\n";
                    data_file.close();
                    log<INFO>("Saved example data to example_data.json");

                    // Store the last action
                    last_action = joint_angles;

                    // Emit servo commands
                    auto body = std::make_unique<Body>();
                    for (int i = 0; i < cfg.num_joints; ++i) {
                        auto servo      = std::make_unique<ServoCommand>();
                        servo->time     = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);
                        servo->position = default_pose[i];
                        servo->state    = ServoState(1.0, 100);  // Default gains
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
            return JointVector::Zero();
        }

        try {
            // Convert observation to float array
            std::vector<float> input_data(TOTAL_OBS_SIZE);
            for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
                input_data[i] = static_cast<float>(observation[i]);
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
            JointVector joint_angles;
            for (int i = 0; i < JOINT_POS_SIZE; ++i) {
                joint_angles[i] = static_cast<double>(output_data[i]);
            }

            return joint_angles;
        }
        catch (const std::exception& e) {
            log<ERROR>("Inference failed: ", e.what());
            return JointVector::Zero();
        }
    }

}  // namespace module::skill
