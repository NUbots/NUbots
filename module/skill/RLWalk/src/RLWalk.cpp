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

    std::vector<int> action_conversion_order = {18, 2, 7, 8, 5, 19, 15, 10, 1, 9, 4, 14, 6, 11, 3, 13, 16, 0, 12, 17};

    inline Eigen::Matrix<double, 20, 1> mjlab_to_nubots(const Eigen::Matrix<double, 20, 1>& mjlab_joint_offsets) {
        Eigen::Matrix<double, 20, 1> nubots_joint_offsets = Eigen::Matrix<double, 20, 1>::Zero();
        // Convert from MJlabs order to NUbots order
        for (int i = 0; i < mjlab_joint_offsets.size(); ++i) {
            nubots_joint_offsets(action_conversion_order[i]) = mjlab_joint_offsets(i);
        }
        return nubots_joint_offsets;
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
            initialise_model_locked();
        });

        on<Every<1000, Per<std::chrono::milliseconds>>, Single>().then([this] {
            std::scoped_lock lock(model_mutex);
            if (model_state == ModelState::READY)
                return;

            const auto now = std::chrono::steady_clock::now();
            if (now < next_retry) {
                return;
            }

            if (initialise_model_locked()) {
                log<INFO>("RLWalk inference model initialised successfully");
            }
            log<DEBUG>("Model state after retry: ", (model_state == ModelState::READY ? "READY" : "FAILED"));
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
                    if (log_level <= DEBUG) {
                        emit(graph("Sensors accelerometer values",
                                   sensors.accelerometer.x(),
                                   sensors.accelerometer.y(),
                                   sensors.accelerometer.z()));
                    };
                    idx += ACC_SIZE;

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
                    Eigen::Vector3d gravity                = sensors.Htw.inverse().rotation() * g_world;
                    observation.segment<GRAVITY_SIZE>(idx) = gravity;
                    if (log_level <= DEBUG) {
                        emit(
                            graph("Sensors accelerometer in body frame values", gravity.x(), gravity.y(), gravity.z()));
                    };
                    idx += GRAVITY_SIZE;

                    // Joint positions relative to default pose (20)
                    auto temp_sensors                        = std::make_unique<Sensors>();
                    temp_sensors->servo                      = sensors.servo;
                    JointVector current_joints               = sensors_to_configuration(temp_sensors);
                    observation.segment<JOINT_POS_SIZE>(idx) = current_joints - default_pose;
                    if (log_level <= DEBUG) {
                        emit(graph("Joint current positions", current_joints.transpose()));
                    };
                    idx += JOINT_POS_SIZE;

                    // Joint velocities relative to default pose (20)
                    // Estimate using finite differences with exponential smoothing to reduce noise
                    const auto now                   = NUClear::clock::now();
                    JointVector joint_vel            = JointVector::Zero();
                    static const double alpha        = 0.5;
                    static JointVector filtered_vel  = JointVector::Zero();
                    static bool filtered_initialised = false;

                    if (have_previous_pose) {
                        std::chrono::duration<double> elapsed = now - last_update_time;
                        double dt                             = elapsed.count();
                        if (dt > 0.0) {
                            JointVector raw_vel = (current_joints - previous_pose) / dt;
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
                    previous_pose                            = current_joints;
                    last_update_time                         = now;
                    observation.segment<JOINT_POS_SIZE>(idx) = joint_vel;
                    if (log_level <= DEBUG) {
                        emit(graph("Joint velocity", joint_vel.transpose()));
                    };
                    idx += JOINT_POS_SIZE;

                    // Last action (20)
                    observation.segment<JOINT_POS_SIZE>(idx) = last_action;
                    idx += JOINT_POS_SIZE;

                    // Command (3)
                    observation.segment<COMMAND_SIZE>(idx) = walk_task.velocity_target;
                    if (log_level <= DEBUG) {
                        emit(graph("Walk velocity target",
                                   walk_task.velocity_target.x(),
                                   walk_task.velocity_target.y(),
                                   walk_task.velocity_target.z()));
                    }
                    idx += COMMAND_SIZE;

                    // Run inference
                    JointVector joint_offsets_action = run_inference(observation);
                    if (log_level <= DEBUG) {
                        emit(graph("Joint offset action from inference", joint_offsets_action.transpose()));
                    };

                    // Store the last action
                    last_action = joint_offsets_action;

                    // Emit servo commands
                    auto body = std::make_unique<Body>();
                    for (int i = 0; i < cfg.num_joints; ++i) {
                        auto servo  = std::make_unique<ServoCommand>();
                        servo->time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);
                        // Apply the joint angles from the policy as offsets to the default pose
                        servo->position                   = default_pose[i] + joint_offsets_action[i];
                        servo->state                      = ServoState(1.0, 100);  // Default gains
                        body->servos[joint_map[i].second] = *servo;
                    }
                    emit<Task>(body);

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

    void RLWalk::invalidate_model_locked() {
        compiled_model = ov::CompiledModel{};
        infer_request  = ov::InferRequest{};
        model_state    = ModelState::FAILED;
    }

    bool RLWalk::initialise_model_locked() {
        auto shape_to_string = [](const ov::Shape& s) {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < s.size(); ++i) {
                if (i > 0)
                    oss << ", ";
                oss << s[i];
            }
            oss << "]";
            return oss.str();
        };

        try {
            const std::filesystem::path p(cfg.model_path);
            const auto abs = std::filesystem::absolute(p).string();

            if (!std::filesystem::exists(p)) {
                throw std::runtime_error("Model file not found: " + abs);
            }

            log<INFO>("RLWalk init model_path=",
                      abs,
                      " device=",
                      cfg.device,
                      " input_name=",
                      cfg.input_name,
                      " output_name=",
                      cfg.output_name);

            std::string last_error = "unknown";

            auto model = core.read_model(abs);
            auto cm    = core.compile_model(model, cfg.device);
            auto req   = cm.create_infer_request();

            // Validate configured ports and shapes once
            const auto in  = cfg.input_name.empty() ? cm.input() : cm.input(cfg.input_name);
            const auto out = cfg.output_name.empty() ? cm.output() : cm.output(cfg.output_name);

            const auto in_shape  = in.get_shape();
            const auto out_shape = out.get_shape();
            if (in_shape.size() != 2 || in_shape[1] != static_cast<size_t>(TOTAL_OBS_SIZE)) {
                throw std::runtime_error("Unexpected input shape");
            }
            if (out_shape.size() != 2 || out_shape[1] != static_cast<size_t>(JOINT_POS_SIZE)) {
                throw std::runtime_error("Unexpected output shape");
            }

            // Commit only after full success
            compiled_model = std::move(cm);
            infer_request  = std::move(req);
            model_state    = ModelState::READY;
            retry_backoff  = std::chrono::milliseconds(1000);

            log<INFO>("Initialisation success");

            return true;
        }
        catch (const std::exception& e) {
            log<ERROR>("RLWalk model init failed: ", e.what());
            invalidate_model_locked();
            next_retry    = std::chrono::steady_clock::now() + retry_backoff;
            retry_backoff = std::min(retry_backoff * 2, std::chrono::milliseconds(5000));
            return false;
        }
    }

    JointVector RLWalk::run_inference(const ObservationVector& observation) {
        if (log_level <= DEBUG) {
            emit(graph("DEBUG: NON-normalized observation", observation.transpose()));
        }
        std::scoped_lock lock(model_mutex);

        if (model_state != ModelState::READY) {
            return JointVector::Zero();
        }

        try {

            std::vector<float> input_data(TOTAL_OBS_SIZE);
            for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
                input_data[i] = static_cast<float>(observation[i]);
            }

            // Create & set input tensor
            ov::Shape input_shape = {1, static_cast<size_t>(TOTAL_OBS_SIZE)};
            ov::Tensor input_tensor(ov::element::f32, input_shape, input_data.data());
            infer_request.set_input_tensor(input_tensor);

            // Run inference
            infer_request.infer();

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

            // Convert output values to joint angle offsets
            float action_scale        = 0.049445848912000656f;  // From mjlab
            JointVector joint_offsets = joint_angles_raw * action_scale;

            // TODO: Ensure order of joints action output is in the same order as in RLWalk.yaml default_pose
            // joint_offsets = mjlab_to_nubots(joint_offsets);
            return joint_offsets;
        }
        catch (const std::exception& e) {
            log<ERROR>("Inference failed: ", e.what());
            invalidate_model_locked();
            next_retry = std::chrono::steady_clock::now() + retry_backoff;
            return JointVector::Zero();
        }
    }

}  // namespace module::skill
