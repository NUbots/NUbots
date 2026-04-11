#include "Mujoco.hpp"

#include <X11/Xlib.h>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/output/Mujoco.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/mujoco/messages.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

extern "C" {
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
}

namespace module::platform {

    using extension::Configuration;

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::platform::get_raw_servo;
    using utility::support::Expression;


    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::input::Image;
    using message::platform::RawSensors;
    using message::platform::mujoco::JointCommand;
    using message::platform::mujoco::JointState;
    using message::platform::mujoco::SimulationRequest;
    using message::platform::mujoco::SimulationResponse;

    std::map<uint32_t, std::string> id_to_joint_name = {
        {0, "right_shoulder_pitch"}, {1, "left_shoulder_pitch"}, {2, "right_shoulder_roll"}, {3, "left_shoulder_roll"},
        {4, "right_elbow_pitch"},    {5, "left_elbow_pitch"},    {6, "right_hip_yaw"},       {7, "left_hip_yaw"},
        {8, "right_hip_roll [hip]"}, {9, "left_hip_roll [hip]"}, {10, "right_hip_pitch"},    {11, "left_hip_pitch"},
        {12, "right_knee_pitch"},    {13, "left_knee_pitch"},    {14, "right_ankle_pitch"},  {15, "left_ankle_pitch"},
        {16, "right_ankle_roll"},    {17, "left_ankle_roll"},    {18, "neck_yaw"},           {19, "head_pitch"}};

    int Mujoco::tcpip_connect() const {
        addrinfo hints{};
        memset(&hints, 0, sizeof(addrinfo));
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo* address = nullptr;
        const int error   = getaddrinfo(cfg.remote_host.c_str(), cfg.remote_port.c_str(), &hints, &address);
        if (error != 0) {
            log<ERROR>(fmt::format("Cannot resolve remote bridge {}:{} ({})",
                                   cfg.remote_host,
                                   cfg.remote_port,
                                   gai_strerror(error)));
            return -1;
        }

        for (addrinfo* ptr = address; ptr != nullptr; ptr = ptr->ai_next) {
            const int fd = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
            if (fd == -1) {
                continue;
            }
            if (connect(fd, ptr->ai_addr, ptr->ai_addrlen) != -1) {
                freeaddrinfo(address);
                return fd;
            }
            close(fd);
        }

        freeaddrinfo(address);
        log<ERROR>(fmt::format("Cannot connect to remote bridge {}:{}", cfg.remote_host, cfg.remote_port));
        return -1;
    }

    bool Mujoco::connect_remote_bridge() {
        if (remote_connected && remote_fd >= 0) {
            return true;
        }

        remote_fd = tcpip_connect();
        if (remote_fd < 0) {
            remote_connected = false;
            return false;
        }

        rx_buffer.clear();
        remote_sequence_id = 0;
        remote_connected   = true;
        log<INFO>("Connected to remote MuJoCo bridge at", cfg.remote_host + ":" + cfg.remote_port);
        return true;
    }

    bool Mujoco::send_remote_request(const SimulationRequest& request) {
        std::vector<uint8_t> payload = NUClear::util::serialise::Serialise<SimulationRequest>::serialise(request);
        const uint32_t length        = htonl(static_cast<uint32_t>(payload.size()));

        if (send(remote_fd, &length, sizeof(length), 0) != sizeof(length)) {
            log<ERROR>("Failed to send MuJoCo bridge request length");
            return false;
        }
        if (send(remote_fd, payload.data(), payload.size(), 0) != int(payload.size())) {
            log<ERROR>("Failed to send MuJoCo bridge request payload");
            return false;
        }
        return true;
    }

    bool Mujoco::read_remote_response(SimulationResponse& response) {
        while (true) {
            auto read_length = [](const std::vector<uint8_t>& buffer) {
                return buffer.size() >= sizeof(uint32_t) ? ntohl(*reinterpret_cast<const uint32_t*>(buffer.data()))
                                                         : 0u;
            };

            const uint32_t length = read_length(rx_buffer);
            if (length > 0 && rx_buffer.size() >= (sizeof(uint32_t) + length)) {
                uint8_t* payload = rx_buffer.data() + sizeof(uint32_t);
                response = NUClear::util::serialise::Serialise<SimulationResponse>::deserialise(payload, length);
                rx_buffer.erase(rx_buffer.begin(), std::next(rx_buffer.begin(), sizeof(uint32_t) + length));
                return true;
            }

            unsigned long available = 0;
            if (::ioctl(remote_fd, FIONREAD, &available) < 0) {
                log<ERROR>("Error querying MuJoCo bridge socket for available bytes");
                return false;
            }

            if (available == 0) {
                // No complete packet available yet, try a blocking read for at least one byte.
                uint8_t byte = 0;
                const auto n = ::read(remote_fd, &byte, 1);
                if (n <= 0) {
                    return false;
                }
                rx_buffer.push_back(byte);
                continue;
            }

            const size_t old_size = rx_buffer.size();
            rx_buffer.resize(old_size + available);
            const auto bytes_read = ::read(remote_fd, rx_buffer.data() + old_size, available);
            if (bytes_read <= 0) {
                return false;
            }
            rx_buffer.resize(old_size + size_t(bytes_read));
        }
    }

    void Mujoco::emit_remote_raw_sensors(const SimulationResponse& response) {
        auto raw_sensors = std::make_unique<RawSensors>();

        raw_sensors->accelerometer.x() = response.accelerometer.x;
        raw_sensors->accelerometer.y() = response.accelerometer.y;
        raw_sensors->accelerometer.z() = response.accelerometer.z;

        raw_sensors->gyroscope.x() = response.gyroscope.x;
        raw_sensors->gyroscope.y() = response.gyroscope.y;
        raw_sensors->gyroscope.z() = response.gyroscope.z;

        auto set_servo = [&](const JointState& joint) {
            if (joint.name == "right_shoulder_pitch") {
                raw_sensors->servo.r_shoulder_pitch.present_position = joint.position;
                raw_sensors->servo.r_shoulder_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_shoulder_pitch") {
                raw_sensors->servo.l_shoulder_pitch.present_position = joint.position;
                raw_sensors->servo.l_shoulder_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_shoulder_roll") {
                raw_sensors->servo.r_shoulder_roll.present_position = joint.position;
                raw_sensors->servo.r_shoulder_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_shoulder_roll") {
                raw_sensors->servo.l_shoulder_roll.present_position = joint.position;
                raw_sensors->servo.l_shoulder_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_elbow_pitch") {
                raw_sensors->servo.r_elbow.present_position = joint.position;
                raw_sensors->servo.r_elbow.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_elbow_pitch") {
                raw_sensors->servo.l_elbow.present_position = joint.position;
                raw_sensors->servo.l_elbow.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_hip_yaw") {
                raw_sensors->servo.r_hip_yaw.present_position = joint.position;
                raw_sensors->servo.r_hip_yaw.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_hip_yaw") {
                raw_sensors->servo.l_hip_yaw.present_position = joint.position;
                raw_sensors->servo.l_hip_yaw.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_hip_roll") {
                raw_sensors->servo.r_hip_roll.present_position = joint.position;
                raw_sensors->servo.r_hip_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_hip_roll") {
                raw_sensors->servo.l_hip_roll.present_position = joint.position;
                raw_sensors->servo.l_hip_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_hip_pitch") {
                raw_sensors->servo.r_hip_pitch.present_position = joint.position;
                raw_sensors->servo.r_hip_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_hip_pitch") {
                raw_sensors->servo.l_hip_pitch.present_position = joint.position;
                raw_sensors->servo.l_hip_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_knee_pitch") {
                raw_sensors->servo.r_knee.present_position = joint.position;
                raw_sensors->servo.r_knee.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_knee_pitch") {
                raw_sensors->servo.l_knee.present_position = joint.position;
                raw_sensors->servo.l_knee.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_ankle_pitch") {
                raw_sensors->servo.r_ankle_pitch.present_position = joint.position;
                raw_sensors->servo.r_ankle_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_ankle_pitch") {
                raw_sensors->servo.l_ankle_pitch.present_position = joint.position;
                raw_sensors->servo.l_ankle_pitch.present_velocity = joint.velocity;
            }
            else if (joint.name == "right_ankle_roll") {
                raw_sensors->servo.r_ankle_roll.present_position = joint.position;
                raw_sensors->servo.r_ankle_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "left_ankle_roll") {
                raw_sensors->servo.l_ankle_roll.present_position = joint.position;
                raw_sensors->servo.l_ankle_roll.present_velocity = joint.velocity;
            }
            else if (joint.name == "neck_yaw") {
                raw_sensors->servo.neck_yaw.present_position = joint.position;
                raw_sensors->servo.neck_yaw.present_velocity = joint.velocity;
            }
            else if (joint.name == "head_pitch") {
                raw_sensors->servo.head_pitch.present_position = joint.position;
                raw_sensors->servo.head_pitch.present_velocity = joint.velocity;
            }
        };

        for (const auto& joint : response.joint_states) {
            set_servo(joint);
        }

        emit(std::move(raw_sensors));
    }

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();
            const auto mode = config["mode"].as<std::string>("local");
            cfg.mode        = (mode == "remote") ? Mode::REMOTE : Mode::LOCAL;
            cfg.remote_host = config["remote_host"].as<std::string>(cfg.remote_host);
            cfg.remote_port = config["remote_port"].as<std::string>(cfg.remote_port);

            if (cfg.mode == Mode::REMOTE) {
                log<INFO>("Mujoco running in remote mode using bridge at", cfg.remote_host + ":" + cfg.remote_port);
                return;
            }

            // load and compile
            log<INFO>("Loading MuJoCo world model from:", cfg.world_path);
            char error[1000] = "Could not load binary model";
            if (std::strlen(cfg.world_path.c_str()) > 4
                && !std::strcmp(cfg.world_path.c_str() + std::strlen(cfg.world_path.c_str()) - 4, ".mjb")) {
                m = mj_loadModel(cfg.world_path.c_str(), 0);
            }
            else {
                log<INFO>("Loading world from XML");
                m = mj_loadXML(cfg.world_path.c_str(), 0, error, 1000);
            }
            if (!m) {
                mju_error("Load model error: %s", error);
            }
            log<INFO>("Scene loaded successfully");

            // make data, run one computation to initialize all fields
            d = mj_makeData(m);
            mj_forward(m, d);

            // Set initial joint positions
            auto initial_positions = config["initial_joint_positions"].as<std::map<std::string, double>>();
            d->qpos[0]             = config["initial_floating_base"]["pos_x"].as<double>();
            d->qpos[1]             = config["initial_floating_base"]["pos_y"].as<double>();
            d->qpos[2]             = config["initial_floating_base"]["pos_z"].as<double>();
            d->qpos[3]             = config["initial_floating_base"]["quat_w"].as<double>();
            d->qpos[4]             = config["initial_floating_base"]["quat_x"].as<double>();
            d->qpos[5]             = config["initial_floating_base"]["quat_y"].as<double>();
            d->qpos[6]             = config["initial_floating_base"]["quat_z"].as<double>();
            for (auto& joint : initial_positions) {
                int id = mj_name2id(m, mjOBJ_JOINT, joint.first.c_str());
                log<INFO>("Joint name:", joint.first, "ID:", id);
                if (id != -1) {
                    d->qpos[m->jnt_qposadr[id]]            = joint.second;
                    servo_state[joint.first].id            = id;
                    servo_state[joint.first].servo_name    = joint.first;
                    servo_state[joint.first].goal_position = joint.second;
                    servo_state[joint.first].p_gain        = 50;
                    servo_state[joint.first].d_gain        = 0;
                }
                else {
                    log<WARN>("Joint name not found in model:", joint.first);
                }
            }
        });

        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH, Sync<ServoState>>().then(
            "Simulator Update Loop",
            [this] {
                double sim_delta = 1.0 / UPDATE_FREQUENCY;

                if (cfg.mode == Mode::REMOTE) {
                    const auto now = NUClear::clock::now();
                    const auto dt  = std::chrono::duration_cast<std::chrono::microseconds>(now - current_real_time);
                    // Drive remote sim time from real elapsed wall-time to avoid slow-motion when this loop runs below
                    // target frequency.
                    sim_delta         = std::max(1e-4, std::min(0.05, dt.count() / 1e6));
                    current_real_time = now;

                    if (!connect_remote_bridge()) {
                        return;
                    }

                    SimulationRequest request;
                    request.sequence_id = ++remote_sequence_id;
                    request.step_dt     = sim_delta;
                    request.world_path  = cfg.world_path;
                    request.reset       = false;

                    for (const auto& servo : servo_state) {
                        request.joint_commands.push_back(JointCommand(servo.second.servo_name,
                                                                      servo.second.goal_position,
                                                                      servo.second.p_gain,
                                                                      servo.second.torque));
                    }

                    if (!send_remote_request(request)) {
                        close(remote_fd);
                        remote_fd        = -1;
                        remote_connected = false;
                        return;
                    }

                    SimulationResponse response;
                    if (!read_remote_response(response)) {
                        log<WARN>("Lost connection to MuJoCo bridge, reconnecting on next tick");
                        close(remote_fd);
                        remote_fd        = -1;
                        remote_connected = false;
                        return;
                    }

                    if (!response.ok) {
                        log<WARN>("MuJoCo bridge reported error:", response.error);
                        return;
                    }

                    emit_remote_raw_sensors(response);
                    return;
                }

                double next_sim_step = d->time + sim_delta;
                while (d->time < next_sim_step) {
                    // ctrl
                    for (auto servo : servo_state) {
                        int joint_id = mj_name2id(m, mjOBJ_JOINT, servo.second.servo_name.c_str());
                        if (joint_id == -1) {
                            log<WARN>("Joint name not found:", servo.second.servo_name);
                            continue;
                        }
                        int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, servo.second.servo_name.c_str());
                        if (actuator_id == -1) {
                            log<WARN>("Actuator not found for joint:", servo.second.servo_name);
                            continue;
                        }
                        // position control
                        d->ctrl[actuator_id] = servo.second.goal_position;
                        log<DEBUG>("Joint:",
                                   servo.second.servo_name,
                                   "Goal:",
                                   servo.second.goal_position,
                                   "Current:",
                                   d->qpos[m->jnt_qposadr[joint_id]],
                                   "Control:",
                                   d->ctrl[actuator_id],
                                   "P Gain:",
                                   servo.second.p_gain);
                        emit(graph(servo.second.servo_name + " goal position", servo.second.goal_position));
                        emit(graph(servo.second.servo_name + " current position", d->qpos[m->jnt_qposadr[joint_id]]));
                        emit(graph(servo.second.servo_name + " control", d->ctrl[actuator_id]));
                    }

                    // advance simulation
                    mj_step(m, d);
                }

                // sensors
                auto raw_sensors = std::make_unique<RawSensors>();

                // accelerometer
                int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "accelerometer");
                if (accel_sensor_id != -1) {
                    int adr = m->sensor_adr[accel_sensor_id];
                    int dim = m->sensor_dim[accel_sensor_id];
                    if (dim == 3) {
                        raw_sensors->accelerometer.x() = d->sensordata[adr + 0];  // X
                        raw_sensors->accelerometer.y() = d->sensordata[adr + 1];  // Y
                        raw_sensors->accelerometer.z() = d->sensordata[adr + 2];  // Z
                    }
                    else {
                        log<WARN>("Accelerometer sensor dimension not 3, got:", dim);
                    }
                }
                else {
                    log<WARN>("Accelerometer sensor not found");
                }

                // gyroscope
                int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "gyro");
                if (gyro_sensor_id != -1) {
                    int adr = m->sensor_adr[gyro_sensor_id];
                    int dim = m->sensor_dim[gyro_sensor_id];
                    if (dim == 3) {
                        raw_sensors->gyroscope.x() = d->sensordata[adr + 0];  // X
                        raw_sensors->gyroscope.y() = d->sensordata[adr + 1];  // Y
                        raw_sensors->gyroscope.z() = d->sensordata[adr + 2];  // Z
                    }
                    else {
                        log<WARN>("Gyroscope sensor dimension not 3, got:", dim);
                    }
                }
                else {
                    log<WARN>("Gyroscope sensor not found");
                }

                // joint encoders
                auto get_sensor_data = [&](const char* sensor_name) -> double {
                    int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name);
                    if (sensor_id == -1) {
                        log<WARN>("Sensor not found:", sensor_name);
                        return 0.0;
                    }
                    int adr = m->sensor_adr[sensor_id];
                    int dim = m->sensor_dim[sensor_id];
                    if (dim != 1) {
                        log<WARN>("Sensor dimension not 1:", sensor_name, "dim:", dim);
                        return 0.0;
                    }
                    return d->sensordata[adr];
                };

                // Joint positions
                raw_sensors->servo.r_shoulder_pitch.present_position = get_sensor_data("right_shoulder_pitch_pos");
                raw_sensors->servo.l_shoulder_pitch.present_position = get_sensor_data("left_shoulder_pitch_pos");
                raw_sensors->servo.r_shoulder_roll.present_position  = get_sensor_data("right_shoulder_roll_pos");
                raw_sensors->servo.l_shoulder_roll.present_position  = get_sensor_data("left_shoulder_roll_pos");
                raw_sensors->servo.r_elbow.present_position          = get_sensor_data("right_elbow_pitch_pos");
                raw_sensors->servo.l_elbow.present_position          = get_sensor_data("left_elbow_pitch_pos");
                raw_sensors->servo.r_hip_yaw.present_position        = get_sensor_data("right_hip_yaw_pos");
                raw_sensors->servo.l_hip_yaw.present_position        = get_sensor_data("left_hip_yaw_pos");
                raw_sensors->servo.r_hip_roll.present_position       = get_sensor_data("right_hip_roll_pos");
                raw_sensors->servo.l_hip_roll.present_position       = get_sensor_data("left_hip_roll_pos");
                raw_sensors->servo.r_hip_pitch.present_position      = get_sensor_data("right_hip_pitch_pos");
                raw_sensors->servo.l_hip_pitch.present_position      = get_sensor_data("left_hip_pitch_pos");
                raw_sensors->servo.r_knee.present_position           = get_sensor_data("right_knee_pitch_pos");
                raw_sensors->servo.l_knee.present_position           = get_sensor_data("left_knee_pitch_pos");
                raw_sensors->servo.r_ankle_pitch.present_position    = get_sensor_data("right_ankle_pitch_pos");
                raw_sensors->servo.l_ankle_pitch.present_position    = get_sensor_data("left_ankle_pitch_pos");
                raw_sensors->servo.r_ankle_roll.present_position     = get_sensor_data("right_ankle_roll_pos");
                raw_sensors->servo.l_ankle_roll.present_position     = get_sensor_data("left_ankle_roll_pos");
                raw_sensors->servo.neck_yaw.present_position         = get_sensor_data("neck_yaw_pos");
                raw_sensors->servo.head_pitch.present_position       = get_sensor_data("head_pitch_pos");

                // Joint velocities
                raw_sensors->servo.r_shoulder_pitch.present_velocity = get_sensor_data("right_shoulder_pitch_vel");
                raw_sensors->servo.l_shoulder_pitch.present_velocity = get_sensor_data("left_shoulder_pitch_vel");
                raw_sensors->servo.r_shoulder_roll.present_velocity  = get_sensor_data("right_shoulder_roll_vel");
                raw_sensors->servo.l_shoulder_roll.present_velocity  = get_sensor_data("left_shoulder_roll_vel");
                raw_sensors->servo.r_elbow.present_velocity          = get_sensor_data("right_elbow_pitch_vel");
                raw_sensors->servo.l_elbow.present_velocity          = get_sensor_data("left_elbow_pitch_vel");
                raw_sensors->servo.r_hip_yaw.present_velocity        = get_sensor_data("right_hip_yaw_vel");
                raw_sensors->servo.l_hip_yaw.present_velocity        = get_sensor_data("left_hip_yaw_vel");
                raw_sensors->servo.r_hip_roll.present_velocity       = get_sensor_data("right_hip_roll_vel");
                raw_sensors->servo.l_hip_roll.present_velocity       = get_sensor_data("left_hip_roll_vel");
                raw_sensors->servo.r_hip_pitch.present_velocity      = get_sensor_data("right_hip_pitch_vel");
                raw_sensors->servo.l_hip_pitch.present_velocity      = get_sensor_data("left_hip_pitch_vel");
                raw_sensors->servo.r_knee.present_velocity           = get_sensor_data("right_knee_pitch_vel");
                raw_sensors->servo.l_knee.present_velocity           = get_sensor_data("left_knee_pitch_vel");
                raw_sensors->servo.r_ankle_pitch.present_velocity    = get_sensor_data("right_ankle_pitch_vel");
                raw_sensors->servo.l_ankle_pitch.present_velocity    = get_sensor_data("left_ankle_pitch_vel");
                raw_sensors->servo.r_ankle_roll.present_velocity     = get_sensor_data("right_ankle_roll_vel");
                raw_sensors->servo.l_ankle_roll.present_velocity     = get_sensor_data("left_ankle_roll_vel");
                raw_sensors->servo.neck_yaw.present_velocity         = get_sensor_data("neck_yaw_vel");
                raw_sensors->servo.head_pitch.present_velocity       = get_sensor_data("head_pitch_vel");

                emit(std::move(raw_sensors));

                // render in main thread, not in simulation thread
                if (cfg.mode == Mode::LOCAL) {
                    emit(std::make_unique<Render>());
                }
            });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, Sync<ServoState>, Priority::HIGH>().then([this](const ServoTargets& targets) {
            // Loop through each of our commands
            for (const auto& target : targets.targets) {
                servo_state[target.name].id            = target.id;
                servo_state[target.name].servo_name    = target.name;
                servo_state[target.name].p_gain        = target.gain;
                servo_state[target.name].goal_position = target.position;
            }
        });

        on<Trigger<ServoTarget>, Priority::HIGH>().then([this](const ServoTarget& target) {
            auto targets = std::make_unique<ServoTargets>();
            targets->targets.emplace_back(target);
            emit<Scope::INLINE>(targets);
        });

        on<Trigger<Render>, Single, MainThread>().then("Render Mujoco", [this] {
            if (cfg.mode != Mode::LOCAL) {
                return;
            }

            static bool initialized = false;
            if (!initialized) {
                // init GLFW
                XInitThreads();
                if (!glfwInit()) {
                    mju_error("Could not initialize GLFW");
                }

                glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
                glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
                glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
                glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
                glfwWindowHint(GLFW_X11_XCB_VULKAN_SURFACE, 0);

                log<INFO>("Creating window");

                // Create window with explicit sharing context
                window = glfwCreateWindow(W, H, "Mujoco Window", NULL, NULL);
                if (!window) {
                    const char* description;
                    int code = glfwGetError(&description);
                    log<ERROR>("Could not create GLFW window: ", description, " (error code: ", code, ")");
                    glfwTerminate();
                    return;
                }

                // Make context current in the main thread
                glfwMakeContextCurrent(window);
                glfwSwapInterval(1);
                glfwShowWindow(window);

                // initialize visualization data structures
                mjv_defaultCamera(&cam);
                mjv_defaultOption(&opt);
                mjv_defaultScene(&scn);
                mjr_defaultContext(&con);

                // create scene and context
                mjv_makeScene(m, &scn, 2000);
                mjr_makeContext(m, &con, 200);

                // Set up tracking camera
                int camera_id = mj_name2id(m, mjOBJ_CAMERA, "track");
                if (camera_id != -1) {
                    cam.type       = mjCAMERA_FIXED;
                    cam.fixedcamid = camera_id;
                }
                else {
                    log<WARN>("Could not find tracking camera, using default free camera");
                    mjv_defaultFreeCamera(m, &cam);
                }

                // set rendering to visible window buffer for local GUI mode
                mjr_setBuffer(mjFB_WINDOW, &con);
                if (con.currentBuffer != mjFB_WINDOW) {
                    log<WARN>("Window framebuffer not supported, falling back to offscreen buffer");
                    mjr_setBuffer(mjFB_OFFSCREEN, &con);
                }

                // get current framebuffer size
                glfwGetFramebufferSize(window, &W, &H);
                viewport = {0, 0, W, H};

                // allocate rgb and depth buffers
                rgb   = (unsigned char*) std::malloc(3 * W * H);
                depth = (float*) std::malloc(sizeof(float) * W * H);
                if (!rgb) {
                    log<ERROR>("Could not allocate buffers");
                }

                initialized = true;
            }

            glfwGetFramebufferSize(window, &W, &H);
            viewport = {0, 0, W, H};
            glfwPollEvents();
            render();
        });

        on<Shutdown>().then("Shutdown Mujoco", [this] {
            if (remote_fd >= 0) {
                close(remote_fd);
                remote_fd = -1;
            }

            if (cfg.mode == Mode::REMOTE) {
                return;
            }

            // free buffers
            std::free(rgb);
            std::free(depth);
            mj_deleteData(d);
            mj_deleteModel(m);
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
            glfwTerminate();
        });
    }

}  // namespace module::platform
