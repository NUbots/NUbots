#include "Mujoco.hpp"

#include <X11/Xlib.h>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/output/Mujoco.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

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

    std::map<uint32_t, std::string> id_to_joint_name = {
        {0, "right_shoulder_pitch"}, {1, "left_shoulder_pitch"}, {2, "right_shoulder_roll"}, {3, "left_shoulder_roll"},
        {4, "right_elbow_pitch"},    {5, "left_elbow_pitch"},    {6, "right_hip_yaw"},       {7, "left_hip_yaw"},
        {8, "right_hip_roll [hip]"}, {9, "left_hip_roll [hip]"}, {10, "right_hip_pitch"},    {11, "left_hip_pitch"},
        {12, "right_knee_pitch"},    {13, "left_knee_pitch"},    {14, "right_ankle_pitch"},  {15, "left_ankle_pitch"},
        {16, "right_ankle_roll"},    {17, "left_ankle_roll"},    {18, "neck_yaw"},           {19, "head_pitch"}};

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();

            // load and compile
            char error[1000] = "Could not load binary model";
            if (std::strlen(cfg.world_path.c_str()) > 4
                && !std::strcmp(cfg.world_path.c_str() + std::strlen(cfg.world_path.c_str()) - 4, ".mjb")) {
                m = mj_loadModel(cfg.world_path.c_str(), 0);
            }
            else {
                m = mj_loadXML(cfg.world_path.c_str(), 0, error, 1000);
            }
            if (!m) {
                mju_error("Load model error: %s", error);
            }

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
                double sim_delta     = 1.0 / UPDATE_FREQUENCY;
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
                raw_sensors->servo.neck_pitch.present_position       = get_sensor_data("head_pitch_pos");

                emit(std::move(raw_sensors));

                // render in main thread, not in simulation thread
                emit(std::make_unique<Render>());
            });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, With<RawSensors>, Sync<ServoState>, Priority::HIGH>().then(
            [this](const ServoTargets& targets, const RawSensors& sensors) {
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
                glfwWindowHint(GLFW_VISIBLE, 0);  // Make window invisible
                glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
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

                // set rendering to offscreen buffer
                mjr_setBuffer(mjFB_OFFSCREEN, &con);
                if (con.currentBuffer != mjFB_OFFSCREEN) {
                    log<WARN>("Offscreen rendering not supported, using default/window framebuffer");
                }

                // get size of active renderbuffer
                viewport = mjr_maxViewport(&con);
                W        = viewport.width;
                H        = viewport.height;

                // allocate rgb and depth buffers
                rgb   = (unsigned char*) std::malloc(3 * W * H);
                depth = (float*) std::malloc(sizeof(float) * W * H);
                if (!rgb) {
                    log<ERROR>("Could not allocate buffers");
                }

                initialized = true;
            }
            render();
        });

        on<Shutdown>().then("Shutdown Mujoco", [this] {
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
