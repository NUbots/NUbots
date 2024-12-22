#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::platform {

    using extension::Configuration;

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::platform::get_raw_servo;

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
            // init GLFW
            if (!glfwInit()) {
                mju_error("Could not initialize GLFW");
            }

            // create invisible window, single-buffered
            glfwWindowHint(GLFW_VISIBLE, 0);
            glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
            window = glfwCreateWindow(W, H, "Invisible window", NULL, NULL);
            if (!window) {
                mju_error("Could not create GLFW window");
            }

            // make context current
            glfwMakeContextCurrent(window);

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

            // get initial joint positions from configuration
            auto initial_positions = config["initial_joint_positions"].as<std::map<std::string, double>>();

            // set initial joint positions in qpos and servo state
            for (auto& joint : initial_positions) {
                int id = mj_name2id(m, mjOBJ_JOINT, joint.first.c_str());
                log<NUClear::INFO>("Joint name:", joint.first, "ID:", id);
                if (id != -1) {
                    d->qpos[m->jnt_qposadr[id]]            = joint.second;
                    servo_state[joint.first].id            = id;
                    servo_state[joint.first].name          = joint.first;
                    servo_state[joint.first].goal_position = joint.second;
                    servo_state[joint.first].p_gain        = 5;
                    servo_state[joint.first].d_gain        = 0;
                }
                else {
                    log<NUClear::WARN>("Joint name not found in model:", joint.first);
                }
            }

            // initialize visualization data structures
            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjv_defaultScene(&scn);
            mjr_defaultContext(&con);

            // create scene and context
            mjv_makeScene(m, &scn, 2000);
            mjr_makeContext(m, &con, 200);

            // default free camera
            mjv_defaultFreeCamera(m, &cam);

            // set rendering to offscreen buffer
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            if (con.currentBuffer != mjFB_OFFSCREEN) {
                log<NUClear::WARN>("Offscreen rendering not supported, using default/window framebuffer");
            }

            // get size of active renderbuffer
            viewport = mjr_maxViewport(&con);
            W        = viewport.width;
            H        = viewport.height;

            // allocate rgb and depth buffers
            rgb   = (unsigned char*) std::malloc(3 * W * H);
            depth = (float*) std::malloc(sizeof(float) * W * H);
            if (!rgb) {
                log<NUClear::ERROR>("Could not allocate buffers");
            }

            // render
            emit(std::make_unique<Render>());
        });

        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Priority::HIGH, Single, Sync<ServoState>>().then(
            "Simulator Update Loop",
            [this] {
                double next_sim_step = d->time + 1.0 / UPDATE_FREQUENCY;
                while (d->time < next_sim_step) {
                    // ctrl

                    for (auto servo : servo_state) {
                        int joint_id = mj_name2id(m, mjOBJ_JOINT, servo.second.name.c_str());
                        if (joint_id == -1) {
                            log<NUClear::WARN>("Joint name not found:", servo.second.name);
                            continue;
                        }
                        int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, servo.second.name.c_str());
                        if (actuator_id == -1) {
                            log<NUClear::WARN>("Actuator not found for joint:", servo.second.name);
                            continue;
                        }
                        // PD control
                        d->ctrl[actuator_id] =
                            servo.second.p_gain * (servo.second.goal_position - d->qpos[m->jnt_qposadr[joint_id]]);
                        // + servo.second.d_gain * (0.0 - d->qvel[m->jnt_dofadr[joint_id]]);
                        log<NUClear::DEBUG>("Joint:",
                                            servo.second.name,
                                            "Goal:",
                                            servo.second.goal_position,
                                            "Current:",
                                            d->qpos[m->jnt_qposadr[joint_id]],
                                            "Control:",
                                            d->ctrl[actuator_id],
                                            "P Gain:",
                                            servo.second.p_gain);
                    }

                    // advance simulation
                    mj_step(m, d);
                }

                // sensors
                auto raw_sensors = std::make_unique<RawSensors>();

                // accelerometer
                int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "accelerometer");
                if (accel_sensor_id != -1) {
                    raw_sensors->accelerometer.x() = d->sensordata[accel_sensor_id + 0];  // X
                    raw_sensors->accelerometer.y() = d->sensordata[accel_sensor_id + 1];  // Y
                    raw_sensors->accelerometer.z() = d->sensordata[accel_sensor_id + 2];  // Z
                }

                // gyroscope
                int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "gyro");
                if (gyro_sensor_id != -1) {
                    raw_sensors->gyroscope.x() = d->sensordata[gyro_sensor_id + 0];  // X
                    raw_sensors->gyroscope.y() = d->sensordata[gyro_sensor_id + 1];  // Y
                    raw_sensors->gyroscope.z() = d->sensordata[gyro_sensor_id + 2];  // Z
                }

                // joint encoders
                raw_sensors->servo.r_shoulder_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_pitch")];
                raw_sensors->servo.l_shoulder_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_pitch")];
                raw_sensors->servo.r_shoulder_roll.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_roll")];
                raw_sensors->servo.l_shoulder_roll.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_roll")];
                raw_sensors->servo.r_elbow.present_position   = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_elbow")];
                raw_sensors->servo.l_elbow.present_position   = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_elbow")];
                raw_sensors->servo.r_hip_yaw.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_hip_yaw")];
                raw_sensors->servo.l_hip_yaw.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_hip_yaw")];
                raw_sensors->servo.r_hip_roll.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_hip_roll")];
                raw_sensors->servo.l_hip_roll.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_hip_roll")];
                raw_sensors->servo.r_hip_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_hip_pitch")];
                raw_sensors->servo.l_hip_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_hip_pitch")];
                raw_sensors->servo.r_knee.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_knee")];
                raw_sensors->servo.l_knee.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_knee")];
                raw_sensors->servo.r_ankle_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_ankle_pitch")];
                raw_sensors->servo.l_ankle_pitch.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_ankle_pitch")];
                raw_sensors->servo.r_ankle_roll.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "right_ankle_roll")];
                raw_sensors->servo.l_ankle_roll.present_position =
                    d->act[mj_name2id(m, mjOBJ_ACTUATOR, "left_ankle_roll")];
                raw_sensors->servo.neck_yaw.present_position   = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "neck_yaw")];
                raw_sensors->servo.neck_pitch.present_position = d->act[mj_name2id(m, mjOBJ_ACTUATOR, "neck_pitch")];

                emit(std::move(raw_sensors));

                // render
                // render();
                emit(std::make_unique<Render>());
            });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, With<RawSensors>, Sync<ServoState>>().then(
            [this](const ServoTargets& targets, const RawSensors& sensors) {
                // Loop through each of our commands
                for (const auto& target : targets.targets) {
                    servo_state[target.name].id            = target.id;
                    servo_state[target.name].name          = target.name;
                    servo_state[target.name].p_gain        = target.gain;
                    servo_state[target.name].goal_position = target.position;
                }
            });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& target) {
            auto targets = std::make_unique<ServoTargets>();
            targets->targets.emplace_back(target);
            emit<Scope::DIRECT>(targets);
        });

        on<Trigger<Render>, Single>().then("Render Mujoco", [this] { render(); });

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
