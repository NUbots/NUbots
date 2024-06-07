#include "Mujoco.hpp"

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

            // Update timestep
            // m->opt.timestep = 1.0 / UPDATE_FREQUENCY;

            // get initial joint positions from configuration
            auto initial_positions = config["initial_joint_positions"].as<std::map<std::string, double>>();

            Eigen::Vector3d pos  = Eigen::Vector3d(config["initial_floating_base"]["pos"].as<Expression>());
            Eigen::VectorXd quat = Eigen::VectorXd(config["initial_floating_base"]["quat"].as<Expression>());
            d->qpos[0]           = pos.x();
            d->qpos[1]           = pos.y();
            d->qpos[2]           = pos.z();
            d->qpos[3]           = quat(0);
            d->qpos[4]           = quat(1);
            d->qpos[5]           = quat(2);
            d->qpos[6]           = quat(3);
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

            current_real_time = NUClear::clock::now();

            // Pause RTF
            // emit<Scope::DIRECT>(
            //     std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
            //                                                    0,
            //                                                    NUClear::message::TimeTravel::Action::RELATIVE));
        });

        on<Startup>().then("Mujoco Startup", [this] {
            // emit<Scope::DIRECT>(std::make_unique<NUClear::message::TimeTravel>(
            //     NUClear::clock::now() + std::chrono::milliseconds(1000 / UPDATE_FREQUENCY + 10),
            //     0,
            //     NUClear::message::TimeTravel::Action::ABSOLUTE));
        });

        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH, Sync<ServoState>>().then(
            "Simulator Update Loop",
            [this] {
                real_delta =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - current_real_time)
                        .count()
                    / 1e9;
                current_real_time = NUClear::clock::now();

                double next_sim_step = d->time + real_delta;

                log<NUClear::DEBUG>("Real Delta:", real_delta);
                log<NUClear::DEBUG>("d->time:", d->time);

                auto mujoco = std::make_unique<message::output::Mujoco>();
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
                        // P control
                        d->ctrl[actuator_id]                 = servo.second.goal_position;
                        mujoco->servo_map[servo.second.name] = d->qpos[m->jnt_qposadr[joint_id]];
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
                raw_sensors->servo.r_shoulder_pitch.goal_position = servo_state["right_shoulder_pitch"].goal_position;
                raw_sensors->servo.r_shoulder_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_shoulder_pitch")]];
                log<NUClear::DEBUG>("Right Shoulder Pitch:", raw_sensors->servo.r_shoulder_pitch.present_position);
                raw_sensors->servo.l_shoulder_pitch.goal_position = servo_state["left_shoulder_pitch"].goal_position;
                raw_sensors->servo.l_shoulder_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_shoulder_pitch")]];
                raw_sensors->servo.r_shoulder_roll.goal_position = servo_state["right_shoulder_roll"].goal_position;
                raw_sensors->servo.r_shoulder_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_shoulder_roll")]];
                raw_sensors->servo.l_shoulder_roll.goal_position = servo_state["left_shoulder_roll"].goal_position;
                raw_sensors->servo.l_shoulder_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_shoulder_roll")]];
                raw_sensors->servo.r_elbow.goal_position = servo_state["right_elbow_pitch"].goal_position;
                raw_sensors->servo.r_elbow.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_elbow_pitch")]];
                raw_sensors->servo.l_elbow.goal_position = servo_state["left_elbow_pitch"].goal_position;
                raw_sensors->servo.l_elbow.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_elbow_pitch")]];
                raw_sensors->servo.r_hip_yaw.goal_position = servo_state["right_hip_yaw"].goal_position;
                raw_sensors->servo.r_hip_yaw.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_hip_yaw")]];
                raw_sensors->servo.l_hip_yaw.goal_position = servo_state["left_hip_yaw"].goal_position;
                raw_sensors->servo.l_hip_yaw.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_hip_yaw")]];
                raw_sensors->servo.r_hip_roll.goal_position = servo_state["right_hip_roll"].goal_position;
                raw_sensors->servo.r_hip_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_hip_roll [hip]")]];
                raw_sensors->servo.l_hip_roll.goal_position = servo_state["left_hip_roll"].goal_position;
                raw_sensors->servo.l_hip_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_hip_roll [hip]")]];
                raw_sensors->servo.r_hip_pitch.goal_position = servo_state["right_hip_pitch"].goal_position;
                raw_sensors->servo.r_hip_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_hip_pitch")]];
                raw_sensors->servo.l_hip_pitch.goal_position = servo_state["left_hip_pitch"].goal_position;
                raw_sensors->servo.l_hip_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_hip_pitch")]];
                raw_sensors->servo.r_knee.goal_position = servo_state["right_knee_pitch"].goal_position;
                raw_sensors->servo.r_knee.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_knee_pitch")]];
                raw_sensors->servo.l_knee.goal_position = servo_state["left_knee_pitch"].goal_position;
                raw_sensors->servo.l_knee.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_knee_pitch")]];
                raw_sensors->servo.r_ankle_pitch.goal_position = servo_state["right_ankle_pitch"].goal_position;
                raw_sensors->servo.r_ankle_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_ankle_pitch")]];
                raw_sensors->servo.l_ankle_pitch.goal_position = servo_state["left_ankle_pitch"].goal_position;
                raw_sensors->servo.l_ankle_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_ankle_pitch")]];
                raw_sensors->servo.r_ankle_roll.goal_position = servo_state["right_ankle_roll"].goal_position;
                raw_sensors->servo.r_ankle_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "right_ankle_roll")]];
                raw_sensors->servo.l_ankle_roll.goal_position = servo_state["left_ankle_roll"].goal_position;
                raw_sensors->servo.l_ankle_roll.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "left_ankle_roll")]];
                raw_sensors->servo.neck_yaw.goal_position = servo_state["neck_yaw"].goal_position;
                raw_sensors->servo.neck_yaw.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "neck_yaw")]];
                raw_sensors->servo.neck_pitch.goal_position = servo_state["head_pitch"].goal_position;
                raw_sensors->servo.neck_pitch.present_position =
                    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "head_pitch")]];


                emit(raw_sensors);

                // render
                // render();


                emit(std::make_unique<Render>());

                Eigen::Isometry3d Hwt = Eigen::Isometry3d::Identity();
                Hwt.translation()     = Eigen::Vector3d(d->qpos[0], d->qpos[1], d->qpos[2]);
                Hwt.linear() = Eigen::Quaterniond(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]).toRotationMatrix();
                mujoco->Htw  = Hwt.inverse();
                emit(mujoco);

                // // Save our previous deltas
                // const uint32_t prev_sim_delta  = sim_delta;
                // const uint64_t prev_real_delta = real_delta;

                // // Update our current deltas
                // real_delta =
                //     std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - current_real_time)
                //         .count()
                //     / 1e9;
                // sim_delta = d->time - current_sim_time;

                // // Calculate our custom rtf - the ratio of the past two sim deltas and the past two real time deltas,
                // // smoothed
                // const double ratio =
                //     static_cast<double>(sim_delta + prev_sim_delta) / static_cast<double>(real_delta +
                //     prev_real_delta);

                // // Exponential filter to do the smoothing
                // rtf = rtf * clock_smoothing + (1.0 - clock_smoothing) * ratio;
                // log<NUClear::DEBUG>("RTF:", rtf);
                // log<NUClear::DEBUG>("Sim Delta:", sim_delta, "Real Delta:", real_delta);
                // NUClear::clock::set_clock(NUClear::clock::now(), rtf);

                // // Update our current times
                // current_sim_time  = d->time;
                // current_real_time = NUClear::clock::now();


                // emit<Scope::DIRECT>(std::make_unique<NUClear::message::TimeTravel>(
                //     NUClear::clock::now() + std::chrono::milliseconds(1000 / UPDATE_FREQUENCY + 10),
                //     0,
                //     NUClear::message::TimeTravel::Action::ABSOLUTE));
            });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, With<RawSensors>, Sync<ServoState>, Priority::HIGH>().then(
            [this](const ServoTargets& targets, const RawSensors& sensors) {
                // Loop through each of our commands
                for (const auto& target : targets.targets) {
                    servo_state[target.name].id            = target.id;
                    servo_state[target.name].name          = target.name;
                    servo_state[target.name].p_gain        = target.gain;
                    servo_state[target.name].goal_position = target.position;
                }
            });

        on<Trigger<ServoTarget>, Priority::HIGH>().then([this](const ServoTarget& target) {
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
