/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Webots.hpp"

#include <chrono>
#include <fmt/format.h>
#include <string>

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/OptimisationCommand.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationRobotPosition.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"

// Include headers needed for TCP connection
extern "C" {
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/ioctl.h>  /* definition of ioctl and FIONREAD */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
}

namespace module::platform {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::input::Image;
    using message::input::Sensors;
    using message::platform::RawSensors;
    using message::platform::ResetWebotsServos;

    using message::platform::webots::ActuatorRequests;
    using message::platform::webots::Message;
    using message::platform::webots::MotorPID;
    using message::platform::webots::MotorPosition;
    using message::platform::webots::MotorVelocity;
    using message::platform::webots::OdometryGroundTruth;
    using message::platform::webots::SensorMeasurements;
    using message::platform::webots::SensorTimeStep;
    using message::platform::webots::VisionGroundTruth;
    using message::support::optimisation::OptimisationCommand;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationRobotPosition;
    using message::support::optimisation::OptimisationTimeUpdate;

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::platform::get_raw_servo;
    using utility::support::Expression;
    using utility::vision::fourcc;

    // Converts the NUgus.proto servo name to the equivalent RawSensor.proto name
    [[nodiscard]] RawSensors::Servo& translate_servo_id(const std::string& name, RawSensors::Servos& servos) {

        // clang-format off
        // Left ankle
        if (name == "left_ankle_roll_sensor") { return servos.l_ankle_roll; }
        if (name == "left_ankle_pitch_sensor") { return servos.l_ankle_pitch; }
        // Right ankle
        if (name == "right_ankle_roll_sensor") { return servos.r_ankle_roll; }
        if (name == "right_ankle_pitch_sensor") { return servos.r_ankle_pitch; }
        // Knees
        if (name == "right_knee_pitch_sensor") { return servos.r_knee; }
        if (name == "left_knee_pitch_sensor") { return servos.l_knee; }
        // Left hip
        if (name == "left_hip_roll_sensor") { return servos.l_hip_roll; }
        if (name == "left_hip_pitch_sensor") { return servos.l_hip_pitch; }
        if (name == "left_hip_yaw_sensor") { return servos.l_hip_yaw; }
        // Right hip
        if (name == "right_hip_roll_sensor") { return servos.r_hip_roll; }
        if (name == "right_hip_pitch_sensor") { return servos.r_hip_pitch; }
        if (name == "right_hip_yaw_sensor") { return servos.r_hip_yaw; }
        // Elbows
        if (name == "left_elbow_pitch_sensor") { return servos.l_elbow; }
        if (name == "right_elbow_pitch_sensor") { return servos.r_elbow; }
        // Left shoulder
        if (name == "left_shoulder_roll_sensor") { return servos.l_shoulder_roll; }
        if (name == "left_shoulder_pitch_sensor") { return servos.l_shoulder_pitch; }
        // Right shoulder
        if (name == "right_shoulder_roll_sensor") { return servos.r_shoulder_roll; }
        if (name == "right_shoulder_pitch_sensor") { return servos.r_shoulder_pitch; }
        // Neck and head
        if (name == "neck_yaw_sensor") { return servos.head_pan; }
        if (name == "head_pitch_sensor") { return servos.head_tilt; }
        // clang-format on

        throw std::runtime_error(fmt::format("Unable to translate unknown NUgus.proto sensor name: {}", name));
    }

    [[nodiscard]] std::string translate_id_servo(const uint32_t& id) {
        switch (id) {
            case 0: return "right_shoulder_pitch [shoulder]";
            case 1: return "left_shoulder_pitch [shoulder]";
            case 2: return "right_shoulder_roll";
            case 3: return "left_shoulder_roll";
            case 4: return "right_elbow_pitch";
            case 5: return "left_elbow_pitch";
            case 6: return "right_hip_yaw";
            case 7: return "left_hip_yaw";
            case 8: return "right_hip_roll [hip]";
            case 9: return "left_hip_roll [hip]";
            case 10: return "right_hip_pitch";
            case 11: return "left_hip_pitch";
            case 12: return "right_knee_pitch";
            case 13: return "left_knee_pitch";
            case 14: return "right_ankle_pitch";
            case 15: return "left_ankle_pitch";
            case 16: return "right_ankle_roll";
            case 17: return "left_ankle_roll";
            case 18: return "neck_yaw";
            case 19: return "head_pitch";
        }

        throw std::runtime_error(fmt::format("Unable to translate unknown NUgus.proto servo id: {}", id));
    }

    [[nodiscard]] uint32_t translate_name_to_id(const std::string& name) {
        if (name == "right_shoulder_pitch_sensor") {
            return 0;
        }
        if (name == "left_shoulder_pitch_sensor") {
            return 1;
        }
        if (name == "right_shoulder_roll_sensor") {
            return 2;
        }
        if (name == "left_shoulder_roll_sensor") {
            return 3;
        }
        if (name == "right_elbow_pitch_sensor") {
            return 4;
        }
        if (name == "left_elbow_pitch_sensor") {
            return 5;
        }
        if (name == "right_hip_yaw_sensor") {
            return 6;
        }
        if (name == "left_hip_yaw_sensor") {
            return 7;
        }
        if (name == "right_hip_roll_sensor") {
            return 8;
        }
        if (name == "left_hip_roll_sensor") {
            return 9;
        }
        if (name == "right_hip_pitch_sensor") {
            return 10;
        }
        if (name == "left_hip_pitch_sensor") {
            return 11;
        }
        if (name == "right_knee_pitch_sensor") {
            return 12;
        }
        if (name == "left_knee_pitch_sensor") {
            return 13;
        }
        if (name == "right_ankle_pitch_sensor") {
            return 14;
        }
        if (name == "left_ankle_pitch_sensor") {
            return 15;
        }
        if (name == "right_ankle_roll_sensor") {
            return 16;
        }
        if (name == "left_ankle_roll_sensor") {
            return 17;
        }
        if (name == "neck_yaw_sensor") {
            return 18;
        }
        if (name == "head_pitch_sensor") {
            return 19;
        }

        throw std::runtime_error(fmt::format("Unable to translate unknown NUgus.proto servo name: {}", name));
    }

    [[nodiscard]] ActuatorRequests create_sensor_time_steps(const uint32_t& sensor_timestep,
                                                            const uint32_t& camera_timestep) {
        message::platform::webots::ActuatorRequests msg;

        msg.sensor_time_steps = {{"left_ankle_roll_sensor", sensor_timestep},
                                 {"left_ankle_pitch_sensor", sensor_timestep},
                                 {"right_ankle_roll_sensor", sensor_timestep},
                                 {"right_ankle_pitch_sensor", sensor_timestep},
                                 {"right_knee_pitch_sensor", sensor_timestep},
                                 {"left_knee_pitch_sensor", sensor_timestep},
                                 {"left_hip_roll_sensor", sensor_timestep},
                                 {"left_hip_pitch_sensor", sensor_timestep},
                                 {"left_hip_yaw_sensor", sensor_timestep},
                                 {"right_hip_roll_sensor", sensor_timestep},
                                 {"right_hip_pitch_sensor", sensor_timestep},
                                 {"right_hip_yaw_sensor", sensor_timestep},
                                 {"left_elbow_pitch_sensor", sensor_timestep},
                                 {"right_elbow_pitch_sensor", sensor_timestep},
                                 {"left_shoulder_roll_sensor", sensor_timestep},
                                 {"left_shoulder_pitch_sensor", sensor_timestep},
                                 {"right_shoulder_roll_sensor", sensor_timestep},
                                 {"right_shoulder_pitch_sensor", sensor_timestep},
                                 {"neck_yaw_sensor", sensor_timestep},
                                 {"head_pitch_sensor", sensor_timestep},
                                 {"accelerometer", sensor_timestep},
                                 {"gyroscope", sensor_timestep},
                                 //  {"right_camera", camera_timestep},
                                 {"left_camera", camera_timestep},
                                 {"right_touch_sensor_br", sensor_timestep},
                                 {"right_touch_sensor_bl", sensor_timestep},
                                 {"right_touch_sensor_fl", sensor_timestep},
                                 {"right_touch_sensor_fr", sensor_timestep},
                                 {"left_touch_sensor_br", sensor_timestep},
                                 {"left_touch_sensor_bl", sensor_timestep},
                                 {"left_touch_sensor_fl", sensor_timestep},
                                 {"left_touch_sensor_fr", sensor_timestep}};

        return msg;
    }

    int Webots::tcpip_connect() {
        // Hints for the connection type
        addrinfo hints{};
        memset(&hints, 0, sizeof(addrinfo));  // Defaults on what we do not explicitly set
        hints.ai_family   = AF_UNSPEC;        // IPv4 or IPv6
        hints.ai_socktype = SOCK_STREAM;      // TCP

        // Store the ip address information that we will connect to
        addrinfo* address = nullptr;

        const int error = getaddrinfo(server_address.c_str(), server_port.c_str(), &hints, &address);
        if (error != 0) {
            log<NUClear::ERROR>(fmt::format("Cannot resolve server name: {}. Error {}. Error code {}",
                                            server_address,
                                            gai_strerror(error),
                                            error));
            return -1;
        }

        // Loop through the linked list of potential options for connecting. In order of best to worst.
        for (addrinfo* addr_ptr = address; addr_ptr != nullptr; addr_ptr = addr_ptr->ai_next) {
            const int fd_temp = socket(addr_ptr->ai_family, addr_ptr->ai_socktype, addr_ptr->ai_protocol);

            if (fd_temp == -1) {
                // Bad fd
                continue;
            }
            if (connect(fd_temp, addr_ptr->ai_addr, addr_ptr->ai_addrlen) != -1) {
                // Connection successful
                freeaddrinfo(address);
                return fd_temp;
            }
            // Connection was not successful
            close(fd_temp);
        }

        // No connection was successful
        freeaddrinfo(address);
        log<NUClear::ERROR>(fmt::format("Cannot connect to server: {}:{}", server_address, server_port));
        return -1;
    }

    Webots::Webots(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Configuration>("Webots.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Webots.yaml
            time_step            = config["time_step"].as<int>();
            min_camera_time_step = config["min_camera_time_step"].as<int>();
            min_sensor_time_step = config["min_sensor_time_step"].as<int>();
            max_velocity_mx64    = config["max_velocity_mx64"].as<double>();
            max_velocity_mx106   = config["max_velocity_mx106"].as<double>();
            max_fsr_value        = config["max_fsr_value"].as<float>();

            log_level = config["log_level"].as<NUClear::LogLevel>();

            clock_smoothing = config["clock_smoothing"].as<double>();

            server_address = config["server_address"].as<std::string>();
            server_port    = config["port"].as<std::string>();

            on<Watchdog<Webots, 30, std::chrono::seconds>, Sync<Webots>>().then([this, config] {
                // We haven't received any messages lately
                log<NUClear::WARN>("Connection timed out. Attempting reconnect");
                setup_connection();
            });

            // Connect to the server
            setup_connection();
        });


        on<Configuration>("WebotsCameras").then([this](const Configuration& config) {
            // The camera's name is the filename of the config, with the .yaml stripped off
            const std::string name = config.fileName.stem();

            log<NUClear::INFO>(fmt::format("Connected to the webots {} camera", name));

            CameraContext context;
            context.name = name;
            context.id   = num_cameras++;

            // Compute Hpc, the transform from the camera to the head pitch space
            auto nugus_model = tinyrobotics::import_urdf<double, 20>(config["urdf_path"].as<std::string>());
            auto Hpc         = tinyrobotics::forward_kinematics<double, 20>(nugus_model,
                                                                    nugus_model.home_configuration(),
                                                                    std::string("left_camera"),
                                                                    std::string("head"));

            // Apply roll and pitch offsets
            double roll_offset  = config["roll_offset"].as<Expression>();
            double pitch_offset = config["pitch_offset"].as<Expression>();
            context.Hpc         = Eigen::AngleAxisd(pitch_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                          * Eigen::AngleAxisd(roll_offset, Eigen::Vector3d::UnitY()).toRotationMatrix() * Hpc;

            int width  = config["settings"]["Width"].as<Expression>();
            int height = config["settings"]["Height"].as<Expression>();

            // Renormalise the focal length
            float focal_length = config["lens"]["focal_length"].as<Expression>();
            float fov          = config["lens"]["fov"].as<Expression>();

            // Recentre/renormalise the centre
            Eigen::Vector2f centre = Eigen::Vector2f(config["lens"]["centre"].as<Expression>());

            // Adjust the distortion parameters for the new width units
            Eigen::Vector2f k = config["lens"]["k"].as<Expression>();

            // Set the lens parameters from configuration
            context.lens = Image::Lens{
                config["lens"]["projection"].as<std::string>(),
                focal_length,
                fov,
                centre,
                k,
            };

            // If the lens fov was auto we need to correct it
            if (!std::isfinite(context.lens.fov)) {
                double a = height / width;
                std::array<double, 4> options{
                    utility::vision::unproject(Eigen::Vector2f(0, 0), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(1, 0), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(0, a), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(1, a), context.lens, Eigen::Vector2f(1, a)).x()};
                context.lens.fov = std::acos(*std::min_element(options.begin(), options.end())) * 2.0;
            }

            camera_context[name] = std::move(context);
        });

        on<Trigger<Sensors>>().then("Buffer Sensors", [this](const Sensors& sensors) {
            std::lock_guard<std::mutex> lock(sensors_mutex);
            auto now = NUClear::clock::now();
            Hwps.resize(std::distance(Hwps.begin(), std::remove_if(Hwps.begin(), Hwps.end(), [now](const auto& v) {
                                          return v.first < (now - std::chrono::milliseconds(500));
                                      })));

            // Get torso to head, and torso to world
            Eigen::Isometry3d Htp(sensors.Htx[FrameID::HEAD_PITCH]);
            Eigen::Isometry3d Htw(sensors.Htw);
            Eigen::Isometry3d Hwp = Htw.inverse() * Htp;

            Hwps.emplace_back(sensors.timestamp, Hwp);
        });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, With<RawSensors>, Sync<ServoState>>().then([this](const ServoTargets& targets,
                                                                                    const RawSensors& sensors) {
            // Loop through each of our commands
            for (const auto& target : targets.targets) {
                // Get the difference between the current servo position and our servo target
                const double diff = utility::math::angle::difference(
                    double(target.position),
                    utility::platform::get_raw_servo(target.id, sensors).present_position);
                // Get the difference between the current time and the time the servo should reach its target
                NUClear::clock::duration duration = target.time - NUClear::clock::now();

                // If we have a positive duration, find the velocity.
                // Otherwise, if the duration is negative or 0, the servo should have reached its position
                // before now Because of this, we move the servo as fast as we can to reach the position. The
                // fastest speed is determined by the config, which comes from the max servo velocity from
                // NUgus.proto in Webots
                double max_velocity = 0.0;
                if (target.id >= ServoID::R_HIP_YAW && target.id <= ServoID::L_ANKLE_ROLL) {
                    max_velocity = max_velocity_mx106;
                }
                else {
                    max_velocity = max_velocity_mx64;
                }
                double speed = duration.count() > 0
                                   ? diff / (double(duration.count()) / double(NUClear::clock::period::den))
                                   : max_velocity;

                speed = std::min(max_velocity, speed);
                // Update our internal state if anything has changed for this servo
                if (servo_state[target.id].p_gain != target.gain || servo_state[target.id].i_gain != target.gain * 0.0
                    || servo_state[target.id].d_gain != target.gain * 0.0
                    || servo_state[target.id].moving_speed != speed
                    || servo_state[target.id].goal_position != target.position
                    || servo_state[target.id].torque != target.torque) {

                    servo_state[target.id].dirty = true;
                    servo_state[target.id].id    = target.id;
                    servo_state[target.id].name  = translate_id_servo(target.id);

                    servo_state[target.id].p_gain = target.gain;
                    // `i` and `d` gains are always 0
                    // servo_state[target.id].i_gain        = target.gain * 0.0;
                    // servo_state[target.id].d_gain        = target.gain * 0.0;
                    servo_state[target.id].moving_speed  = speed;
                    servo_state[target.id].goal_position = target.position;

                    servo_state[target.id].torque = target.torque;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& target) {
            auto targets = std::make_unique<ServoTargets>();
            targets->targets.emplace_back(target);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(targets);
        });

        on<Shutdown>().then([this] {
            // Disconnect the fd gracefully
            if (fd != -1) {
                shutdown(fd, SHUT_RDWR);
                close(fd);
                fd = -1;
            }
        });

        // Used to reset our local servo state when the robot is teleported by the referee in the simulation.
        // Needed to cancel old servo targets and reset the pose to account for the teleportation.
        on<Trigger<ResetWebotsServos>>().then([this]() {
            // Reset the servo state
            for (auto& servo : servo_state) {
                servo.dirty            = false;
                servo.p_gain           = 32.0 / 255.0;
                servo.moving_speed     = 0.0;
                servo.goal_position    = 0.0;
                servo.torque           = 0.0;
                servo.present_position = 0.0;
                servo.present_speed    = 0.0;
            }

            auto targets = std::make_unique<ServoTargets>();

            // Clear all servo targets on reset
            for (int i = 0; i < ServoID::NUMBER_OF_SERVOS; i++) {
                targets->targets.emplace_back(NUClear::clock::now(), i, 0.0, 1, 0);
            }

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(targets);
        });

        on<Trigger<OptimisationCommand>>().then([this](const OptimisationCommand& msg) {
            const int msg_command = msg.command;
            switch (msg_command) {
                case OptimisationCommand::CommandType::RESET_ROBOT:
                    // Set the reset world flag to send the reset command to webots with the next ActuatorRequests
                    reset_simulation_world = true;
                    break;

                case OptimisationCommand::CommandType::RESET_TIME:
                    // Set the reset flag to send the reset command to webots with the next ActuatorRequests
                    reset_simulation_time = true;
                    break;

                case OptimisationCommand::CommandType::TERMINATE:
                    // Set the termination flag to send the terminate command to webots with the next ActuatorRequests
                    terminate_simulation = true;
                    break;
            }
        });
    }

    void Webots::setup_connection() {
        // This will return false if a reconnection is not currently in progress
        if (!active_reconnect.exchange(true)) {
            connection_active = false;

            // Unbind any previous reaction handles
            read_io.unbind();
            send_io.unbind();
            error_io.unbind();
            buffer.clear();

            if (fd != -1) {
                // Disconnect the fd gracefully
                shutdown(fd, SHUT_RDWR);
                close(fd);
            }

            fd = tcpip_connect();

            if (fd == -1) {
                // Connection failed
                log<NUClear::ERROR>("Failed to connect to server.");
                active_reconnect.store(false);
                return;
            }

            // Receiving
            read_io =
                on<IO, Sync<Webots>>(fd, IO::READ | IO::CLOSE | IO::ERROR)
                    .then("Read Stream", [this](const IO::Event& event) {
                        if ((event.events & IO::READ) != 0) {
                            // If we have not seen the welcome message yet, look for it
                            if (!connection_active) {
                                // Initialise the string with 0s
                                // make sure we have an extra character just in case we read something that isn't a
                                // null terminator
                                std::array<char, 9> initial_message{};
                                const int n = ::read(fd, initial_message.data(), initial_message.size() - 1);

                                if (n >= 0) {
                                    if (initial_message.data() == std::string("Welcome")) {
                                        // good
                                        log<NUClear::INFO>(
                                            fmt::format("Connected to {}:{}", server_address, server_port));
                                    }
                                    else if (initial_message.data() == std::string("Refused")) {
                                        log<NUClear::FATAL>(
                                            fmt::format("Connection to {}:{} refused: your IP is not white listed.",
                                                        server_address,
                                                        server_port));
                                        // Halt and don't retry as reconnection is pointless.
                                        close(fd);
                                        powerplant.shutdown();
                                    }
                                    else {
                                        log<NUClear::FATAL>(fmt::format("{}:{} sent unknown initial message",
                                                                        server_address,
                                                                        server_port));
                                        // Halt and don't retry as the other end is clearly not Webots
                                        close(fd);
                                        powerplant.shutdown();
                                    }
                                }
                                else {
                                    // There was nothing sent
                                    log<NUClear::DEBUG>("Connection was closed.");
                                    active_reconnect.store(false);
                                    return;
                                }

                                // Set the real time of the connection initiation
                                connect_time = NUClear::clock::now();

                                connection_active = true;
                            }
                            else {
                                // Work out how many bytes are available to read in the buffer and ensure we have
                                // enough space to read them in our data buffer
                                unsigned long available = 0;
                                if (::ioctl(fd, FIONREAD, &available) < 0) {
                                    log<NUClear::ERROR>(
                                        fmt::format("Error querying for available data, {}", strerror(errno)));
                                    return;
                                }
                                const size_t old_size = buffer.size();
                                buffer.resize(old_size + available);

                                // Read data into our buffer and resize it to the new data we read
                                const auto bytes_read = ::read(fd, buffer.data() + old_size, available);
                                // Shrink the buffer to the size that was actually read.
                                buffer.resize(old_size + bytes_read);

                                // Function to read the payload length from the buffer
                                auto read_length = [](const std::vector<uint8_t>& buffer) {
                                    return buffer.size() >= sizeof(uint32_t)
                                               ? ntohl(*reinterpret_cast<const uint32_t*>(buffer.data()))
                                               : 0u;
                                };

                                // So long as we have enough bytes to process an entire packet, process the packets
                                for (uint32_t length = read_length(buffer); buffer.size() >= length + sizeof(length);
                                     length          = read_length(buffer)) {
                                    // Decode the protocol buffer and emit it as a message
                                    uint8_t* payload = buffer.data() + sizeof(length);
                                    translate_and_emit_sensor(
                                        NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(payload,
                                                                                                             length));
                                    // Service the watchdog
                                    emit<Scope::WATCHDOG>(ServiceWatchdog<Webots>());

                                    // Delete the packet we just read ready to read the next one
                                    buffer.erase(buffer.begin(), std::next(buffer.begin(), sizeof(length) + length));
                                }
                            }
                        }

                        // For IO::ERROR and IO::CLOSE conditions the watchdog will handle reconnections so just
                        // report the error
                        else if ((event.events & IO::ERROR) != 0) {
                            if (!active_reconnect.exchange(true)) {
                                log<NUClear::WARN>(fmt::format(
                                    "An invalid request or some other error occurred. Closing our connection"));
                            }
                        }
                        else if ((event.events & IO::CLOSE) != 0) {
                            if (!active_reconnect.exchange(true)) {
                                log<NUClear::WARN>(fmt::format("The Remote hung up. Closing our connection"));
                            }
                        }
                    });

            send_io = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Sync<ServoState>, Priority::HIGH>().then(
                "Simulator Update Loop",
                [this] {
                    // Bound the time_step for the cameras and other sensors by the minimum allowed time_step for
                    // the competition
                    const uint32_t sensor_timestep = std::max(min_sensor_time_step, time_step);
                    const uint32_t camera_timestep = std::max(min_camera_time_step, time_step);

                    // Construct the ActuatorRequests message
                    ActuatorRequests actuator_requests = create_sensor_time_steps(sensor_timestep, camera_timestep);
                    for (auto& servo : servo_state) {
                        if (servo.dirty) {
                            // Servo is no longer dirty
                            servo.dirty = false;

                            // Create servo position message
                            actuator_requests.motor_positions.emplace_back(
                                MotorPosition(servo.name, servo.goal_position));

                            // Create servo velocity message
                            actuator_requests.motor_velocities.emplace_back(
                                MotorVelocity(servo.name, servo.moving_speed));

                            // Create servo PID message
                            actuator_requests.motor_pids.emplace_back(
                                MotorPID(servo.name, {servo.p_gain, servo.i_gain, servo.d_gain}));
                        }

                        // Set the terminate command if the flag is set to terminate the simulator, used by the walk
                        // simulator
                        if (terminate_simulation) {
                            log<NUClear::DEBUG>("Sending terminate on ActuatorRequests.");
                            actuator_requests.optimisation_command.command =
                                OptimisationCommand::CommandType::TERMINATE;
                            terminate_simulation = false;
                        }

                        // Set the reset command if the flag is set to reset the simulator, used by the walk simulator
                        if (reset_simulation_world) {
                            log<NUClear::DEBUG>("Sending RESET_ROBOT to ActuatorRequests.");
                            actuator_requests.optimisation_command.command =
                                OptimisationCommand::CommandType::RESET_ROBOT;
                            reset_simulation_world = false;
                        }
                        else if (reset_simulation_time) {
                            log<NUClear::DEBUG>("Sending RESET_TIME to ActuatorRequests.");
                            actuator_requests.optimisation_command.command =
                                OptimisationCommand::CommandType::RESET_TIME;
                            reset_simulation_time = false;
                        }
                    }

                    // Serialise ActuatorRequests
                    std::vector<uint8_t> data =
                        NUClear::util::serialise::Serialise<ActuatorRequests>::serialise(actuator_requests);

                    // Size of the message, in network endian
                    const uint32_t Nn = htonl(data.size());

                    // Only send actuator requests if we are connected to the controller
                    if (connection_active) {
                        // Send the message size first
                        if (send(fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                            log<NUClear::ERROR>(
                                fmt::format("Error in sending ActuatorRequests' message size,  {}", strerror(errno)));
                        }


                        // Now send the data
                        if (send(fd, data.data(), data.size(), 0) != int(data.size())) {
                            log<NUClear::ERROR>(
                                fmt::format("Error sending ActuatorRequests message, {}", strerror(errno)));
                        }
                        log<NUClear::TRACE>("Sending actuator request.");
                    }
                });

            // Reconnection has now completed
            active_reconnect.store(false);
        }
    }

    void Webots::translate_and_emit_sensor(const SensorMeasurements& sensor_measurements) {
        // ****************************** TIME **************************************
        // Deal with time first

        // If our local sim time is non zero and we just got one that is zero, that means the simulation was reset
        // (which is something we do for the walk optimisation), so reset our local times
        if (sim_delta > 0 && sensor_measurements.time == 0) {
            log<NUClear::DEBUG>("Webots sim time reset to zero, resetting local sim_time. time before reset:",
                                current_sim_time);
            sim_delta         = 0;
            real_delta        = 0;
            current_sim_time  = 0;
            current_real_time = 0;

            // Reset the local raw sensors buffer
            emit(std::make_unique<ResetWebotsServos>());
        }

        // Save our previous deltas
        const uint32_t prev_sim_delta  = sim_delta;
        const uint64_t prev_real_delta = real_delta;

        // Update our current deltas
        real_delta = sensor_measurements.real_time - current_real_time;
        sim_delta  = sensor_measurements.time - current_sim_time;

        // Calculate our custom rtf - the ratio of the past two sim deltas and the past two real time deltas,
        // smoothed
        const double ratio =
            static_cast<double>(sim_delta + prev_sim_delta) / static_cast<double>(real_delta + prev_real_delta);

        // Exponential filter to do the smoothing
        rtf = rtf * clock_smoothing + (1.0 - clock_smoothing) * ratio;
        utility::clock::update_rtf(rtf);

        // Update our current times
        current_sim_time  = sensor_measurements.time;
        current_real_time = sensor_measurements.real_time;

        // Emit the webots time update
        auto time_update_msg        = std::make_unique<OptimisationTimeUpdate>();
        time_update_msg->real_time  = current_real_time;
        time_update_msg->sim_delta  = sim_delta;
        time_update_msg->real_delta = real_delta;
        emit(time_update_msg);

        // ************************* DEBUGGING LOGS *********************************
        log<NUClear::TRACE>("received SensorMeasurements:");
        log<NUClear::TRACE>("  sm.time:", sensor_measurements.time);
        log<NUClear::TRACE>("  sm.real_time:", sensor_measurements.real_time);

        log<NUClear::TRACE>("  sm.messages:");
        for (int i = 0; i < int(sensor_measurements.messages.size()); ++i) {
            const auto& message = sensor_measurements.messages[i];
            log<NUClear::TRACE>("    sm.messages #", i);
            log<NUClear::TRACE>("      message_type:", message.message_type);
            log<NUClear::TRACE>("      text:", message.text);
        }

        log<NUClear::TRACE>("  sm.accelerometers:");
        for (int i = 0; i < int(sensor_measurements.accelerometers.size()); ++i) {
            const auto& acc = sensor_measurements.accelerometers[i];
            log<NUClear::TRACE>("    sm.accelerometers #", i);
            log<NUClear::TRACE>("      name:", acc.name);
            log<NUClear::TRACE>("      value:", acc.value.X, ",", acc.value.Y, ",", acc.value.Z);
        }

        log<NUClear::TRACE>("  sm.bumpers:");
        for (int i = 0; i < int(sensor_measurements.bumpers.size()); ++i) {
            const auto& bumper = sensor_measurements.bumpers[i];
            log<NUClear::TRACE>("    sm.bumpers #", i);
            log<NUClear::TRACE>("      name:", bumper.name);
            log<NUClear::TRACE>("      value:", bumper.value);
        }

        log<NUClear::TRACE>("  sm.cameras:");
        for (int i = 0; i < int(sensor_measurements.cameras.size()); ++i) {
            const auto& camera = sensor_measurements.cameras[i];
            log<NUClear::TRACE>("    sm.cameras #", i);
            log<NUClear::TRACE>("      name:", camera.name);
            log<NUClear::TRACE>("      width:", camera.width);
            log<NUClear::TRACE>("      height:", camera.height);
            log<NUClear::TRACE>("      quality:", camera.quality);
            log<NUClear::TRACE>("      image (size):", camera.image.size());
        }

        log<NUClear::TRACE>("  sm.forces:");
        for (int i = 0; i < int(sensor_measurements.forces.size()); ++i) {
            const auto& force = sensor_measurements.forces[i];
            log<NUClear::TRACE>("    sm.forces #", i);
            log<NUClear::TRACE>("      name:", force.name);
            log<NUClear::TRACE>("      value:", force.value);
        }

        log<NUClear::TRACE>("  sm.force3ds:");
        for (int i = 0; i < int(sensor_measurements.force3ds.size()); ++i) {
            const auto& force = sensor_measurements.force3ds[i];
            log<NUClear::TRACE>("    sm.force3ds #", i);
            log<NUClear::TRACE>("      name:", force.name);
            log<NUClear::TRACE>("      value:", force.value.X, ",", force.value.Y, ",", force.value.Z);
        }

        log<NUClear::TRACE>("  sm.force6ds:");
        for (int i = 0; i < int(sensor_measurements.force6ds.size()); ++i) {
            const auto& force = sensor_measurements.force6ds[i];
            log<NUClear::TRACE>("    sm.force6ds #", i);
            log<NUClear::TRACE>("      name:", force.name);
            log<NUClear::TRACE>("      force:", force.force.X, ",", force.force.Y, ",", force.force.Z);
            log<NUClear::TRACE>("      torque:", force.torque.X, ",", force.force.Y, ",", force.force.Z);
        }

        log<NUClear::TRACE>("  sm.gyros:");
        for (int i = 0; i < int(sensor_measurements.gyros.size()); ++i) {
            const auto& gyro = sensor_measurements.gyros[i];
            log<NUClear::TRACE>("    sm.gyros #", i);
            log<NUClear::TRACE>("      name:", gyro.name);
            log<NUClear::TRACE>("      value:", gyro.value.X, ",", gyro.value.Y, ",", gyro.value.Z);
        }

        log<NUClear::TRACE>("  sm.position_sensors:");
        for (int i = 0; i < int(sensor_measurements.position_sensors.size()); ++i) {
            const auto& sensor = sensor_measurements.position_sensors[i];
            log<NUClear::TRACE>("    sm.position_sensors #", i);
            log<NUClear::TRACE>("      name:", sensor.name);
            log<NUClear::TRACE>("      value:", sensor.value);
        }

        if (sensor_measurements.odometry_ground_truth.exists) {
            log<NUClear::TRACE>("  sm.odometry_ground_truth:");
            log<NUClear::TRACE>("    Htw:\n", sensor_measurements.odometry_ground_truth.Htw);
        }

        if (sensor_measurements.localisation_ground_truth.exists) {
            log<NUClear::TRACE>("  sm.localisation_ground_truth:");
            log<NUClear::TRACE>("    Hfw:\n", sensor_measurements.localisation_ground_truth.Hfw);
        }

        // Parse the errors and warnings from Webots and log them.
        // Note that this is where we should deal with specific messages passed in SensorMeasurements.messages.
        // Or check if those messages have specific information
        for (const auto& message : sensor_measurements.messages) {
            switch (int(message.message_type)) {
                case Message::MessageType::ERROR_MESSAGE: log<NUClear::ERROR>(message.text); break;
                case Message::MessageType::WARNING_MESSAGE: log<NUClear::WARN>(message.text); break;
            }
        }

        // Only emit RawSensors if there is any data!
        if (!(sensor_measurements.position_sensors.empty() && sensor_measurements.accelerometers.empty()
              && sensor_measurements.bumpers.empty() && sensor_measurements.gyros.empty())) {


            // Read each field of msg, translate it to our protobuf and emit the data
            auto sensor_data = std::make_unique<RawSensors>();

            sensor_data->timestamp = NUClear::clock::now();

            for (const auto& position : sensor_measurements.position_sensors) {
                auto& servo            = translate_servo_id(position.name, sensor_data->servo);
                servo.present_position = position.value;
                servo.goal_position    = servo_state[translate_name_to_id(position.name)].goal_position;
            }

            if (!sensor_measurements.accelerometers.empty()) {
                // .accelerometers is a list of one, since our robots have only one accelerometer
                const auto& accelerometer = sensor_measurements.accelerometers[0];
                // Webots has a strictly positive output for the accelerometers. We minus 100 to center the output
                // over 0 The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept
                // consistent with that
                sensor_data->accelerometer.x() = static_cast<float>(accelerometer.value.X) - 100.0f;
                sensor_data->accelerometer.y() = static_cast<float>(accelerometer.value.Y) - 100.0f;
                sensor_data->accelerometer.z() = static_cast<float>(accelerometer.value.Z) - 100.0f;
            }

            if (!sensor_measurements.gyros.empty()) {
                // .gyros is a list of one, since our robots have only one gyroscope
                const auto& gyro = sensor_measurements.gyros[0];
                // Webots has a strictly positive output for the gyros. We minus 100 to center the output over 0
                // The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept consistent with
                // that
                sensor_data->gyroscope.x() = static_cast<float>(gyro.value.X) - 100.0f;
                sensor_data->gyroscope.y() = static_cast<float>(gyro.value.Y) - 100.0f;
                sensor_data->gyroscope.z() = static_cast<float>(gyro.value.Z) - 100.0f;
            }

            for (const auto& bumper : sensor_measurements.bumpers) {
                // We should have eight bumper sensors
                // Right foot
                if (bumper.name == "right_touch_sensor_br") {
                    sensor_data->fsr.right.fsr1 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "right_touch_sensor_bl") {
                    sensor_data->fsr.right.fsr2 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "right_touch_sensor_fl") {
                    sensor_data->fsr.right.fsr3 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "right_touch_sensor_fr") {
                    sensor_data->fsr.right.fsr4 = bumper.value ? max_fsr_value : 0.0f;
                }
                // Left foot
                else if (bumper.name == "left_touch_sensor_br") {
                    sensor_data->fsr.left.fsr1 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "left_touch_sensor_bl") {
                    sensor_data->fsr.left.fsr2 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "left_touch_sensor_fl") {
                    sensor_data->fsr.left.fsr3 = bumper.value ? max_fsr_value : 0.0f;
                }
                else if (bumper.name == "left_touch_sensor_fr") {
                    sensor_data->fsr.left.fsr4 = bumper.value ? max_fsr_value : 0.0f;
                }
            }

            // If we got ground truth data, send it through with the sensors
            if (sensor_measurements.odometry_ground_truth.exists) {
                sensor_data->odometry_ground_truth.exists = true;
                sensor_data->odometry_ground_truth.Htw    = sensor_measurements.odometry_ground_truth.Htw;
                sensor_data->odometry_ground_truth.vTw    = sensor_measurements.odometry_ground_truth.vTw;
            }
            if (sensor_measurements.localisation_ground_truth.exists) {
                sensor_data->localisation_ground_truth.exists = true;
                sensor_data->localisation_ground_truth.Hfw    = sensor_measurements.localisation_ground_truth.Hfw;
            }

            emit(sensor_data);
        }

        for (const auto& camera : sensor_measurements.cameras) {
            // Convert the incoming image so we can emit it to the PowerPlant.
            auto image =
                std::make_unique<Image>();  // Change to CompressedImage when compression is implemented in webots
            image->name           = camera.name;
            image->dimensions.x() = camera.width;
            image->dimensions.y() = camera.height;
            image->format         = fourcc("BGR3");  // Change to "JPEG" when webots compression is implemented
            image->data           = camera.image;

            image->id        = camera_context[camera.name].id;
            image->timestamp = NUClear::clock::now();

            Eigen::Isometry3d Hcw;

            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(sensors_mutex);

                const Eigen::Isometry3d& Hpc = camera_context[camera.name].Hpc;
                Eigen::Isometry3d Hwp        = Eigen::Isometry3d::Identity();
                if (!Hwps.empty()) {
                    // Find the first time that is not less than the target time
                    auto Hwp_it = std::lower_bound(Hwps.begin(),
                                                   Hwps.end(),
                                                   std::make_pair(image->timestamp, Eigen::Isometry3d::Identity()),
                                                   [](const auto& a, const auto& b) { return a.first < b.first; });

                    if (Hwp_it == Hwps.end()) {
                        // Image is newer than most recent sensors
                        Hwp = std::prev(Hwp_it)->second;
                    }
                    else if (Hwp_it == Hwps.begin()) {
                        // Image is older than oldest sensors
                        Hwp = Hwp_it->second;
                    }
                    else {
                        // Check Hwp_it and std::prev(Hwp) for closest match
                        Hwp = std::abs((Hwp_it->first - image->timestamp).count())
                                      < std::abs((std::prev(Hwp_it)->first - image->timestamp).count())
                                  ? Hwp_it->second
                                  : std::prev(Hwp_it)->second;
                    }
                }

                Hcw = Eigen::Isometry3d(Hwp * Hpc).inverse();
            }

            image->lens = camera_context[camera.name].lens;
            image->Hcw  = Hcw;

            // If we got ground truth data, send it through with the image
            if (sensor_measurements.vision_ground_truth.exists) {
                image->vision_ground_truth = sensor_measurements.vision_ground_truth;
            }
            emit(image);
        }

        // Create and emit the OptimisationRobotPosition message used by the walk optimiser
        auto robot_position   = std::make_unique<OptimisationRobotPosition>();
        robot_position->value = sensor_measurements.robot_position.value;
        emit(robot_position);

        // Create and emit the OptimisationResetDone message used by the walk optimiser
        if (sensor_measurements.reset_done) {
            emit(std::make_unique<OptimisationResetDone>());
        }
    }
}  // namespace module::platform
