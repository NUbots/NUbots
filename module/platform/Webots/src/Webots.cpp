/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include "Webots.hpp"

#include <chrono>
#include <fmt/format.h>
#include <string>

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/motion/ServoTarget.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"

#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/vision/fourcc.hpp"

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

    /// @brief The clock period factor, used to convert simulation time to real time
    static constexpr float CLOCK_PERIOD_FACTOR =
        static_cast<float>(NUClear::clock::period::num) / static_cast<float>(NUClear::clock::period::den);

    using extension::Configuration;
    using message::input::Image;
    using message::motion::ServoTarget;
    using message::motion::ServoTargets;
    using message::output::CompressedImage;
    using message::platform::RawSensors;

    using message::platform::webots::ActuatorRequests;
    using message::platform::webots::Message;
    using message::platform::webots::MotorPID;
    using message::platform::webots::MotorPosition;
    using message::platform::webots::MotorVelocity;
    using message::platform::webots::SensorMeasurements;
    using message::platform::webots::SensorTimeStep;

    using utility::platform::getRawServo;
    using utility::vision::fourcc;

    // Converts the NUgus.proto servo name to the equivalent RawSensor.proto name
    RawSensors::Servo& translate_servo_id(const std::string& name, RawSensors::Servos& servos) {

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

        throw std::runtime_error("Unable to translate unknown NUgus.proto sensor name: " + name);
    }

    std::string translate_id_servo(const uint32_t& id) {
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

        throw std::runtime_error("Unable to translate unknown NUgus.proto servo id: " + id);
    }

    ActuatorRequests create_sensor_time_steps(const uint32_t& sensor_timestep, const uint32_t& camera_timestep) {
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
        addrinfo hints;
        memset(&hints, 0, sizeof(addrinfo));  // Defaults on what we do not explicitly set
        hints.ai_family   = AF_UNSPEC;        // IPv4 or IPv6
        hints.ai_socktype = SOCK_STREAM;      // TCP

        // Store the ip address information that we will connect to
        addrinfo* address;

        int error;
        if ((error = getaddrinfo(server_address.c_str(), server_port.c_str(), &hints, &address)) != 0) {
            log<NUClear::ERROR>(fmt::format("Cannot resolve server name: {}. Error {}. Error code {}",
                                            server_address,
                                            gai_strerror(error),
                                            error));
            return -1;
        }

        // Loop through the linked list of potential options for connecting. In order of best to worst.
        for (addrinfo* addr_ptr = address; addr_ptr != NULL; addr_ptr = addr_ptr->ai_next) {
            int fd_temp = socket(addr_ptr->ai_family, addr_ptr->ai_socktype, addr_ptr->ai_protocol);

            if (fd_temp == -1) {
                // Bad fd
                continue;
            }
            else if (connect(fd_temp, addr_ptr->ai_addr, addr_ptr->ai_addrlen) != -1) {
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
        on<Configuration>("webots.yaml").then([this](const Configuration& config) {
            // Use configuration here from file webots.yaml
            time_step            = config["time_step"].as<int>();
            min_camera_time_step = config["min_camera_time_step"].as<int>();
            min_sensor_time_step = config["min_sensor_time_step"].as<int>();
            max_velocity         = config["max_velocity"].as<double>();

            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            clock_smoothing = config["clock_smoothing"].as<double>();

            server_address = config["server_address"].as<std::string>();
            server_port    = config["port"].as<std::string>();

            on<Watchdog<Webots, 5, std::chrono::seconds>>().then([this, config] {
                // We haven't received any messages lately
                log<NUClear::WARN>("Connection timed out. Attempting reconnect");
                setup_connection();
            });

            // Connect to the server
            setup_connection();
        });

        // This trigger updates our current servo state
        on<Trigger<ServoTargets>, With<RawSensors>>().then([this](const ServoTargets& targets,
                                                                  const RawSensors& sensors) {
            // Loop through each of our commands
            for (const auto& target : targets.targets) {
                // Get the difference between the current servo position and our servo target
                const double diff = utility::math::angle::difference(
                    double(target.position),
                    utility::platform::getRawServo(target.id, sensors).present_position);
                // Get the difference between the current time and the time the servo should reach its target
                NUClear::clock::duration duration = target.time - NUClear::clock::now();

                // If we have a positive duration, find the velocity.
                // Otherwise, if the duration is negative or 0, the servo should have reached its position before now
                // Because of this, we move the servo as fast as we can to reach the position.
                // The fastest speed is determined by the config, which comes from the max servo velocity from
                // NUgus.proto in Webots
                double speed =
                    duration.count() > 0 ? diff / std::chrono::duration<double>(duration).count() : max_velocity;
                speed = std::min(max_velocity, speed);
                // Update our internal state
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
    }

    void Webots::setup_connection() {
        // This will return false if a reconnection is not currently in progress
        if (!active_reconnect.exchange(true)) {
            connection_active = false;

            // Unbind any previous reaction handles
            read_io.unbind();
            send_io.unbind();
            error_io.unbind();

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
            read_io = on<IO>(fd, IO::READ | IO::CLOSE | IO::ERROR).then("Read Stream", [this](const IO::Event& event) {
                if ((event.events & IO::READ) != 0) {
                    // Service the watchdog
                    emit<Scope::WATCHDOG>(ServiceWatchdog<Webots>());
                    // If we have not seen the welcome message yet, look for it
                    if (!connection_active) {
                        // Initaliase the string with ???????
                        std::string initial_message = std::string(7, '?');
                        const int n                 = ::read(fd, initial_message.data(), sizeof(initial_message));

                        if (n >= 0) {
                            if (initial_message == "Welcome") {
                                // good
                                log<NUClear::INFO>(fmt::format("Connected to {}:{}", server_address, server_port));
                            }
                            else if (initial_message == "Refused") {
                                log<NUClear::FATAL>(
                                    fmt::format("Connection to {}:{} refused: your IP is not white listed.",
                                                server_address,
                                                server_port));
                                // Halt and don't retry as reconnection is pointless.
                                close(fd);
                                powerplant.shutdown();
                            }
                            else {
                                log<NUClear::FATAL>(
                                    fmt::format("{}:{} sent unknown initial message", server_address, server_port));
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
                        // Reset the simulation connection time
                        utility::clock::last_update = NUClear::base_clock::now();

                        connection_active = true;
                    }
                    else {
                        // Work out how many bytes are available to read in the buffer and ensure we have enough
                        // space to read them in our data buffer
                        unsigned long available = 0;
                        if (::ioctl(fd, FIONREAD, &available) < 0) {
                            log<NUClear::ERROR>(fmt::format("Error querying for available data, {}", strerror(errno)));
                            return;
                        }
                        const size_t old_size = buffer.size();
                        buffer.resize(old_size + available);

                        // Read data into our buffer and resize it to the new data we read
                        auto bytes_read = ::read(fd, buffer.data() + old_size, available);
                        // Shrink the buffer to the size that was actually read.
                        buffer.resize(old_size + bytes_read);

                        // Function to read the payload length from the buffer
                        auto read_length = [this](const std::vector<uint8_t>& buffer) {
                            return buffer.size() >= sizeof(uint32_t)
                                       ? ntohl(*reinterpret_cast<const uint32_t*>(buffer.data()))
                                       : 0u;
                        };

                        // So long as we have enough bytes to process an entire packet, process the packets
                        for (uint32_t length = read_length(buffer); buffer.size() >= length + sizeof(length);
                             length          = read_length(buffer)) {
                            // Decode the protocol buffer and emit it as a message
                            char* payload = reinterpret_cast<char*>(buffer.data()) + sizeof(length);

                            translate_and_emit_sensor(
                                NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(payload, length));

                            // Delete the packet we just read ready to read the next one
                            buffer.erase(buffer.begin(), std::next(buffer.begin(), sizeof(length) + length));
                        }
                    }
                }

                // For IO::ERROR and IO::CLOSE conditions the watchdog will handle reconnections so just report
                // the error
                else if ((event.events & IO::ERROR) != 0) {
                    if (!active_reconnect.exchange(true)) {
                        log<NUClear::WARN>(
                            fmt::format("An invalid request or some other error occurred. Closing our connection"));
                    }
                }
                else if ((event.events & IO::CLOSE) != 0) {
                    if (!active_reconnect.exchange(true)) {
                        log<NUClear::WARN>(fmt::format("The Remote hung up. Closing our connection"));
                    }
                }
            });

            send_io = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then(
                "Simulator Update Loop",
                [this] {
                    // Bound the time_step for the cameras and other sensors by the minimum allowed time_step for the
                    // competition
                    uint32_t sensor_timestep = std::max(min_sensor_time_step, time_step);
                    uint32_t camera_timestep = std::max(min_camera_time_step, time_step);

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
                    }

                    // Serialise ActuatorRequests
                    std::vector<char> data =
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

        // Save our previous deltas
        const uint32_t prev_sim_delta  = sim_delta;
        const uint64_t prev_real_delta = real_delta;

        // Update our current deltas
        real_delta = sensor_measurements.real_time - current_real_time;
        sim_delta  = sensor_measurements.time - current_sim_time;

        // Calculate our custom rtf - the ratio of the past two sim deltas and the past two real time deltas, smoothed
        const double ratio =
            static_cast<double>(sim_delta + prev_sim_delta) / static_cast<double>(real_delta + prev_real_delta);
        utility::clock::custom_rtf = utility::clock::custom_rtf * clock_smoothing + (1.0 - clock_smoothing) * ratio;

        // ************************* DEBUGGING LOGS *********************************

        // Update our current times
        current_sim_time  = sensor_measurements.time;
        current_real_time = sensor_measurements.real_time;
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
        if (!(sensor_measurements.position_sensors.size() == 0 && sensor_measurements.accelerometers.size() == 0
              && sensor_measurements.bumpers.size() == 0 && sensor_measurements.gyros.size() == 0)) {


            // Read each field of msg, translate it to our protobuf and emit the data
            auto sensor_data = std::make_unique<RawSensors>();

            sensor_data->timestamp = NUClear::clock::now();

            for (const auto& position : sensor_measurements.position_sensors) {
                translate_servo_id(position.name, sensor_data->servo).present_position = position.value;
            }

            if (sensor_measurements.accelerometers.size() > 0) {
                // .accelerometers is a list of one, since our robots have only one accelerometer
                const auto& accelerometer = sensor_measurements.accelerometers[0];
                // Webots has a strictly positive output for the accelerometers. We minus 100 to center the output over
                // 0 The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept consistent with
                // that
                sensor_data->accelerometer.x = static_cast<float>(accelerometer.value.X) - 100.0f;
                sensor_data->accelerometer.y = static_cast<float>(accelerometer.value.Y) - 100.0f;
                sensor_data->accelerometer.z = static_cast<float>(accelerometer.value.Z) - 100.0f;
            }

            if (sensor_measurements.gyros.size() > 0) {
                // .gyros is a list of one, since our robots have only one gyroscope
                const auto& gyro = sensor_measurements.gyros[0];
                // Webots has a strictly positive output for the gyros. We minus 100 to center the output over 0
                // The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept consistent with
                // that
                sensor_data->gyroscope.x = static_cast<float>(gyro.value.X) - 100.0f;
                sensor_data->gyroscope.y = static_cast<float>(gyro.value.Y) - 100.0f;
                sensor_data->gyroscope.z = static_cast<float>(gyro.value.Z) - 100.0f;
            }

            for (const auto& bumper : sensor_measurements.bumpers) {
                // We should have eight bumper sensors
                // Right foot
                if (bumper.name == "right_touch_sensor_br") {
                    sensor_data->fsr.right.fsr1 = bumper.value;
                }
                else if (bumper.name == "right_touch_sensor_bl") {
                    sensor_data->fsr.right.fsr2 = bumper.value;
                }
                else if (bumper.name == "right_touch_sensor_fl") {
                    sensor_data->fsr.right.fsr3 = bumper.value;
                }
                else if (bumper.name == "right_touch_sensor_fr") {
                    sensor_data->fsr.right.fsr4 = bumper.value;
                }
                // Left foot
                else if (bumper.name == "left_touch_sensor_br") {
                    sensor_data->fsr.left.fsr1 = bumper.value;
                }
                else if (bumper.name == "left_touch_sensor_bl") {
                    sensor_data->fsr.left.fsr2 = bumper.value;
                }
                else if (bumper.name == "left_touch_sensor_fl") {
                    sensor_data->fsr.left.fsr3 = bumper.value;
                }
                else if (bumper.name == "left_touch_sensor_fr") {
                    sensor_data->fsr.left.fsr4 = bumper.value;
                }
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
            image->format         = fourcc("RGB3");  // Change to "JPEG" when webots compression is implemented
            image->data           = camera.image;
            emit(image);
        }
    }
}  // namespace module::platform
