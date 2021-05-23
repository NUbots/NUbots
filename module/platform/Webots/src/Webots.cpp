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
#include "message/platform/webots/ConnectRequest.hpp"
#include "message/platform/webots/messages.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/vision/fourcc.hpp"

// Include headers needed for TCP connection
extern "C" {
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
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
    using message::platform::webots::ConnectRequest;
    using message::platform::webots::Message;
    using message::platform::webots::MotorPID;
    using message::platform::webots::MotorPosition;
    using message::platform::webots::MotorTorque;
    using message::platform::webots::MotorVelocity;
    using message::platform::webots::SensorMeasurements;
    using message::platform::webots::SensorTimeStep;

    using utility::input::ServoID;
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

    ActuatorRequests create_sensor_time_steps(const uint32_t& time_step) {
        message::platform::webots::ActuatorRequests msg;
        msg.sensor_time_steps = {{"left_ankle_roll_sensor", time_step},
                                 {"left_ankle_pitch_sensor", time_step},
                                 {"right_ankle_roll_sensor", time_step},
                                 {"right_ankle_pitch_sensor", time_step},
                                 {"right_knee_pitch_sensor", time_step},
                                 {"left_knee_pitch_sensor", time_step},
                                 {"left_hip_roll_sensor", time_step},
                                 {"left_hip_pitch_sensor", time_step},
                                 {"left_hip_yaw_sensor", time_step},
                                 {"right_hip_roll_sensor", time_step},
                                 {"right_hip_pitch_sensor", time_step},
                                 {"right_hip_yaw_sensor", time_step},
                                 {"left_elbow_pitch_sensor", time_step},
                                 {"right_elbow_pitch_sensor", time_step},
                                 {"left_shoulder_roll_sensor", time_step},
                                 {"left_shoulder_pitch_sensor", time_step},
                                 {"right_shoulder_roll_sensor", time_step},
                                 {"right_shoulder_pitch_sensor", time_step},
                                 {"neck_yaw_sensor", time_step},
                                 {"head_pitch_sensor", time_step},
                                 {"accelerometer", time_step},
                                 {"gyroscope", time_step},
                                 {"right_camera", time_step},
                                 {"left_camera", time_step}};

        return msg;
    }

    int Webots::tcpip_connect(const std::string& server_name, const std::string& port) {
        // Hints for the connection type
        addrinfo hints;
        memset(&hints, 0, sizeof(addrinfo));  // Defaults on what we do not explicitly set
        hints.ai_family   = AF_UNSPEC;        // IPv4 or IPv6
        hints.ai_socktype = SOCK_STREAM;      // TCP

        // Store the ip address information that we will connect to
        addrinfo* address;

        int error;
        if ((error = getaddrinfo(server_name.c_str(), port.c_str(), &hints, &address)) != 0) {
            log<NUClear::ERROR>(fmt::format("Cannot resolve server name: {}. Error {}. Error code {}",
                                            server_name,
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
        log<NUClear::ERROR>(fmt::format("Cannot connect to server: {}:{}", server_name, port));
        return -1;
    }

    Webots::Webots(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Configuration>("webots.yaml").then([this](const Configuration& config) {
            // Use configuration here from file webots.yaml
            time_step = config["time_step"].as<int>();

            // clang-format off
            auto lvl = config["log_level"].as<std::string>();
            if      (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
            else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
            else if (lvl == "INFO")  { this->log_level = NUClear::INFO; }
            else if (lvl == "WARN")  { this->log_level = NUClear::WARN; }
            else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
            else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
            // clang-format on

            on<Watchdog<Webots, 5, std::chrono::seconds>>().then([this, config] {
                // We haven't received any messages lately
                log<NUClear::ERROR>("Connection timed out.");
                setup_connection(config["server_address"].as<std::string>(), config["port"].as<std::string>());
            });

            // Connect to the server
            setup_connection(config["server_address"].as<std::string>(), config["port"].as<std::string>());
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

                double speed = 0.0;
                // If we have a positive duration, find the velocity
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }
                else {
                    // The duration is negative, so the servo should have reached its position before now
                    // Because of this, we move the servo as fast as we can to reach the position.
                    // 5.236 == 50 rpm which is similar to the max speed of the servos
                    speed = 5.236;
                }

                // Update our internal state
                if (servo_state[target.id].p_gain != target.gain || servo_state[target.id].i_gain != target.gain * 0.0
                    || servo_state[target.id].d_gain != target.gain * 0.0
                    || servo_state[target.id].moving_speed != speed
                    || servo_state[target.id].goal_position != target.position
                    || servo_state[target.id].torque != target.torque) {

                    servo_state[target.id].dirty = true;
                    servo_state[target.id].id    = target.id;
                    servo_state[target.id].name  = translate_id_servo(target.id);

                    servo_state[target.id].p_gain        = target.gain;
                    servo_state[target.id].i_gain        = target.gain * 0.0;
                    servo_state[target.id].d_gain        = target.gain * 0.0;
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

    void Webots::setup_connection(const std::string& server_address, const std::string& port) {
        // Unbind any previous reaction handles
        read_io.unbind();
        send_io.unbind();
        error_io.unbind();

        if (fd != -1) {
            // Disconnect the fd gracefully
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }

        fd = tcpip_connect(server_address, port);

        if (fd == -1) {
            // Connection failed
            // TODO(Cameron) Try to reconnect after a delay.
            // setup_connection(server_address, port);
            log<NUClear::FATAL>("Quitting due to failed connection attempt");
            powerplant.shutdown();
            return;
        }

        // Initaliase the string with ???????
        std::string initial_message = std::string(7, '?');
        const int n                 = recv(fd, initial_message.data(), sizeof(initial_message), MSG_WAITALL);

        if (n >= 0) {
            if (initial_message == "Welcome") {
                // good
                log<NUClear::INFO>(fmt::format("Connected to {}:{}", server_address, port));
            }
            else if (initial_message == "Refused") {
                log<NUClear::FATAL>(
                    fmt::format("Connection to {}:{} refused: your ip is not white listed.", server_address, port));
                // Halt and don't retry as reconnection is pointless.
                close(fd);
                powerplant.shutdown();
            }
            else {
                log<NUClear::FATAL>(fmt::format("{}:{} sent unknown initial message", server_address, port));
                // Halt and don't retry as the other end is clearly not Webots
                close(fd);
                powerplant.shutdown();
            }
        }
        else {
            // There was nothing sent
            log<NUClear::DEBUG>("Connection was closed.");
            setup_connection(server_address, port);
            return;
        }

        // Set the real time of the connection initiation
        connect_time = NUClear::clock::now();
        // Reset the simulation connection time
        utility::clock::last_update = NUClear::base_clock::now();

        // Now that we are connected, we can set up our reaction handles with this file descriptor and send the sensor
        // timestamps message

        // Create the sensor timestamps message
        // This will activate all of the sensors in the simulator
        const std::vector<char> data =
            NUClear::util::serialise::Serialise<ActuatorRequests>::serialise(create_sensor_time_steps(time_step));

        const uint32_t Nn = htonl(data.size());

        // Send the sensor timestamps message
        if (send(fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
            log<NUClear::ERROR>(fmt::format("Error in sending ActuatorRequests' message size,  {}", strerror(errno)));
            return;
        }
        if (send(fd, data.data(), data.size(), 0) != (signed) data.size()) {
            log<NUClear::ERROR>(fmt::format("Error sending ActuatorRequests message, {}", strerror(errno)));
            return;
        }

        // Receiving
        read_io = on<IO>(fd, IO::READ).then([this]() {
            // ************************** Receiving ***************************************
            // Get the size of the message
            uint32_t Nn;
            if (recv(fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                log<NUClear::ERROR>("Failed to read message size from TCP connection");
                return;
            }

            // Convert from network endian to host endian
            const uint32_t Nh = ntohl(Nn);

            // Get the message
            std::vector<char> data(Nh, 0);
            uint64_t message = uint64_t(recv(fd, data.data(), Nh, MSG_WAITALL));
            if (message != Nh) {
                log<NUClear::ERROR>("Failed to read message from TCP connection");
                return;
            }

            log<NUClear::TRACE>("Received sensor measurements");

            // If we have data to deserialise then deserialise the message into a neutron
            if (Nh > 0) {
                translate_and_emit_sensor(NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(data));
            }
            // If we don't have any data then this should correspond to a message with all fields set to defaults
            else {
                translate_and_emit_sensor(SensorMeasurements(SensorMeasurements::protobuf_type()));
            }

            // Service the watchdog
            emit<Scope::WATCHDOG>(ServiceWatchdog<Webots>());
        });

        send_io = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then(
            "Simulator Update Loop",
            [this] {
                // Construct the ActuatorRequests message
                ActuatorRequests actuator_requests = create_sensor_time_steps(time_step);
                for (auto& servo : servo_state) {
                    if (servo.dirty) {
                        // Servo is no longer dirty
                        servo.dirty = false;

                        // Create servo position message
                        actuator_requests.motor_positions.emplace_back(MotorPosition(servo.name, servo.goal_position));

                        // Create servo velocity message
                        actuator_requests.motor_velocities.emplace_back(MotorVelocity(servo.name, servo.moving_speed));

                        // Create servo PID message
                        actuator_requests.motor_pids.emplace_back(
                            MotorPID(servo.name, {servo.p_gain, servo.i_gain, servo.d_gain}));
                    }
                }

                // Serialise ActuatorRequests
                std::vector<char> data =
                    NUClear::util::serialise::Serialise<ActuatorRequests>::serialise(actuator_requests);

                // Size of the message, in network endian
                uint32_t Nn = htonl(data.size());

                // Send the message size first
                if (send(fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                    log<NUClear::ERROR>(
                        fmt::format("Error in sending ActuatorRequests' message size,  {}", strerror(errno)));
                }

                // Now send the data
                if (send(fd, data.data(), data.size(), 0) != int(data.size())) {
                    log<NUClear::ERROR>(fmt::format("Error sending ActuatorRequests message, {}", strerror(errno)));
                }

                log<NUClear::TRACE>("Sending actuator request.");
            });

        error_io = on<IO>(fd, IO::CLOSE | IO::ERROR).then([this, server_address, port](const IO::Event& /*event*/) {
            // Something went wrong, reopen the connection
            setup_connection(server_address, port);
        });
    }

    void Webots::translate_and_emit_sensor(const SensorMeasurements& sensor_measurements) {
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


        // Read each field of msg, translate it to our protobuf and emit the data
        auto sensor_data = std::make_unique<RawSensors>();

        sensor_data->timestamp = NUClear::clock::now();

        for (const auto& position : sensor_measurements.position_sensors) {
            translate_servo_id(position.name, sensor_data->servo).present_position = position.value;
        }

        if (sensor_measurements.accelerometers.size() > 0) {
            // .accelerometers is a list of one, since our robots have only one accelerometer
            const auto& accelerometer = sensor_measurements.accelerometers[0];
            // Webots has a strictly positive output for the accelerometers. We minus 100 to center the output over 0
            // The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept consistent with that
            sensor_data->accelerometer.x = static_cast<float>(accelerometer.value.X) - 100.0f;
            sensor_data->accelerometer.y = static_cast<float>(accelerometer.value.Y) - 100.0f;
            sensor_data->accelerometer.z = static_cast<float>(accelerometer.value.Z) - 100.0f;
        }

        if (sensor_measurements.gyros.size() > 0) {
            // .gyros is a list of one, since our robots have only one gyroscope
            const auto& gyro = sensor_measurements.gyros[0];
            // Webots has a strictly positive output for the gyros. We minus 100 to center the output over 0
            // The value 100.0 is based on the Look-up Table from NUgus.proto and should be kept consistent with that
            sensor_data->gyroscope.x = static_cast<float>(gyro.value.X) - 100.0f;
            sensor_data->gyroscope.y = static_cast<float>(gyro.value.Y) - 100.0f;
            sensor_data->gyroscope.z = static_cast<float>(gyro.value.Z) - 100.0f;
        }

        // TODO Implement fsrs
        /*
        for (const auto& bumper : sensor_measurements.bumpers) {
            // string name
            // bool value
        }
        */

        emit(sensor_data);

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

        // ****************************** TIME **************************************

        // Deal with time

        // Save our previous deltas
        const uint32_t prev_sim_delta  = sim_delta;
        const uint32_t prev_real_delta = real_delta;

        // Update our current deltas
        sim_delta  = sensor_measurements.time - current_sim_time;
        real_delta = sensor_measurements.real_time - current_real_time;

        // Calculate our custom rtf - the ratio of the past two sim deltas and the past two real time deltas
        utility::clock::custom_rtf =
            static_cast<double>(sim_delta + prev_sim_delta) / static_cast<double>(real_delta + prev_real_delta);

        // Update our current times
        current_sim_time  = sensor_measurements.time;
        current_real_time = sensor_measurements.real_time;
    }
}  // namespace module::platform
