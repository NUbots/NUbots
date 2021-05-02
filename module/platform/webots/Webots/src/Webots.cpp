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

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/motion/ServoTarget.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/platform/darwin/DarwinSensors.hpp"
#include "message/platform/webots/ConnectRequest.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/platform/darwin/DarwinSensors.hpp"
#include "utility/vision/fourcc.hpp"

// Include headers needed for TCP connection
extern "C" {
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
}

namespace module::platform::webots {


    using extension::Configuration;
    using message::motion::ServoTargets;
    using message::output::CompressedImage;
    using message::platform::darwin::DarwinSensors;

    using message::platform::webots::ActuatorRequests;
    using message::platform::webots::ConnectRequest;
    using message::platform::webots::Message;
    using message::platform::webots::MotorPID;
    using message::platform::webots::MotorPosition;
    using message::platform::webots::MotorTorque;
    using message::platform::webots::MotorVelocity;
    using message::platform::webots::SensorMeasurements;

    using message::support::GlobalConfig;

    using utility::input::ServoID;
    using utility::platform::darwin::getDarwinServo;
    using utility::vision::fourcc;

    // Converts the NUgus.proto servo name to the equivalent DarwinSensor.proto name
    DarwinSensors::Servo& translate_servo_id(const std::string& name, DarwinSensors::Servos& servos) {

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

    ActuatorRequests make_acutator_request(const ServoTargets& commands, const DarwinSensors& sensors) {
        message::platform::webots::ActuatorRequests to_send_next;

        // Convert the servo targets to the ActuatorRequests
        for (const auto& target : commands.targets) {
            MotorPosition position_msg;
            position_msg.name     = target.id;
            position_msg.position = target.position;
            to_send_next.motor_positions.push_back(position_msg);

            MotorVelocity velocity_msg;
            // We need to calculate the servo velocity to add to the velocity message
            // (method stolen from HardwareIO.cpp)
            // velocity = distance / time
            const float distance =
                utility::math::angle::difference(target.position, getDarwinServo(target.id, sensors).present_position);

            NUClear::clock::duration time = target.time - NUClear::clock::now();
            float velocity;
            if (time.count() > 0) {
                velocity = distance / static_cast<float>(time.count()) * static_cast<float>(NUClear::clock::period::num)
                           / static_cast<float>(NUClear::clock::period::den);
            }
            else {
                velocity = 0;
            }
            velocity_msg.name     = target.id;
            velocity_msg.velocity = velocity;
            to_send_next.motor_velocities.push_back(velocity_msg);

            MotorTorque torque_msg;
            torque_msg.name   = target.id;
            torque_msg.torque = target.torque;
            to_send_next.motor_torques.push_back(torque_msg);

            // MotorPID, only sending P gain. Set I and D to zero
            MotorPID motorpid_msg;
            motorpid_msg.name  = target.id;
            motorpid_msg.PID.X = static_cast<double>(target.gain);
            motorpid_msg.PID.Y = 0.0;
            motorpid_msg.PID.Z = 0.0;
            to_send_next.motor_pids.push_back(motorpid_msg);
        }
        return to_send_next;
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
            int fd = socket(addr_ptr->ai_family, addr_ptr->ai_socktype, addr_ptr->ai_protocol);

            if (fd == -1) {
                // Bad fd
                continue;
            }
            else if (connect(fd, addr_ptr->ai_addr, addr_ptr->ai_addrlen) != -1) {
                // Connection successful
                freeaddrinfo(address);
                return fd;
            }
            // Connection was not successful
            close(fd);
        }

        // No connection was successful
        freeaddrinfo(address);
        log<NUClear::ERROR>(fmt::format("Cannot connect to server: {}:{}", server_name, port));
        return -1;
    }

    Webots::Webots(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Trigger<GlobalConfig>, Configuration>("webots.yaml")
            .then([this](const GlobalConfig& /*global_config*/, const Configuration& local_config) {
                // Use configuration here from file webots.yaml

                // clang-format off
                auto lvl = local_config["log_level"].as<std::string>();
                if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
                else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
                else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
                else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
                else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
                else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
                // clang-format on

                on<Watchdog<Webots, 5, std::chrono::seconds>>().then([this, local_config] {
                    // We haven't received any messages lately
                    setup_connection(local_config["server_address"].as<std::string>(),
                                     local_config["port"].as<std::string>());
                });

                setup_connection(local_config["server_address"].as<std::string>(),
                                 local_config["port"].as<std::string>());
            });
    }

    void Webots::setup_connection(const std::string& server_address, const std::string& port) {
        // Unbind any previous reaction handles
        read_io.unbind();
        send_loop.unbind();
        error_io.unbind();
        shutdown_handle.unbind();

        if (fd != -1) {
            // Disconnect the fd gracefully
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }

        fd = tcpip_connect(server_address, port);

        std::string initial_message;
        const int n = recv(fd, initial_message.data(), sizeof(initial_message), 0);

        if (n > 0) {
            if (initial_message == "Welcome") {
                // good
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
            log<NUClear::FATAL>("Connection was closed.");
        }

        // Set the real time of the connection initiation
        connect_time = NUClear::clock::now();
        // Reset the simulation connection time
        utility::clock::last_update = std::chrono::steady_clock::now();

        log<NUClear::INFO>(fmt::format("Connected to {}:{}", server_address, port));

        // Now that we are connected, we can set up our reaction handles with this file descriptor

        // Receiving
        read_io =
            on<IO, With<ServoTargets>, With<DarwinSensors>>(fd, IO::READ)
                .then([this](const ServoTargets& commands, const DarwinSensors& sensors) {
                    // ************************** Receiving ***************************************
                    // Get the size of the message
                    uint32_t Nn;
                    if (recv(fd, &Nn, sizeof(Nn), 0 != sizeof(Nn))) {
                        log<NUClear::ERROR>("Failed to read message size from TCP connection");
                        return;
                    }

                    // Convert from network endian to host endian
                    const uint32_t Nh = ntohl(Nn);

                    // Get the message
                    std::vector<char> data(Nh, 0);
                    if (uint64_t(recv(fd, data.data(), Nh, 0)) != Nh) {
                        log<NUClear::ERROR>("Failed to read message from TCP connection");
                        return;
                    }

                    // Deserialise the message into a neutron
                    SensorMeasurements msg = NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(data);

                    translate_and_emit_sensor(msg);

                    // Service the watchdog
                    emit<Scope::WATCHDOG>(ServiceWatchdog<Webots>());

                    // ****************************** SENDING **************************************

                    // Deal with time

                    // Save our previous deltas
                    const uint32_t prev_sim_delta  = sim_delta;
                    const uint32_t prev_real_delta = real_delta;

                    // Update our current deltas
                    sim_delta  = msg.time - current_sim_time;
                    real_delta = msg.real_time - current_real_time;

                    // Calculate our custom rtf - the ratio of the past two sim deltas and the past two real time deltas
                    utility::clock::custom_rtf = static_cast<double>(sim_delta + prev_sim_delta)
                                                 / static_cast<double>(real_delta + prev_real_delta);

                    // Update our current times
                    current_sim_time  = msg.time;
                    current_real_time = msg.real_time;

                    ActuatorRequests to_send_now = make_acutator_request(commands, sensors);

                    // Sending
                    data = NUClear::util::serialise::Serialise<ActuatorRequests>::serialise(to_send_now);
                    // Size of the message, in network endian
                    Nn = htonl(data.size());
                    // Send the message size first
                    if (send(fd, &Nn, sizeof(Nn), 0) == sizeof(Nn)) {
                        log<NUClear::ERROR>(
                            fmt::format("Error in sending ActuatorRequests' message size,  {}", strerror(errno)));
                    }
                    // then send the data
                    if (send(fd, data.data(), Nn, 0) == Nn) {
                        log<NUClear::ERROR>(fmt::format("Error sending ActuatorRequests message, {}", strerror(errno)));
                    }
                });

        error_io = on<IO>(fd, IO::CLOSE | IO::ERROR).then([this, server_address, port](const IO::Event& /*event*/) {
            // Something went wrong, reopen the connection
            setup_connection(server_address, port);
        });

        shutdown_handle = on<Shutdown>().then([this] {
            // Disconnect the fd gracefully
            if (fd != -1) {
                shutdown(fd, SHUT_RDWR);
                close(fd);
                fd = -1;
            }
        });
    }

    void Webots::translate_and_emit_sensor(const SensorMeasurements& sensor_measurements) {
        // Read each field of msg, translate it to our protobuf and emit the data
        auto sensor_data = std::make_unique<DarwinSensors>();

        sensor_data->timestamp = NUClear::clock::now();

        for (const auto& position : sensor_measurements.position_sensors) {
            translate_servo_id(position.name, sensor_data->servo).present_position = position.value;
        }

        // TODO(KipHamiltons or ANYONE who can test!!) We need to work out what to do with these. At the moment, these
        // loops just overwrite the same values each iteration. We should test and see what we need to do ASAP!!
        for (const auto& accelerometer : sensor_measurements.accelerometers) {
            sensor_data->accelerometer.x = static_cast<float>(accelerometer.value.X);
            sensor_data->accelerometer.y = static_cast<float>(accelerometer.value.Y);
            sensor_data->accelerometer.z = static_cast<float>(accelerometer.value.Z);
        }

        for (const auto& gyro : sensor_measurements.gyros) {
            sensor_data->gyroscope.x = static_cast<float>(gyro.value.X);
            sensor_data->gyroscope.y = static_cast<float>(gyro.value.Y);
            sensor_data->gyroscope.z = static_cast<float>(gyro.value.Z);
        }

        // Ignore as we don't have physical functionality for this
        /*
        for (const auto& bumper : sensor_measurements.bumpers) {
            // string name
            // bool value
        }

        for (const auto& force_3d : sensor_measurements.force3ds) {
            // string name
            // Vector3 value
        }

        for (const auto& force_6d : sensor_measurements.force6ds) {
            // string name
            // Vector3 value
        }
        */

        emit(sensor_data);

        for (const auto& camera : sensor_measurements.cameras) {
            // Convert the incoming image so we can emit it to the PowerPlant.
            auto compressed_image            = std::make_unique<CompressedImage>();
            compressed_image->name           = camera.name;
            compressed_image->dimensions.x() = camera.width;
            compressed_image->dimensions.y() = camera.height;
            compressed_image->format         = fourcc("JPEG");
            compressed_image->data           = camera.image;
            emit(compressed_image);
        }

        // Parse the errors and warnings from Webots and log them.
        // Note that this is where we should deal with specific messages passed in the sensor_measurements "messages"
        // or check if those messages have specific information
        for (const auto& message : sensor_measurements.messages) {
            switch (int(message.message_type)) {
                case Message::MessageType::ERROR_MESSAGE: log<NUClear::ERROR>(message.text); break;
                case Message::MessageType::WARNING_MESSAGE: log<NUClear::WARN>(message.text); break;
            }
        }
    }
}  // namespace module::platform::webots
