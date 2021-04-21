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

#include "extension/Configuration.hpp"

#include "message/motion/ServoTarget.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/platform/darwin/DarwinSensors.hpp"
#include "message/platform/webots/ConnectRequest.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/GlobalConfig.hpp"

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

using extension::Configuration;
using message::motion::ServoTargets;
using message::output::CompressedImage;
using message::platform::darwin::DarwinSensors;

using message::platform::webots::ActuatorRequests;
using message::platform::webots::ConnectRequest;
using message::platform::webots::Message;
using message::platform::webots::MotorPosition;
using message::platform::webots::MotorTorque;
using message::platform::webots::MotorVelocity;
using message::platform::webots::SensorMeasurements;

using message::support::GlobalConfig;
using utility::vision::fourcc;

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
        .then([this](const GlobalConfig& global_config, const Configuration& local_config) {
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

            on<Watchdog<Webots, 5, std::chrono::seconds>>().then([this] {
                // We haven't received any messages lately
            });

            setup_connection(local_config["server_address"].as<std::string>(), local_config["port"].as<std::string>());
        });

    // Create the message that we are going to send.
    on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
        // Maybe keep the `ServoTarget`s we have not sent yet, instead of just overriding them.
        to_send.motor_positions.clear();
        to_send.motor_velocities.clear();
        to_send.motor_torques.clear();

        // Store each ServoTarget to send in the next lot
        for (const auto& command : commands.targets) {
            MotorPosition position_msg;
            position_msg.name     = command.id;
            position_msg.position = command.position;
            to_send.motor_positions.push_back(position_msg);

            // TODO(cmurtagh) work out if gain is velocity or force
            MotorVelocity velocity_msg;
            velocity_msg.name     = command.id;
            velocity_msg.velocity = command.gain;
            to_send.motor_velocities.push_back(velocity_msg);

            MotorTorque torque_msg;
            torque_msg.name   = command.id;
            torque_msg.torque = command.torque;
            to_send.motor_torques.push_back(torque_msg);

            // MotorPID ? Do we need to send this?
        }
    });
}

void Webots::setup_connection(const std::string& server_address, const std::string& port) {
    // Unbind the
    read_io.unbind();
    send_loop.unbind();
    error_io.unbind();
    shutdown_handle.unbind();

    int fd = tcpip_connect(server_address, port);

    char inital_message[8];
    int n = recv(fd, inital_message, sizeof(inital_message), 0);

    if(n > 0){
        if (strncmp(inital_message, "Welcome", sizeof(inital_message)) == 0){
            //good
        }
        else if (strncmp(inital_message, "Refused", sizeof(inital_message)) == 0){
            log<NUClear::FATAL>(fmt::format("Connection to {}:{} refused: your ip is not white listed.", server_address, port));
            // Halt and don't retry as reconnection is pointless.
            close(fd);
            powerplant.shutdown();
        }
        else {
            log<NUClear::FATAL>(fmt::format("{}:{} sent unknown initial message", server_address, port));
            // Halt and don't retry
            close(fd);
            powerplant.shutdown();
        }
    }
    else {
        log<NUClear::FATAL>("Connection was closed.");
    }

    connect_time = NUClear::clock::now();

    log<NUClear::INFO>(fmt::format("Connected to {}:{}", server_address, port));

    // Tell webots who we are
    // send_player_details(fd, global_config); Not needed? Set by the port we connect to?

    read_io = on<IO>(fd, IO::READ).then([this, fd]() {
        // Receiving

        // Get the size of the message
        uint32_t Nn;
        if (recv(fd, &Nn, sizeof(Nn), 0 != sizeof(Nn))) {
            log<NUClear::ERROR>("Failed to read message size from TCP connection");
            return;
        }

        uint32_t Nh = ntohl(Nn);  // Convert from network endian to host endian

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

        // Tick the clock forward
        Clock.tick();
    });

    send_loop = on<Every<10, std::chrono::milliseconds>>().then([this, fd]() {
        // Sending
        std::vector<char> data = NUClear::util::serialise::Serialise<ActuatorRequests>::serialise(to_send);
        uint32_t Nn            = htonl(data.size());
        if (send(fd, &Nn, sizeof(Nn), 0) == sizeof(Nn)){
            log<NUClear::ERROR>(fmt::format("Error in sending actuator requests message size, {}", strerror(errno)));
        }
        if(send(fd, data.data(), Nn, 0) == Nn){
            log<NUClear::ERROR>("Error in sending actuator requests message, {}", strerror(errno));
        }
    });

    error_io = on<IO>(fd, IO::CLOSE | IO::ERROR).then([this, fd](const IO::Event& event) {
        // Something went wrong
    });

    shutdown_handle = on<Shutdown>().then([this, fd] {
        // Disconnect the fd gracefully
        shutdown(fd, SHUT_RDWR);
        close(fd);
    });
}

void Webots::send_player_details(const int& fd, const GlobalConfig& player_details) {
    // TODO(cameron) resend if failes
    std::vector<char> data = NUClear::util::serialise::Serialise<GlobalConfig>::serialise(player_details);
    uint32_t Nn            = htonl(data.size());
    send(fd, &Nn, sizeof(Nn), 0);
    send(fd, data.data(), Nn, 0);
}

void Webots::translate_and_emit_sensor(const SensorMeasurements& sensor_measurements) {
    // Read each field of msg, translate it to our protobuf and emit the data
    auto sensor_data = std::make_unique<DarwinSensors>();

    //TODO(cameron) Work out if it makes sense to combine simulation time with real time.
    sensor_data->timestamp = NUClear::clock::now();
        //connect_time + std::chrono::milliseconds(sensor_measurements.time);

    // Vector3 is a neutron of 3 doubles

    for (const auto& position : sensor_measurements.position_sensors) {
        // string name
        // double value
    }

    for (const auto& accelerometer : sensor_measurements.accelerometers) {
        // string name
        // Vector3 value
    }

    for (const auto& gyro : sensor_measurements.gyros) {
        // string name
        // Vector3 value
    }

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

    // Parse the errors and warnings from Webots and log them. Maybe check for certain things.
    for (const auto& message : sensor_measurements.messages) {
        switch (int(message.message_type)) {
            case Message::MessageType::ERROR_MESSAGE: log<NUClear::ERROR>(message.text); break;
            case Message::MessageType::WARNING_MESSAGE: log<NUClear::WARN>(message.text); break;
        }
    }
}

}  // namespace module::platform
