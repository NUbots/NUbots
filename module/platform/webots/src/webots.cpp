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

#include "webots.hpp"

#include <fmt/format>

#include "extension/Configuration.hpp"

#include "message/motion/ServoTarget.proto"
#include "message/output/CompressedImage.proto"
#include "message/platform/darwin/DarwinSensors.proto"
#include "message/platform/webots/ConnectRequest.proto"
#include "message/platform/webots/messages.proto"

namespace module::platform {

using extension::Configuration;
using message::motion::ServoTargets;
using message::platform::darwin::DarwinSensors;
using msessage::output::CompressedImage;
// TODO(cmurtagh) work out namespace of protobuf files from webots

namespace {
    // Should probs go in shared
    int tcpip_connect(const std::string& server_address, const int& port) {
        // Create the socket
        int fd = socket(AF_INET, SOCK_STREAM, 0);

        // Store the ip address information that we will connect to
        sockaddr_in address;
        memset(&address, 0, sizeof(sockaddr_in));
        address.sin_family = AF_INET;
        address.sin_port   = htons(port);
        hostent* server    = gethostbyname(server_address);

        // Check that dns was successful
        if (server) {
            std::memcpy(reinterpret_cast<char*>(&address.sin_addr.s_addr),
                        reinterpret_cast<char*>(server->h_addr),
                        server->h_length);
        }
        else {
            close(fd);
            log<NUClear::ERROR>(fmt::format("Cannot resolve server name: {}", server_address));
            return -1;
        }

        // Connect to the ip address
        if (int error = connect(fd, address, size(address))) {
            close(fd);
            log<NUClear::ERROR>(fmt::format("Cannot connect server: {}", server_address));
            return -1;
        }

        return fd;
    }
}  // namespace

Webots::Webots(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {

    on<Configuration>("webots.yaml").then([this](const Configuration& cfg) {
        // Use configuration here from file webots.yaml

        // clang-format off
        auto lvl = cfg["log_level"].as<std::string>();
        if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
        else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
        else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
        else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
        else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
        else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
        // clang-format on

        int fd = tcpip_connect(cfg["port"].as<int>(), cfg["server_address"].as<std::string>());

        // Tell webots who we are
        send_connect(fd, cfg["team_id"].as<int>(), cfg["robot_id"].as<int>());

        on<IO>(fd).then([this]() {
            // Receiveing

            // Get the size of the message
            uint64_t N;
            if (recv(fd, &N, sizeof(N), 0 != sizeof(N))) {
                log<NUClear::ERROR>("Failed to read message size from TCP connection");
                return;
            }

            // Get the message
            std::vector<char> data(N, 0);
            if (recv(fd, data.data(), N, 0) != N) {
                log<NUClear::ERROR>("Error: Failed to read message from TCP connection");
                return;
            }

            // Deserialise the message into a neutron
            SensorMeasurements msg = NUClear::util::serialise::Serialise<SensorMeasurements>.deserialise(data);

            // Read each field of msg, translate it to our protobuf and emit the data
            auto sensor_data = std::make_unique<DarwinSensors>();
            sensor_data.FromSeconds(msg.time);

            for (auto& position : msg.position_sensor) {
                // string name
                // double valvue
            }

            for (auto& accelerometer : msg.accelerometer) {
                // string name
                // Vector3 value
            }

            for (auto& gyro : msg.gyro) {
                // string name
                // Vector3 value
            }

            for (auto& bumper : msg.bumper) {
                // string name
                // bool value
            }

            for (auto& force_3d : msg.force_3d) {
                // string name
                // Vector3 value
            }

            for (auto& force_6d : msg.force6d) {
                // string name
                // Vector3 value
            }

            emit(sensor_data);

            for (auto& camera : msg.camera) {
                // Convert the incoming image so we can emit it to the PowerPlant.
                auto compressed_image = std::make_unique<CompressedImage>();
                compressed_image->timestamp.FromSeconds(msg.time);
                compressed_image->name          = camera.name;
                compressed_image->dimensions.x  = camera.width;
                compressed_image->dimensions.y  = camera.height;
                compressed_image->format        = camera.quality;  // This is probably wrong, we havent documented :(
                compressed_image->data          = camera.data;
                compressed_image->lens.fov      = (float) camera.horizontalFieldOfView;
                compressed_image->lens.centre.x = (float) camera.centreX;
                compressed_image->lens.centre.y = (float) camera.centreY;
                // Radial coefficients
                // tangential coefficients
                emit(compressed_image);
            }
        });

        on<Every<1, std::chrono::seconds>>().then([this]() {
            // Sending
            std::vector<char> data = NUClear::util::serialise::Serialise<ActingMessage>.serialise(to_send);
            uint64_t N             = data.size();
            send(fd, &N, sizeof(N), 0);
            send(fd, data.data(), N, 0);
        });
    });

    // Create the message that we are going to send.
    on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
        // Maybe keep the `ServoTarget`s we have not sent yet, instead of just overriding them.
        to_send = ActuatorRequests();

        // Store each servotarget to send in the next lot
        for (auto& command : commands) {
            MotorPosition position_msg to_send.add_motor_position();
            position_msg.name     = command.id;
            position_msg.position = command.position;

            // TODO(cmurtagh) work out if gain is velocity or force
            MotorVelocity velocity_msg = to_send.add_motor_velocity();
            velocity_msg.name          = command.id;
            velocity_msg.velocity      = command.gain;

            MotorTorque torque_msg = to_send.add_motor_torque();
            torque_msg.name        = command.id;
            torque_msg.torque      = command.torque;

            // MotorPID ? Do we need to send this?
        }
    });
}

void Webots::send_connect(int& fd, int& team_id, int& robot_id) {
    // TODO(cameron) workout what to do if failes
    ConnectRequest connect_request = ConnectRequest;
    connect_request.teamId         = team_id;
    connect_request.playerId       = robot_id;

    std::vector<char> data = NUClear::util::Serialise<ConnectRequest>.serialise(connect_request);
    uint64_t N             = data.size();
    send(fd, &N, sizeof(N), 0);
    send(fd, data.data(), N, 0);
}

}  // namespace module::platform
