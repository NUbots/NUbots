#include "webots.hpp"

#include <fmt/format>

#include "extension/Configuration.hpp"

#include "message/motion/ServoTarget.proto"
#include "message/platform/darwin/DarwinSensors.proto"

namespace module::platform {

using extension::Configuration;
using message::motion::ServoTargets;
using message::platform::darwin::DarwinSensors;

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

webots::webots(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {

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

        send_connect(fd, cfg["teamId"].as<int>(), cfg["robotId"].as<int>());

        on<IO>(fd).then([this]() {
            // Receiveing

            uint64_t N;
            // Somehow work out how big the message was

            std::vector<char> data(N, 0);
            if (recv(fd, data.data(), N, 0) != N) {
                log<NUClear::ERROR>("Error: Failed to read message from TCP connection");
                return;
            }

            SensorMeasurements msg = NUClear::util::serialise::Serialise<SensorMeasurements>.deserialise(data);

            // Read each field of msg, translate it to our protobuf and emit the data
            auto sensor_data = std::make_unique<DarwinSensors>();
            sensor_data.FromSeconds(msg.time);

            for (auto& position : position_sensor) {
                //
            }

            emit(sensor_data);
        });

        on<Every<1, std::chrono::seconds>>().then([this]() {
            // Sending
            std::vector<char> data = NUClear::util::serialise::Serialise<ActingMessage>.serialise(to_send);
            uint64_t N             = data.size();
            send(fd, data.data(), N, 0);
        });
    });

    // Create the message that we are going to send.
    on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
        // Maybe keep the `ServoTarget`s we have not sent yet, instead of just overriding them.
        to_send = ActuatorRequests();
        for (auto& command : commands) {
            MotorPosition position_msg to_send.add_motor_position();
            position_msg.name     = command.id;
            position_msg.position = command.position;

            // TODO work out if gain is velocity or force
            MotorVelocity velocity_msg = to_send.add_motor_velocity();
            velocity_msg.name          = command.id;
            velocity_msg.velocity      = command.gain;

            MotorTorque torque_msg = to_send.add_motor_torque();
            torque_msg.name        = command.id;
            torque_msg.torque      = command.torque;
        }
    });
}

}  // namespace module::platform
