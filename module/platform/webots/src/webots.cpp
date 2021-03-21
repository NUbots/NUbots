#include "webots.hpp"

#include <fmt/format>

#include "extension/Configuration.hpp"


namespace module::platform {

using extension::Configuration;

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

            if (msg.ParseFromArray(data.data(), N)) {
                log<NUClear::ERROR>("Error: Failed to parse serialised message");
                return;
            }

            // Read each field of msg, translate it to our protobuf and emit the data
        });

        on<Every<1, std::chrono::seconds>>().then([this]() {
            // Sending
            std::vector<char> data = NUClear::util::serialise::Serialise<ActingMessage>.serialise(to_send);
            uint64_t N             = data.size();
            send(fd, data.data(), N, 0);
        });
    });
}

}  // namespace module::platform
