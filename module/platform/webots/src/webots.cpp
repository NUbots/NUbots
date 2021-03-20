#include "webots.hpp"

#include <sys/socket.h>

#include "extension/Configuration.hpp"

namespace module::platform {

using extension::Configuration;

namespace {
    // Should probs go in shared
    int tcpip_connect(int port, uint32_t ip) {
        // Create the socket
        int fd = socket(AF_INET, SOCK_STREAM, 0);

        // Store the ip address information that we will connect to
        sockaddr_in address;
        memset(&address, 0, sizeof(sockaddr_in));
        address.sin_family      = AF_INET;
        address.sin_port        = htons(port);
        address.sin_addr.s_addr = htonl(ip));

        // Connect to the ip address
        if (int error = connect(fd, address, size(address))) {
            // Throw error
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

        int fd = tcpip_connect(cfg["port"].as<int>(), cfg["ip"].as<uint32_t>());

        send_connect(fd, cfg["teamId"].as<int>(), cfg["robotId"].as<int>());

        on<IO>(fd).then([this]() {
            // Receiveing
        });

        on<Always>().then([this]() {
            // Sending
        });
    });
}

}  // namespace module::platform
