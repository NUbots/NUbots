#include "MCPServer.hpp"

#include "extension/Configuration.hpp"

namespace module::network {

using extension::Configuration;

MCPServer::MCPServer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("MCPServer.yaml").then([this](const Configuration& config) {
        // Use configuration here from file MCPServer.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::network
