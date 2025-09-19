#include "Ping.hpp"

#include "extension/Configuration.hpp"

namespace module::onboard {

using extension::Configuration;

Ping::Ping(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Ping.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::onboard
