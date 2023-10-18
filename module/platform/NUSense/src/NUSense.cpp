#include "NUSense.hpp"

#include "extension/Configuration.hpp"

namespace module::platform {

using extension::Configuration;

NUSense::NUSense(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("NUSense.yaml").then([this](const Configuration& config) {
        // Use configuration here from file NUSense.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::platform
