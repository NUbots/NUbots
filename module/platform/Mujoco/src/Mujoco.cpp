#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

namespace module::platform {

using extension::Configuration;

Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Mujoco.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::platform
