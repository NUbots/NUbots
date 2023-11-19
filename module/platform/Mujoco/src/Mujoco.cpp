#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

namespace module::platform {

    using extension::Configuration;

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then("Start Mujoco", [this] {
            // Start Mujoco world
            log<NUClear::INFO>("Mujoco started");
        });


        on<Trigger<Sensors>>().then("Update Mujoco", [this](const Sensors& sensors) {
            // TODO: Update Mujoco world with sensor data, object positions, etc.
            log<NUClear::INFO>("Mujoco updated");
        });
    }

}  // namespace module::platform
