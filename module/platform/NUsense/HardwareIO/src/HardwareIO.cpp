#include "HardwareIO.hpp"

#include "NUsense/SIProcessor.hpp"

#include "extension/Configuration.hpp"

namespace module::platform::NUsense {

    using extension::Configuration;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HardwareIO.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.port        = config["port"].as<std::string>();
        });

        on<Startup>().then([this] {
            // Send some data to the serial using the header
        });
    }

}  // namespace module::platform::NUsense
