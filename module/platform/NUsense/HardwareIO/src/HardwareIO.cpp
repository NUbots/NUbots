#include "HardwareIO.hpp"

#include "extension/Configuration.hpp"

namespace module::platform::NUsense {

using extension::Configuration;

HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {

    on<Configuration>("HardwareIO.yaml").then([this](const Configuration& cfg) {
        // Use configuration here from file HardwareIO.yaml
        this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::platform::NUsense
