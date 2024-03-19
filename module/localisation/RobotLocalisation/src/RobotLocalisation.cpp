#include "RobotLocalisation.hpp"

#include "extension/Configuration.hpp"

namespace module::localisation {

using extension::Configuration;

RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
        // Use configuration here from file RobotLocalisation.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::localisation
