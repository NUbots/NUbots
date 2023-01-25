#include "Localisation.hpp"

#include "extension/Configuration.hpp"

namespace module::localisation {

using extension::Configuration;

Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Localisation.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Localisation.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::localisation
