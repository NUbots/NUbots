#include "FieldLocalisationSRIF.hpp"

#include "extension/Configuration.hpp"

namespace module::localisation {
    using extension::Configuration;

    FieldLocalisationSRIF::FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLocalisationSRIF.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

        }

    }  // namespace module::localisation
