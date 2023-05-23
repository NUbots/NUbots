#include "LocalisationSimulator.hpp"

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"

namespace module::support {

    using extension::Configuration;
    using message::localisation::Field;

    LocalisationSimulator::LocalisationSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("LocalisationSimulator.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file LocalisationSimulator.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        on<Every<3, Per<std::chrono::seconds>>, With<Field>>().then("Heart Beat",
                                                                    [this](const Field& f) { log(f.Hfw); });
    }

}  // namespace module::support
