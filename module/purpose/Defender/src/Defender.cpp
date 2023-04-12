#include "Defender.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::purpose {

using extension::Configuration;

Defender::Defender(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

    on<Configuration>("Defender.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Defender.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::purpose
