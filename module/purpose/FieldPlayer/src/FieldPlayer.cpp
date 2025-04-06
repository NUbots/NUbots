#include "FieldPlayer.hpp"

#include "extension/Configuration.hpp"

namespace module {

using extension::Configuration;

FieldPlayer::FieldPlayer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("FieldPlayer.yaml").then([this](const Configuration& config) {
        // Use configuration here from file FieldPlayer.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module
