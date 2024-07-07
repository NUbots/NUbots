#include "AllRounder.hpp"

#include "extension/Configuration.hpp"

namespace module::purpose {

using extension::Configuration;

AllRounder::AllRounder(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("AllRounder.yaml").then([this](const Configuration& config) {
        // Use configuration here from file AllRounder.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::purpose
