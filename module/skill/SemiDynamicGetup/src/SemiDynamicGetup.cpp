#include "SemiDynamicGetup.hpp"

#include "extension/Configuration.hpp"

namespace module::skill {

using extension::Configuration;

SemiDynamicGetup::SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("SemiDynamicGetup.yaml").then([this](const Configuration& config) {
        // Use configuration here from file SemiDynamicGetup.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::skill
