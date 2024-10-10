#include "Judge.hpp"

#include "extension/Configuration.hpp"

namespace module::onboarding {

using extension::Configuration;

Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Judge.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::onboarding
