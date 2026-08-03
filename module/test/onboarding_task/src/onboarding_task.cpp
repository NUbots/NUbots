#include "onboarding_task.hpp"

#include "extension/Configuration.hpp"

namespace module::test {

using extension::Configuration;

onboarding_task::onboarding_task(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("onboarding_task.yaml").then([this](const Configuration& config) {
        // Use configuration here from file onboarding_task.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::test
