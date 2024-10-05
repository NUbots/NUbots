#include "Task.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/Task.hpp"

namespace module::onboarding {

using extension::Configuration;

Task::Task(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Task.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Task.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::onboarding
