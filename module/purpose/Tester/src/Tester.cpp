#include "Tester.hpp"

#include "extension/Configuration.hpp"

namespace module::purpose {

using extension::Configuration;

Tester::Tester(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Tester.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Tester.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::purpose
