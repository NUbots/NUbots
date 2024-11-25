#include "testNod.hpp"

#include "extension/Configuration.hpp"

namespace module::obtask {

using extension::Configuration;

testNod::testNod(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("testNod.yaml").then([this](const Configuration& config) {
        // Use configuration here from file testNod.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::obtask
