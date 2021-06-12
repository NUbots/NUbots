#include "FusionFilter.hpp"

#include "extension/Configuration.hpp"

namespace module::input {

using extension::Configuration;

FusionFilter::FusionFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {

    on<Configuration>("FusionFilter.yaml").then([this](const Configuration& cfg) {
        // Use configuration here from file FusionFilter.yaml
        this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::input
