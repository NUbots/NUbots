#include "Segmentation.hpp"

#include "extension/Configuration.hpp"

namespace module::vision {

using extension::Configuration;

Segmentation::Segmentation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Segmentation.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Segmentation.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::vision
