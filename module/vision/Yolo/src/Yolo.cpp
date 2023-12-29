#include "Yolo.hpp"

#include "extension/Configuration.hpp"

namespace module::vision {

using extension::Configuration;

Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Yolo.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::vision
