#include "ORBSLAM3.hpp"

#include "extension/Configuration.hpp"

namespace module::vision {

using extension::Configuration;

ORBSLAM3::ORBSLAM3(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("ORBSLAM3.yaml").then([this](const Configuration& config) {
        // Use configuration here from file ORBSLAM3.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::vision
