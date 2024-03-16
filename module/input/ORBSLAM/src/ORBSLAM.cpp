#include "ORBSLAM.hpp"

#include "extension/Configuration.hpp"

namespace module::input {

    using extension::Configuration;

    ORBSLAM::ORBSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ORBSLAM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ORBSLAM.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            ORB_SLAM3::System SLAM("", "", ORB_SLAM3::System::IMU_MONOCULAR, true);
        });
    }

}  // namespace module::input
