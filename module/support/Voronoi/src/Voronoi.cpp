#include "Voronoi.hpp"

#include "message/platform/webots/messages.hpp"

#include "extension/Configuration.hpp"

namespace module::support {

using extension::Configuration;
using message::platform::webots::SensorMeasurements;
using message::platform::webots::LocalisationGroundTruth;

Voronoi::Voronoi(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Voronoi.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Voronoi.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<LocalisationGroundTruth>>().then([this](const LocalisationGroundTruth lgt) {
        NUClear::log<NUClear::DEBUG>("Robot Positions ", lgt.x() );

    });
}

}  // namespace module::support
