#include "Voronoi.hpp"

#include "extension/Configuration.hpp"

namespace module {

using extension::Configuration;

Voronoi::Voronoi(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("Voronoi.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Voronoi.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module
