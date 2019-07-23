#include "Gazebo.h"

#include "extension/Configuration.h"
#include "message/input/gazebo/Simulation.h"

namespace module {
namespace platform {

    using extension::Configuration;
    using message::input::gazebo::Simulation;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Gazebo.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Gazebo.yaml
        });

        on<Network<Simulation>>().then([this] (const NetworkSource& network_source, const Simulation& simulation) {
            log(network_source.name, simulation.sim_time, simulation.real_time);
        });
    }
}
}
