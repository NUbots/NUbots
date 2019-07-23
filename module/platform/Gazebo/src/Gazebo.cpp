#include "Gazebo.h"

#include "extension/Configuration.h"
#include "message/input/gazebo/Simulation.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "utility/clock/CustomClock.h"

namespace module {
namespace platform {

    using extension::Configuration;
    using message::input::gazebo::Simulation;
    using message::platform::darwin::DarwinSensors;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), sim_time(0), real_time(0) {

        on<Configuration>("Gazebo.yaml").then([this] (const Configuration& cfg) {
            config.clock_smoothing = cfg["clock_smoothing"];
            config.simulator_name = cfg["simulator_name"].as<std::string>();
        });

        on<Network<Simulation>>().then([this] (const NetworkSource& network_source, const Simulation& simulation) {
            if (network_source.name == config.simulator_name) {
                if (real_time != 0 || sim_time != 0) {
                    double real_delta = simulation.real_time - real_time;
                    double sim_delta = simulation.sim_time - sim_time;
                    utility::clock::custom_rtf = utility::clock::custom_rtf * config.clock_smoothing +
                                                 (1.0 - config.clock_smoothing) * (real_delta / sim_delta);
                }

                sim_time = simulation.sim_time;
                real_time = simulation.real_time;
            }
        });

        on<Network<DarwinSensors>>().then([this] (const NetworkSource& network_source, const DarwinSensors& sensors) {
            emit(std::make_unique<DarwinSensors>(sensors));
        });
    }
}
}
