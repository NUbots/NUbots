#include "Gazebo.h"

#include "extension/Configuration.h"

#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/platform/gazebo/Ball.h"
#include "message/platform/gazebo/Command.h"
#include "message/platform/gazebo/RawSensors.h"
#include "message/platform/gazebo/ServoTargets.h"
#include "message/platform/gazebo/Simulation.h"
#include "message/platform/gazebo/Torso.h"

#include "utility/clock/CustomClock.h"

namespace module {
namespace platform {

    using extension::Configuration;

    using message::motion::ServoTarget;
    using message::motion::ServoTargets;
    using message::platform::darwin::DarwinSensors;
    using message::platform::gazebo::Ball;
    using message::platform::gazebo::Command;
    using message::platform::gazebo::RawSensors;
    using GazeboTargets = message::platform::gazebo::ServoTargets;
    using message::platform::gazebo::Simulation;
    using message::platform::gazebo::Torso;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), sim_time(0), real_time(0) {

        on<Configuration>("Gazebo.yaml").then([this](const Configuration& cfg) {
            config.clock_smoothing = cfg["clock_smoothing"];
            config.simulator_name  = cfg["simulator_name"].as<std::string>();
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& event) {
            // Reset the world and the simulation time on startup
            auto msg  = std::make_unique<Command>();
            msg->type = Command::Type::RESET;
            emit<Scope::NETWORK>(msg, event.name, false);
            msg       = std::make_unique<Command>();
            msg->type = Command::Type::RESET_TIME;
            emit<Scope::NETWORK>(msg, event.name, false);
        });

        on<Network<Ball>>().then([this](const NetworkSource& network_source, const Ball& simulation) {});

        on<Network<Simulation>>().then([this](const NetworkSource& network_source, const Simulation& simulation) {
            if (network_source.name == config.simulator_name) {
                if (real_time != 0 || sim_time != 0) {
                    double real_delta          = simulation.real_time - real_time;
                    double sim_delta           = simulation.simulation_time - sim_time;
                    utility::clock::custom_rtf = utility::clock::custom_rtf * config.clock_smoothing
                                                 + (1.0 - config.clock_smoothing) * (real_delta / sim_delta);
                }

                sim_time  = simulation.simulation_time;
                real_time = simulation.real_time;
            }
        });

        on<Network<RawSensors>>().then([this](const NetworkSource& network_source, const RawSensors& sensors) {
            // Swizzle the IMU axes so that they match the CM740
            DarwinSensors msg(sensors.sensors);
            msg.accelerometer = {-msg.accelerometer.x, msg.accelerometer.y, -msg.accelerometer.z};
            msg.gyroscope     = {msg.gyroscope.y, msg.gyroscope.x, -msg.gyroscope.z};
            emit(std::make_unique<DarwinSensors>(msg));
        });

        on<Network<Torso>>().then([this](const NetworkSource& network_source, const Torso& sensors) {});

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto msg = std::make_unique<ServoTargets>();
            msg->targets.push_back(command);
            emit(msg);
        });

        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            auto msg     = std::make_unique<GazeboTargets>();
            msg->model   = "nugus";
            msg->time    = NUClear::clock::now();
            msg->targets = commands;
            emit<Scope::NETWORK>(msg, config.simulator_name, false);
        });
    }
}  // namespace platform
}  // namespace module
