#include "Gazebo.hpp"

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/motion/ServoTarget.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/gazebo/Ball.hpp"
#include "message/platform/gazebo/Command.hpp"
#include "message/platform/gazebo/RawSensors.hpp"
#include "message/platform/gazebo/ServoTargets.hpp"
#include "message/platform/gazebo/Simulation.hpp"
#include "message/platform/gazebo/Torso.hpp"

namespace module::platform {

    using extension::Configuration;

    using message::motion::ServoTarget;
    using message::motion::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::gazebo::Command;
    using GazeboSensors = message::platform::gazebo::RawSensors;
    using GazeboTargets = message::platform::gazebo::ServoTargets;
    using message::platform::gazebo::Simulation;
    using message::platform::gazebo::Torso;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), sim_time(0), real_time(0) {

        on<Configuration>("Gazebo.yaml").then([this](const Configuration& cfg) {
            config.clock_smoothing = cfg["clock_smoothing"];
            config.simulator_name  = cfg["simulator_name"].as<std::string>();
            config.model_name      = cfg["model_name"].as<std::string>();
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

        on<Network<Simulation>>().then([this](const NetworkSource& network_source, const Simulation& simulation) {
            // Only listen to our target simulation
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

        on<Network<GazeboSensors>>().then([this](const NetworkSource& network_source, const GazeboSensors& sensors) {
            // Only listen to the target model in the target simulation
            if (network_source.name == config.simulator_name && sensors.model == config.model_name) {
                // Swizzle the IMU axes so that they match the CM740
                RawSensors msg(sensors.sensors);
                msg.accelerometer = -msg.accelerometer;
                msg.gyroscope     = Eigen::Vector3f(-msg.gyroscope.x(), -msg.gyroscope.y(), msg.gyroscope.z());
                emit(std::make_unique<RawSensors>(msg));
            }
        });

        on<Network<Torso>>().then([this](const NetworkSource& network_source, const Torso& sensors) {
            if (network_source.name == config.simulator_name && sensors.model == config.model_name) {
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto msg = std::make_unique<ServoTargets>();
            msg->targets.push_back(command);
            emit(msg);
        });

        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            auto msg     = std::make_unique<GazeboTargets>();
            msg->model   = config.model_name;
            msg->time    = NUClear::clock::now();
            msg->targets = commands;
            emit<Scope::NETWORK>(msg, config.simulator_name, false);
        });
    }
}  // namespace module::platform
