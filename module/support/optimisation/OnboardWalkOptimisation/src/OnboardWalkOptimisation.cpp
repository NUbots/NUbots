#include "OnboardWalkOptimisation.hpp"

#include "extension/Configuration.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/input/Sensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"

namespace module::support::optimisation {

using extension::Configuration;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;
    using message::platform::RawSensors;
    using message::platform::webots::OptimisationCommand;
    using message::input::Sensors;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonLeftDown;
    // using message::platform::webots::OptimisationRobotPosition;
    // using message::support::optimisation::NSGA2EvaluationRequest;
    // using message::support::optimisation::NSGA2EvaluatorReadinessQuery;
    // using message::support::optimisation::NSGA2EvaluatorReady;
    // using message::support::optimisation::NSGA2FitnessScores;
    // using message::support::optimisation::NSGA2Terminate;
    // using message::support::optimisation::NSGA2TrialExpired;
    // using message::support::optimisation::OptimisationResetDone;
    // using message::support::optimisation::OptimisationTimeUpdate;

    // using utility::behaviour::RegisterAction;
    // using utility::input::LimbID;
    // using utility::input::ServoID;
    // using utility::support::Expression;

OnboardWalkOptimisation::OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("OnboardWalkOptimisation.yaml").then([this](const Configuration& config) {
        // Use configuration here from file OnboardWalkOptimisation.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();

        cfg.fallen_angle = config["fallen_angle"].as<float>();

        gravity_max = config["gravity"]["max"].as<float>();
        gravity_min = config["gravity"]["min"].as<float>();
    });

    on<Trigger<OptimisationCommand>>().then([this](const OptimisationCommand& msg) {
        const int msg_command = msg.command;
        switch (msg_command) {
            case OptimisationCommand::CommandType::RESET_WORLD:
                // Set the reset world flag to send the reset command to webots with the next ActuatorRequests
                NUClear::log<NUClear::DEBUG>("Reset world");
                reset_simulation_world = true;

                break;

            case OptimisationCommand::CommandType::RESET_TIME:
                // Set the reset flag to send the reset command to webots with the next ActuatorRequests
                NUClear::log<NUClear::DEBUG>("Reset time");
                reset_simulation_time = true;
                break;

            case OptimisationCommand::CommandType::TERMINATE:
                // Set the termination flag to send the terminate command to webots with the next ActuatorRequests
                NUClear::log<NUClear::DEBUG>("Terminate");
                terminate_simulation = true;
                break;
        }
    });

    on<Every<10, std::chrono::seconds>>().then([this]() {
        emit(std::make_unique<OptimisationResetDone>());
    });

    on<Trigger<Sensors>>().then("Optimisation Fallen Check", [this](const Sensors& sensors) {
        // auto accelerometer = sensors.accelerometer;

        // Transform to torso {t} from world {w} space
        Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
        // Basis Z vector of torso {t} in world {w} space
        Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);
        // Basis X vector of torso {t} in world {w} space
        // Eigen::Vector3d uXTw = Hwt.block(0, 0, 3, 1);

        // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
        if (std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.fallen_angle) {
            NUClear::log<NUClear::DEBUG>("FALLEN!");
            // evaluator->emit(std::make_unique<NSGA2Evaluator::Event>(NSGA2Evaluator::Event::TERMINATE_EARLY));
        }
    });

    // Reset for next run or terminate when paused
    on<Trigger<ButtonLeftDown>, Single>().then([this] {
        NUClear::log<NUClear::DEBUG>("Left Button Pressed");
        if ()
            emit(std::make_unique<OptimisationResetDone>());
    });

    // Pause optimisation
    on<Trigger<ButtonMiddleDown>, Single>().then([this] {

    });
}
}  // namespace module::support::optimisation
