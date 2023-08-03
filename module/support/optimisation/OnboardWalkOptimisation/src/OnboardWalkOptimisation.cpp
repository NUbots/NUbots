#include "OnboardWalkOptimisation.hpp"

#include "extension/Configuration.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"

namespace module::support::optimisation {

using extension::Configuration;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;
    using message::platform::RawSensors;
    using message::platform::webots::OptimisationCommand;
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

    on<Startup>().then([this]() {

    });

    on<Every<10, std::chrono::seconds>>().then([this]() {
        emit(std::make_unique<OptimisationResetDone>());
    });
}

    // void OnboardWalkOptimisation::translate_and_emit_sensor(const SensorMeasurements& sensor_measurements) {
    //     // ****************************** TIME **************************************
    //     // Deal with time first

    //     // If our local sim time is non zero and we just got one that is zero, that means the simulation was reset
    //     // (which is something we do for the walk optimisation), so reset our local times
    //     if (sim_delta > 0 && sensor_measurements.time == 0) {
    //         log<NUClear::DEBUG>("Webots sim time reset to zero, resetting local sim_time. time before reset:",
    //                             current_sim_time);
    //         sim_delta         = 0;
    //         real_delta        = 0;
    //         current_sim_time  = 0;
    //         current_real_time = 0;

    //         // Reset the local raw sensors buffer
    //         emit(std::make_unique<ResetWebotsServos>());
    //     }

    //     // Create and emit the OptimisationRobotPosition message used by the walk optimiser
    //     auto robot_position   = std::make_unique<OptimisationRobotPosition>();
    //     robot_position->value = sensor_measurements.robot_position.value;
    //     emit(robot_position);

    //     // Create and emit the OptimisationResetDone message used by the walk optimiser
    //     if (sensor_measurements.reset_done) {
    //         emit(std::make_unique<OptimisationResetDone>());
    //     }
    // }
}  // namespace module::support::optimisation
