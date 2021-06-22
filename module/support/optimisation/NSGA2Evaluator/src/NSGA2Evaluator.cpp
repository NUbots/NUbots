#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/WebotsResetDone.hpp"
#include "message/platform/webots/WebotsTimeUpdate.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2EvaluationRequest.hpp"
#include "message/support/optimisation/NSGA2FitnessScores.hpp"
#include "message/support/optimisation/NSGA2Terminate.hpp"
#include "message/support/optimisation/NSGA2TrialExpired.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;
            using extension::ExecuteScriptByName;

            using message::behaviour::MotionCommand;
            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::motion::StopCommand;
            using message::motion::WalkCommand;
            using RawSensorsMsg = message::platform::RawSensors;
            using message::platform::webots::OptimisationCommand;
            using message::platform::webots::OptimisationRobotPosition;
            using message::platform::webots::WebotsResetDone;
            using message::platform::webots::WebotsTimeUpdate;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;
            using message::support::optimisation::NSGA2TrialExpired;

            using utility::behaviour::RegisterAction;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::support::Expression;

            NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

                emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                    subsumptionId,
                    "NSGA2 Evaluator",
                    {std::pair<float, std::set<LimbID>>(
                        1,
                        {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                    [this](const std::set<LimbID>& given_limbs) {
                        if (given_limbs.find(LimbID::LEFT_LEG) != given_limbs.end()) {
                            // Enable the walk engine.},
                            emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
                        }
                    },
                    [this](const std::set<LimbID>& taken_limbs) {
                        if (taken_limbs.find(LimbID::LEFT_LEG) != taken_limbs.end()) {
                            // Shut down the walk engine, since we don't need it right now.
                            emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumptionId));
                        }
                    },
                    [this](const std::set<ServoID>&) {}}));

                // Read the NSGA2Evaluator.yaml config file and set the walk command parameters
                on<Configuration>("NSGA2Evaluator.yaml").then([this](const Configuration& config) {
                    walk_command_velocity = config["walk_command"]["velocity"].as<Expression>();
                    walk_command_rotation = config["walk_command"]["rotation"].as<Expression>();
                });

                // Handle a state transition event
                on<Trigger<Event>, Sync<NSGA2Evaluator>>().then([this](const Event& event) {
                    // log("event", event);

                    State oldState = currentState;
                    State newState = HandleTransition(currentState, event);

                    // Some transitions are ignored
                    if (newState == State::IGNORE) {
                        return;
                    }

                    log<NUClear::INFO>("transitioning on", event, ", from state", oldState, "to state", newState);

                    switch (newState) {
                        case State::WAITING_FOR_REQUEST:
                            currentState = newState;
                            WaitingForRequest(oldState, event);
                            break;
                        case State::SETTING_UP_TRIAL:
                            currentState = newState;
                            SettingUpTrial(oldState, event);
                            break;
                        case State::RESETTING_SIMULATION:
                            currentState = newState;
                            ResettingSimulation(oldState, event);
                            break;
                        case State::STANDING:
                            currentState = newState;
                            Standing(oldState, event);
                            break;
                        case State::WALKING:
                            currentState = newState;
                            Walking(oldState, event);
                            break;
                        case State::TERMINATING_EARLY:
                            currentState = newState;
                            TerminatingEarly(oldState, event);
                            break;
                        case State::TERMINATING_GRACEFULLY:
                            currentState = newState;
                            TerminatingGracefully(oldState, event);
                            break;
                        default:
                            log<NUClear::WARN>("unable to transition to unknown state from", currentState, "on", event);
                    }
                });

                on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
                    lastEvalRequestMsg = request;
                    emit(std::make_unique<Event>(Event::EvaluateRequest));
                });

                on<Trigger<RawSensorsMsg>, Single>().then([this](const RawSensorsMsg& sensors) {
                    CheckForFall(sensors);

                    if (currentState == State::STANDING) {
                        CheckForStandDone(sensors);
                    }
                });

                on<Trigger<WebotsResetDone>, Single>().then(
                    [this](const WebotsResetDone&) { emit(std::make_unique<Event>(Event::ResetDone)); });

                on<Trigger<WebotsTimeUpdate>, Single>().then([this](const WebotsTimeUpdate& update) {
                    simTimeDelta = update.sim_time - simTime;
                    simTime      = update.sim_time;
                });

                on<Trigger<NSGA2Terminate>, Single>().then([this]() {
                    log("Got NSGA2Terminate, figure how to use this event");
                    emit(std::make_unique<Event>(Event::TerminateEvaluation));  // TODO: handle this in WAITING, other
                });

                on<Trigger<OptimisationRobotPosition>, Single>().then(
                    [this](const OptimisationRobotPosition& position) {
                        robotDistanceTravelled += std::pow(std::pow(position.value.X - robotPosition[0], 2)
                                                               + std::pow(position.value.Y - robotPosition[1], 2),
                                                           0.5);

                        robotPosition[0] = position.value.X;
                        robotPosition[1] = position.value.Y;
                        robotPosition[2] = position.value.Z;
                    });
            }

            void NSGA2Evaluator::CheckForFall(const RawSensorsMsg& sensors) {
                if (currentState == State::STANDING || currentState == State::WALKING) {
                    auto accelerometer = sensors.accelerometer;

                    if ((std::abs(accelerometer.x) > 9.2 || std::abs(accelerometer.y) > 9.2)
                        && std::abs(accelerometer.z) < 0.5) {
                        log("fallen, sending StandStill, then Fallen!");
                        // emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));

                        // Send a zero walk command to stop walking
                        emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(0.0, 0.0, 0.0)));

                        emit(std::make_unique<Event>(Event::Fallen));
                    }
                }
            }

            void NSGA2Evaluator::CheckForStandDone(const RawSensorsMsg& sensors) {
                // Where we expect these servos to be when the stand is complete
                float l_elbow_target          = -1.569;
                float r_elbow_target          = -1.569;
                float l_shoulder_pitch_target = 2.0505;
                float r_shoulder_pitch_target = 1.98;

                // The acceptable error margin for the target positions
                float epsilon = 0.015;

                bool l_elbow_ok = std::fabs(sensors.servo.l_elbow.present_position - l_elbow_target) < epsilon;
                bool r_elbow_ok = std::fabs(sensors.servo.r_elbow.present_position - r_elbow_target) < epsilon;
                bool l_shoulder_pitch_ok =
                    std::fabs(sensors.servo.l_shoulder_pitch.present_position - l_shoulder_pitch_target) < epsilon;
                bool r_shoulder_pitch_ok =
                    std::fabs(sensors.servo.r_shoulder_pitch.present_position - r_shoulder_pitch_target) < epsilon;

                if (l_elbow_ok && r_elbow_ok && l_shoulder_pitch_ok && r_shoulder_pitch_ok) {
                    log("stand done!");
                    emit(std::make_unique<Event>(Event::StandDone));
                }

                // log("checking for stand");
                // log("  l_elbow: ", sensors.servo.l_elbow.present_position);
                // log("  r_elbow: ", sensors.servo.r_elbow.present_position);
                // log("  l_shoulder_pitch: ", sensors.servo.l_shoulder_pitch.present_position);
                // log("  r_shoulder_pitch: ", sensors.servo.r_shoulder_pitch.present_position);
            }

            NSGA2Evaluator::State NSGA2Evaluator::HandleTransition(NSGA2Evaluator::State currentState,
                                                                   NSGA2Evaluator::Event event) {
                switch (currentState) {
                    case State::WAITING_FOR_REQUEST:
                        switch (event) {
                            case Event::EvaluateRequest: return State::SETTING_UP_TRIAL;
                            // case Event::TerminateEvaluation: return State::WAITING_FOR_REQUEST;
                            default: return State::UNKNOWN;
                        }
                    case State::SETTING_UP_TRIAL:
                        switch (event) {
                            case Event::TrialSetupDone: return State::RESETTING_SIMULATION;
                            default: return State::UNKNOWN;
                        }
                    case State::RESETTING_SIMULATION:
                        switch (event) {
                            case Event::ResetDone: return State::STANDING;
                            default: return State::UNKNOWN;
                        }
                    case State::STANDING:
                        switch (event) {
                            case Event::Fallen: return State::TERMINATING_EARLY;
                            case Event::StandDone: return State::WALKING;
                            default: return State::UNKNOWN;
                        }
                    case State::WALKING:
                        switch (event) {
                            case Event::Fallen: return State::TERMINATING_EARLY;
                            case Event::TrialTimeExpired: return State::TERMINATING_GRACEFULLY;
                            default: return State::UNKNOWN;
                        }
                    case State::TERMINATING_EARLY:
                        switch (event) {
                            case Event::FitnessScoresSent: return State::WAITING_FOR_REQUEST;
                            default: return State::UNKNOWN;
                        }
                    case State::TERMINATING_GRACEFULLY:
                        switch (event) {
                            case Event::ExpiredTrialInvalid: return State::WALKING;
                            case Event::FitnessScoresSent: return State::WAITING_FOR_REQUEST;
                            default: return State::UNKNOWN;
                        }
                    default: return State::UNKNOWN;
                }
            }

            /// @brief Handle the WAITING_FOR_REQUEST state
            void NSGA2Evaluator::WaitingForRequest(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("WaitingForRequest");
            }

            /// @brief Handle the SETTING_UP_TRIAL state
            void NSGA2Evaluator::SettingUpTrial(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("SettingUpTrial");

                // Set our genration and individual identifiers from the request
                generation = lastEvalRequestMsg.generation;
                individual = lastEvalRequestMsg.id;

                // Read the QuinticWalk config and overwrite the config parameters with the current individual's
                // parameters
                YAML::Node walk_config = YAML::LoadFile("config/webots/QuinticWalk.yaml");

                auto walk                    = walk_config["walk"];
                walk["freq"]                 = lastEvalRequestMsg.parameters.freq;
                walk["double_support_ratio"] = lastEvalRequestMsg.parameters.double_support_ratio;

                auto foot        = walk["foot"];
                foot["distance"] = lastEvalRequestMsg.parameters.foot.distance;
                foot["rise"]     = lastEvalRequestMsg.parameters.foot.rise;

                auto trunk        = walk["trunk"];
                trunk["height"]   = lastEvalRequestMsg.parameters.trunk.height;
                trunk["pitch"]    = lastEvalRequestMsg.parameters.trunk.pitch;
                trunk["x_offset"] = lastEvalRequestMsg.parameters.trunk.x_offset;
                trunk["y_offset"] = lastEvalRequestMsg.parameters.trunk.y_offset;
                trunk["swing"]    = lastEvalRequestMsg.parameters.trunk.swing;
                trunk["pause"]    = lastEvalRequestMsg.parameters.trunk.pause;

                auto pause        = walk["pause"];
                pause["duration"] = lastEvalRequestMsg.parameters.pause.duration;

                auto gains    = walk["gains"];
                gains["legs"] = lastEvalRequestMsg.parameters.gains.legs;

                // Write the updated config to disk
                std::ofstream output_file_stream("config/webots/QuinticWalk.yaml");
                output_file_stream << YAML::Dump(walk_config);
                output_file_stream.close();

                emit(std::make_unique<Event>(Event::TrialSetupDone));
            }

            /// @brief Handle the RESETTING_SIMULATION state
            void NSGA2Evaluator::ResettingSimulation(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("ResettingSimulation: sending message to reset world, resetting local");

                // Reset our local state
                simTimeDelta           = 0.0;
                robotDistanceTravelled = 0.0;

                // Tell Webots to reset the world
                std::unique_ptr<OptimisationCommand> reset = std::make_unique<OptimisationCommand>();
                reset->command                             = OptimisationCommand::CommandType::RESET_WORLD;
                emit(reset);
            }

            /// @brief Handle the STANDING state
            void NSGA2Evaluator::Standing(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("Standing: emitting Stand script");

                emit(std::make_unique<Event>(Event::StandDone));

                // // Start the stand script if we just finished resetting the world
                // if (event == Event::ResetDone) {
                //     emit(std::make_unique<ExecuteScriptByName>(subsumptionId, "Stand.yaml"));
                // }

                // // Check that we haven't fallen if we just got updated RawSensors
                // if (event == Event::RawSensors) {

                // }
            }

            /// @brief Handle the WALKING state
            void NSGA2Evaluator::Walking(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("Walking");

                if (event == Event::StandDone) {
                    // Create and send the walk command, which will be evaluated when we get back sensors and time
                    // updates
                    log<NUClear::INFO>(fmt::format("Trialling with walk command: ({}) {}",
                                                   walk_command_velocity.transpose(),
                                                   walk_command_rotation));

                    // Send the command to start walking
                    emit(std::make_unique<WalkCommand>(
                        subsumptionId,
                        Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation)));

                    // Prepare the trial expired message
                    std::unique_ptr<NSGA2TrialExpired> message = std::make_unique<NSGA2TrialExpired>();
                    message->time_started                      = simTime;
                    message->generation                        = generation;
                    message->individual                        = individual;

                    // Schedule the end of the walk trial after 30 seconds
                    emit<Scope::DELAY>(message, std::chrono::seconds(30));
                }

                // // Check that we haven't fallen if we just got updated RawSensors
                // if (event == Event::RawSensors) {
                //     CheckForFall();
                // }
            }

            /// @brief Handle the TERMINATING_EARLY state
            void NSGA2Evaluator::TerminatingEarly(NSGA2Evaluator::State previousState, NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("TerminatingEarly");

                // Calculate and send the fitness scores for a fall if just fell over
                if (event == Event::Fallen) {
                    std::vector<double> scores = {
                        1.0,  // Fix the first score as we're trying to optimise only the distance travelled
                        1.0 / robotDistanceTravelled  // 1/x since the NSGA2 optimiser is a minimiser
                    };

                    std::vector<double> constraints = {
                        -10.0,  // Punish for falling over: TODO: set based on total time walking
                        0.0,    // Second constraint unused, fixed to 0
                    };

                    // Send the fitness scores back to the optimiser
                    SendFitnessScores(scores, constraints);

                    // Go back to waiting for the next request
                    emit(std::make_unique<Event>(Event::FitnessScoresSent));
                }
                else {
                    log("Transitioned to TerminateEarly from an event that wasn't fallen: this shouldn't happen!!");
                }
            }

            /// @brief Handle the TERMINATING_GRACEFULLY state
            void NSGA2Evaluator::TerminatingGracefully(NSGA2Evaluator::State previousState,
                                                       NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("TerminatingGracefully");

                // Calculate and emit the fitness scores if the TrialExpired message applies to the current evaluation
                if (lastTrialExpiredMsg.generation == generation && lastTrialExpiredMsg.individual == individual) {
                    std::vector<double> scores = {
                        1,  // Fix the first score as we're trying to optimise only the distance travelled
                        1.0 / robotDistanceTravelled  // 1/x since the NSGA2 optimiser is a minimiser
                    };

                    std::vector<double> constraints = {
                        0,  // Robot didn't fall
                        0,  // Second constraint unused, fixed to 0
                    };

                    // Send the fitness scores back to the optimiser
                    SendFitnessScores(scores, constraints);

                    // Go back to waiting for the next request
                    emit(std::make_unique<Event>(Event::FitnessScoresSent));
                }
                // Otherwise we got a trial expired for a previous evaluation that's no longer relevant. This happens
                // if a previous trial ended early without the trial timeout completing.
                else {
                    emit(std::make_unique<Event>(Event::ExpiredTrialInvalid));
                }
            }

            void NSGA2Evaluator::SendFitnessScores(std::vector<double> scores, std::vector<double> constraints) {
                log<NUClear::DEBUG>("SendFitnessScores");

                // Create the fitness scores message based on the given results and emit it back to the Optimiser
                std::unique_ptr<NSGA2FitnessScores> fitnessScores = std::make_unique<NSGA2FitnessScores>();
                fitnessScores->id                                 = individual;
                fitnessScores->generation                         = generation;
                fitnessScores->objScore                           = scores;
                fitnessScores->constraints                        = constraints;
                emit(fitnessScores);
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
