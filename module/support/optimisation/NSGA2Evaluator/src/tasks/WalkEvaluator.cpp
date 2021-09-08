#include <fstream>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "WalkEvaluator.hpp"

#include "utility/support/yaml_expression.hpp"
#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

#include "message/motion/WalkCommand.hpp"
#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::platform::RawSensors;
            using message::motion::WalkCommand;
            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::support::optimisation::NSGA2TrialExpired;
            using message::support::optimisation::NSGA2FitnessScores;

            using utility::behaviour::RegisterAction;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::support::Expression;

            WalkEvaluator::WalkEvaluator(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

                emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                    subsumptionId,
                    "Walk Evaluator",
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
            }

            bool WalkEvaluator::processRawSensorMsg(const RawSensors& sensors) {
                bool shouldTerminateEarly = false;
                shouldTerminateEarly = checkForFall(sensors);
                updateMaxFieldPlaneSway(sensors);
                return shouldTerminateEarly;
            }

            void WalkEvaluator::processOptimisationRobotPosition(const OptimisationRobotPosition& position) {
                if(!initialPositionSet) {
                    initialPositionSet = true;
                    initialRobotPosition.x() = position.value.X;
                    initialRobotPosition.y() = position.value.Y;
                    initialRobotPosition.z() = position.value.Z;
                }
                robotPosition.x() = position.value.X;
                robotPosition.y() = position.value.Y;
                robotPosition.z() = position.value.Z;
            }

            void WalkEvaluator::setUpTrial(const NSGA2EvaluationRequest& currentRequest) {
                // Set our generation and individual identifiers from the request

                //TODO: 2021-09-08 Joel Wong add this in
                //trial_duration_limit  = request.trial_duration_limit;

                // Set our walk command
                walk_command_velocity.x() = currentRequest.parameters.velocity;
                walk_command_velocity.y() = 0.0;
                walk_command_rotation = 0.0;

                // Read the QuinticWalk config and overwrite the config parameters with the current individual's
                // parameters
                YAML::Node walk_config = YAML::LoadFile("config/webots/QuinticWalk.yaml");

                auto walk                    = walk_config["walk"];
                walk["freq"]                 = currentRequest.parameters.freq;
                walk["double_support_ratio"] = currentRequest.parameters.double_support_ratio;

                auto foot        = walk["foot"];
                foot["distance"] = currentRequest.parameters.foot.distance;
                foot["rise"]     = currentRequest.parameters.foot.rise;

                auto trunk        = walk["trunk"];
                trunk["height"]   = currentRequest.parameters.trunk.height;
                trunk["pitch"]    = currentRequest.parameters.trunk.pitch;
                trunk["x_offset"] = currentRequest.parameters.trunk.x_offset;
                trunk["y_offset"] = currentRequest.parameters.trunk.y_offset;
                trunk["swing"]    = currentRequest.parameters.trunk.swing;
                trunk["pause"]    = currentRequest.parameters.trunk.pause;

                auto pause        = walk["pause"];
                pause["duration"] = currentRequest.parameters.pause.duration;

                auto gains    = walk["gains"];
                gains["legs"] = currentRequest.parameters.gains.legs;

                // Write the updated config to disk
                std::ofstream output_file_stream("config/webots/QuinticWalk.yaml");
                output_file_stream << YAML::Dump(walk_config);
                output_file_stream.close();
            }

            void WalkEvaluator::resetSimulation() {
                // Reset our local state
                trialStartTime         = 0.0;
                robotPosition = Eigen::Vector3d::Zero();
                initialRobotPosition = Eigen::Vector3d::Zero();
                maxFieldPlaneSway      = 0.0;
            }

            void WalkEvaluator::evaluatingState(bool finishedReset, double simTime, int generation, int individual) {
                if (finishedReset) {
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

                    // Schedule the end of the walk trial after the duration limit
                    emit<Scope::DELAY>(message, std::chrono::seconds(trial_duration_limit));
                }
            }

            void WalkEvaluator::stopCurrentTask() {
                // Send a zero walk command to stop walking
                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(0.0, 0.0, 0.0)));
            }

            void WalkEvaluator::sendFitnessScores(bool constraintsViolated, double simTime, int generation, int individual) {
                auto scores = calculateScores();
                auto constraints = constraintsViolated ? calculateConstraints(simTime) : constraintsNotViolated();

                double trialDuration = simTime - trialStartTime;
                log<NUClear::INFO>("Trial ran for", trialDuration);
                log<NUClear::INFO>("SendFitnessScores for generation", generation, "individual", individual);
                log<NUClear::INFO>("    scores:", scores[0], scores[1]);
                log<NUClear::INFO>("    constraints:", constraints[0], constraints[1]);

                // Create the fitness scores message based on the given results and emit it back to the Optimiser
                std::unique_ptr<NSGA2FitnessScores> fitnessScores = std::make_unique<NSGA2FitnessScores>();
                fitnessScores->id                                 = individual;
                fitnessScores->generation                         = generation;
                fitnessScores->objScore                           = scores;
                fitnessScores->constraints                        = constraints;
                emit(fitnessScores);
            }

            std::vector<double> WalkEvaluator::calculateScores() {
                auto robotDistanceTravelled = std::fabs(initialRobotPosition.x() - robotPosition.x());
                return {
                    maxFieldPlaneSway,  // For now, we want to reduce this
                    1.0 / robotDistanceTravelled  // 1/x since the NSGA2 optimiser is a minimiser
                };
            }

            std::vector<double> WalkEvaluator::calculateConstraints(double simTime) {
                // Convert trial duration limit to ms, add 1 for overhead
                const double overhead = 1;
                double max_trial_duration = (trial_duration_limit + overhead) * 1000;
                double trialDuration = simTime - trialStartTime;
                return {
                    trialDuration - max_trial_duration,  // Punish for falling over, based on how long the trial took
                                                         // (more negative is worse)
                    0.0                                  // Second constraint unused, fixed to 0
                };
            }

            std::vector<double> WalkEvaluator::constraintsNotViolated() {
                return {
                    0,  // Robot didn't fall
                    0   // Second constraint unused, fixed to 0
                };
            }


            bool WalkEvaluator::checkForFall(const RawSensors& sensors) {
                bool shouldTerminateEarly = false;
                auto accelerometer = sensors.accelerometer;

                if ((std::fabs(accelerometer.x) > 9.2 || std::fabs(accelerometer.y) > 9.2)
                    && std::fabs(accelerometer.z) < 0.5) {
                    log<NUClear::INFO>("Fallen!");
                    log<NUClear::DEBUG>("acc at fall (x y z):",
                                        std::fabs(accelerometer.x),
                                        std::fabs(accelerometer.y),
                                        std::fabs(accelerometer.z));
                    shouldTerminateEarly = true;
                }
                return shouldTerminateEarly;
            }

            void WalkEvaluator::updateMaxFieldPlaneSway(const RawSensors& sensors) {
                auto accelerometer = sensors.accelerometer;

                // Calculate the robot sway along the field plane (left/right, forward/backward)
                double fieldPlaneSway = std::pow(std::pow(accelerometer.x, 2) + std::pow(accelerometer.y, 2), 0.5);
                if(fieldPlaneSway > maxFieldPlaneSway) {
                    maxFieldPlaneSway = fieldPlaneSway;
                }
            }

        }  // namespace optimisation
    }      // namespace support
}  // namespace module
