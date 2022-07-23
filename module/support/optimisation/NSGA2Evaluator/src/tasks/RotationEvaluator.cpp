#include "RotationEvaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "message/motion/WalkCommand.hpp"
#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::motion::WalkCommand;
            using message::platform::RawSensors;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2TrialExpired;

            using utility::behaviour::RegisterAction;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::support::Expression;

            void RotationEvaluator::processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {

                updateMaxFieldPlaneSway(sensors);
                if (checkForFall(sensors)) {
                    evaluator->emit(std::make_unique<NSGA2Evaluator::Event>(NSGA2Evaluator::Event::TerminateEarly));
                }
                double simTime = evaluator->simTime;
                omega          = sensors.gyroscope.z();
                deltaT         = simTime - oldTime;
                oldTime        = simTime;
                theta += omega * deltaT / 1000;
                // NUClear::log<NUClear::DEBUG>("Theta =", theta);
                // NUClear::log<NUClear::DEBUG>("Omega =", omega);
                // NUClear::log<NUClear::DEBUG>("DeltaT =", deltaT);
            }

            void RotationEvaluator::processOptimisationRobotPosition(const OptimisationRobotPosition& position,
                                                                     NSGA2Evaluator* evaluator) {
                if (!initialPositionSet) {
                    initialPositionSet       = true;
                    initialRobotPosition.x() = position.value.X;
                    initialRobotPosition.y() = position.value.Y;
                    initialRobotPosition.z() = position.value.Z;
                }

                robotPosition.x() = position.value.X;
                robotPosition.y() = position.value.Y;
                robotPosition.z() = position.value.Z;
            }

            void RotationEvaluator::setUpTrial(const NSGA2EvaluationRequest& currentRequest) {
                // Set our generation and individual identifiers from the request

                trial_duration_limit = std::chrono::seconds(currentRequest.trial_duration_limit);

                // Set our walk command
                walk_command_velocity.x() = -0.05;  // currentRequest.parameters.real_params[11];
                walk_command_velocity.y() = -0.05;  // currentRequest.parameters.real_params[11];  // 0.0;
                walk_command_rotation     = currentRequest.parameters.real_params[11];  // -0.785;  // 0.0;

                // Read the QuinticWalk config and overwrite the config parameters with the current individual's
                // parameters
                YAML::Node walk_config = YAML::LoadFile(currentRequest.task_config_path);

                // The mapping of parameters depends on how the config file was read by the optimiser
                auto walk                    = walk_config["walk"];
                walk["freq"]                 = currentRequest.parameters.real_params[0];
                walk["double_support_ratio"] = currentRequest.parameters.real_params[1];

                auto foot        = walk["foot"];
                foot["distance"] = currentRequest.parameters.real_params[2];
                foot["rise"]     = currentRequest.parameters.real_params[3];

                auto trunk        = walk["trunk"];
                trunk["height"]   = currentRequest.parameters.real_params[4];
                trunk["pitch"]    = currentRequest.parameters.real_params[5];
                trunk["x_offset"] = currentRequest.parameters.real_params[6];
                trunk["y_offset"] = currentRequest.parameters.real_params[7];

                trunk["swing"] = currentRequest.parameters.real_params[8];
                trunk["pause"] = currentRequest.parameters.real_params[9];

                auto pause        = walk["pause"];
                pause["duration"] = currentRequest.parameters.real_params[10];

                // Write the updated config to disk
                std::ofstream overwrite_file_stream(currentRequest.task_config_path);
                overwrite_file_stream << YAML::Dump(walk_config);
                overwrite_file_stream.close();

                // Write the config to keep for later
                NUClear::log<NUClear::DEBUG>(fmt::format("Saving as: gen{:03d}_ind{:03d}_task-{}.yaml",
                                                         currentRequest.generation,
                                                         currentRequest.id,
                                                         currentRequest.task));
                std::ofstream save_file_stream(fmt::format("gen{:03d}_ind{:03d}_task-{}.yaml",
                                                           currentRequest.generation,
                                                           currentRequest.id,
                                                           currentRequest.task));
                save_file_stream << YAML::Dump(walk_config);
                save_file_stream.close();
            }

            void RotationEvaluator::resetSimulation() {
                // Reset our local stateconst OptimisationRobotPosition& position
                trialStartTime       = 0.0;
                robotPosition        = Eigen::Vector3d::Zero();
                initialRobotPosition = Eigen::Vector3d::Zero();
                maxFieldPlaneSway    = 0.0;
            }

            void RotationEvaluator::evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator) {
                NUClear::log<NUClear::DEBUG>(fmt::format("Trialling with walk command: ({} {}) {}",
                                                         walk_command_velocity.x(),
                                                         walk_command_velocity.y(),
                                                         walk_command_rotation));

                evaluator->emit(std::make_unique<WalkCommand>(
                    subsumptionId,
                    Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation)));
                evaluator->ScheduleTrialExpiredMessage(0, trial_duration_limit);
            }

            std::unique_ptr<NSGA2FitnessScores> RotationEvaluator::calculateFitnessScores(bool earlyTermination,
                                                                                          double simTime,
                                                                                          int generation,
                                                                                          int individual) {
                auto scores      = calculateScores();
                auto constraints = earlyTermination ? calculateConstraints(simTime) : constraintsNotViolated();

                double trialDuration = simTime - trialStartTime;
                NUClear::log<NUClear::DEBUG>("Trial ran for", trialDuration);
                NUClear::log<NUClear::DEBUG>("SendFitnessScores for generation", generation, "individual", individual);
                NUClear::log<NUClear::DEBUG>("    scores:", scores[0], scores[1]);
                NUClear::log<NUClear::DEBUG>("    constraints:", constraints[0], constraints[1]);

                // Create the fitness scores message based on the given results and emit it back to the Optimiser
                std::unique_ptr<NSGA2FitnessScores> fitnessScores = std::make_unique<NSGA2FitnessScores>();
                fitnessScores->id                                 = individual;
                fitnessScores->generation                         = generation;
                fitnessScores->objScore                           = scores;
                fitnessScores->constraints                        = constraints;
                return fitnessScores;
            }

            std::vector<double> RotationEvaluator::calculateScores() {
                // make this pythagorean
                auto robotDistanceTravelled_X = std::fabs(initialRobotPosition.x() - robotPosition.x());
                auto robotDistanceTravelled_Y = std::fabs(initialRobotPosition.y() - robotPosition.y());
                auto robotDistanceTravelled =
                    std::pow(std::pow(robotDistanceTravelled_X, 2) + std::pow(robotDistanceTravelled_Y, 2), 0.5);

                return {
                    maxFieldPlaneSway,                    // For now, we want to reduce this
                    1.0 / theta + robotDistanceTravelled  // finish closer to where we start the better. Minumiser!!!
                };
            }

            std::vector<double> RotationEvaluator::calculateConstraints(double simTime) {
                // Convert trial duration limit to ms, add 1 for overhead
                const auto overhead = std::chrono::seconds(1);
                double max_trial_duration =
                    (std::chrono::duration_cast<std::chrono::milliseconds>(trial_duration_limit + overhead)).count();
                double trialDuration = simTime - trialStartTime;
                return {
                    trialDuration - max_trial_duration,  // Punish for falling over, based on how long the trial took
                                                         // (more negative is worse)
                    0.0                                  // Second constraint unused, fixed to 0
                };
            }

            std::vector<double> RotationEvaluator::constraintsNotViolated() {
                return {
                    0,  // Robot didn't fall
                    0   // Second constraint unused, fixed to 0
                };
            }


            bool RotationEvaluator::checkForFall(const RawSensors& sensors) {
                bool fallen        = false;
                auto accelerometer = sensors.accelerometer;

                if ((std::fabs(accelerometer.x()) > 9.2 || std::fabs(accelerometer.y()) > 9.2)
                    && std::fabs(accelerometer.z()) < 0.5) {
                    NUClear::log<NUClear::DEBUG>("Fallen!");
                    NUClear::log<NUClear::DEBUG>("acc at fall (x y z):",
                                                 std::fabs(accelerometer.x()),
                                                 std::fabs(accelerometer.y()),
                                                 std::fabs(accelerometer.z()));
                    fallen = true;
                }
                return fallen;
            }

            void RotationEvaluator::updateMaxFieldPlaneSway(const RawSensors& sensors) {
                auto accelerometer = sensors.accelerometer;

                // Calculate the robot sway along the field plane (left/right, forward/backward)
                double fieldPlaneSway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
                if (fieldPlaneSway > maxFieldPlaneSway) {
                    maxFieldPlaneSway = fieldPlaneSway;
                }
            }

        }  // namespace optimisation
    }      // namespace support
}  // namespace module
