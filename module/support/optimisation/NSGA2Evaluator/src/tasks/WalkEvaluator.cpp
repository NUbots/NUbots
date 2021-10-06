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

                trial_duration_limit  = currentRequest.trial_duration_limit;

                // Set our walk command
                walk_command_velocity.x() = currentRequest.parameters.real_params[11];
                walk_command_velocity.y() = 0.0;
                walk_command_rotation = 0.0;

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
                trunk["swing"]    = currentRequest.parameters.real_params[8];
                trunk["pause"]    = currentRequest.parameters.real_params[9];

                auto pause        = walk["pause"];
                pause["duration"] = currentRequest.parameters.real_params[10];

                // Write the updated config to disk
                std::ofstream output_file_stream(currentRequest.task_config_path);
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

            std::map<std::string, float> WalkEvaluator::evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator) {
                evaluator->emit(std::make_unique<WalkCommand>(
                    subsumptionId,
                    Eigen::Vector3d(walk_command_velocity.x(),
                    walk_command_velocity.y(), walk_command_rotation)));

                std::map<std::string, float> map {
                    {"walk_x", walk_command_velocity.x()},
                    {"walk_y", walk_command_velocity.y()},
                    {"walk_rotation", walk_command_rotation},
                    {"trial_duration_limit", trial_duration_limit}};
                return map;
            }

            std::unique_ptr<NSGA2FitnessScores> WalkEvaluator::calculateFitnessScores(bool constraintsViolated, double simTime, int generation, int individual) {
                auto scores = calculateScores();
                auto constraints = constraintsViolated ? calculateConstraints(simTime) : constraintsNotViolated();

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
                    NUClear::log<NUClear::DEBUG>("Fallen!");
                    NUClear::log<NUClear::DEBUG>("acc at fall (x y z):",
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
