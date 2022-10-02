#include "MultiPathEvaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "message/motion/WalkCommand.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

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

            void MultiPathEvaluator::processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
                updateMaxFieldPlaneSway(sensors);

                if (pathNo == ROTCCW || pathNo == ROTCW) {
                    processRotation(sensors, evaluator);
                }

                if (checkForFall(sensors)) {
                    processRoundEnd();
                    evaluator->emit(std::make_unique<NSGA2Evaluator::Event>(NSGA2Evaluator::Event::TerminateEarly));
                }

                if ((int) evaluator->simTime % 10000 == 0) {
                    processRoundEnd();
                    pathNo++;
                    processNextPath();

                    NUClear::log<NUClear::DEBUG>(fmt::format("Trialling with walk command: ({} {}) {}",
                                                             walk_command_velocity.x(),
                                                             walk_command_velocity.y(),
                                                             walk_command_rotation));

                    evaluator->emit(std::make_unique<WalkCommand>(
                        subsumptionID,
                        Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation)));
                }
            }

            void MultiPathEvaluator::processOptimisationRobotPosition(const OptimisationRobotPosition& position) {
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

            void MultiPathEvaluator::processRotation(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
                double rotTime = evaluator->simTime;
                omega          = std::fabs(sensors.gyroscope.z());
                deltaT         = rotTime - oldTime;
                oldTime        = rotTime;
                theta += omega * deltaT / 1000;
            }

            double MultiPathEvaluator::processDistanceTravelled() {
                if (pathNo == ROTCCW || pathNo == ROTCW) {
                    auto t = theta;
                    theta  = 0.0;
                    return t;
                }
                auto dta = std::fabs(initialRobotPosition.x() - robotPosition.x());
                auto dtb = std::fabs(initialRobotPosition.y() - robotPosition.y());

                initialRobotPosition.x() = robotPosition.x();
                initialRobotPosition.y() = robotPosition.y();
                initialRobotPosition.z() = robotPosition.z();

                return std::pow(std::pow(dta, 2) + std::pow(dtb, 2), 0.5);
            }

            void MultiPathEvaluator::processRoundEnd() {
                auto travelScore = processDistanceTravelled();

                if (pathNo == FWD || pathNo == BKWD || pathNo == STRFL || pathNo == STRFR) {
                    travelScore = 1.0 / travelScore * 20;
                }
                else if (pathNo == ROTCCW || pathNo == ROTCW) {
                    travelScore = 1.0 / travelScore * 5;
                }

                std::vector<double> scores = {travelScore, maxFieldPlaneSway};

                maxFieldPlaneSway = 0.0;
                for (double i : scores) {
                    NUClear::log<NUClear::DEBUG>("Scores:", i);
                }

                pathScores.push_back(scores);
                NUClear::log<NUClear::DEBUG>("Path Scores Size:", pathScores.size());
            }

            void MultiPathEvaluator::processNextPath() {
                switch (pathNo) {
                    case 0:
                        walk_command_velocity.x() = walk_command_velocity_X;
                        walk_command_velocity.y() = 0.0;
                        walk_command_rotation     = 0.0;
                        break;

                    case 1:
                        walk_command_velocity.x() = -walk_command_velocity_X;
                        walk_command_velocity.y() = 0.0;
                        walk_command_rotation     = 0.0;
                        break;

                    case 2:
                        walk_command_velocity.x() = 0.0;
                        walk_command_velocity.y() = walk_command_velocity_Y;
                        walk_command_rotation     = 0.0;
                        break;

                    case 3:
                        walk_command_velocity.x() = 0.0;
                        walk_command_velocity.y() = -walk_command_velocity_Y;
                        walk_command_rotation     = 0.0;
                        break;

                    case 4:
                        walk_command_velocity.x() = -0.05;
                        walk_command_velocity.y() = 0.05;
                        walk_command_rotation     = walk_command_Rotation;
                        break;

                    case 5:
                        walk_command_velocity.x() = -0.05;
                        walk_command_velocity.y() = -0.05;
                        walk_command_rotation     = -walk_command_Rotation;
                        break;

                    default:
                        walk_command_velocity.x() = 0.0;
                        walk_command_velocity.y() = 0.0;
                        walk_command_rotation     = 0.0;
                        break;
                }
            }

            void MultiPathEvaluator::setUpTrial(const NSGA2EvaluationRequest& currentRequest) {
                // Set our generation and individual identifiers from the request

                trial_duration_limit = std::chrono::seconds(currentRequest.trial_duration_limit);

                // Set our walk command
                walk_command_velocity_X = currentRequest.parameters.real_params[11];
                walk_command_velocity_Y = currentRequest.parameters.real_params[11];
                walk_command_Rotation   = currentRequest.parameters.real_params[12];

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

                // Set up param array for normalisiation
                for (unsigned int i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
                    params[i] = currentRequest.parameters.real_params[i];
                }
                for (const double& num : params) {
                    NUClear::log<NUClear::DEBUG>("value of param: ", num);
                }
            }

            void MultiPathEvaluator::resetSimulation() {
                // Reset our local stateconst OptimisationRobotPosition& position
                trialStartTime       = 0.0;
                robotPosition        = Eigen::Vector3d::Zero();
                initialRobotPosition = Eigen::Vector3d::Zero();
                maxFieldPlaneSway    = 0.0;
            }

            void MultiPathEvaluator::evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator) {

                subsumptionID = subsumptionId;
                NUClear::log<NUClear::DEBUG>("SubsumptionID =", subsumptionID);
                processNextPath();

                NUClear::log<NUClear::DEBUG>(fmt::format("Trialling with walk command: ({} {}) {}",
                                                         walk_command_velocity.x(),
                                                         walk_command_velocity.y(),
                                                         walk_command_rotation));

                evaluator->emit(std::make_unique<WalkCommand>(
                    subsumptionId,
                    Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation)));
                evaluator->ScheduleTrialExpiredMessage(0, trial_duration_limit);
            }

            std::unique_ptr<NSGA2FitnessScores> MultiPathEvaluator::calculateFitnessScores(bool earlyTermination,
                                                                                           double simTime,
                                                                                           int generation,
                                                                                           int individual) {
                auto scores      = calculateScores();
                auto constraints = earlyTermination ? calculateConstraints(simTime) : constraintsNotViolated();

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
            // I want a pattern here passed as an arg
            std::vector<double> MultiPathEvaluator::calculateScores() {
                // auto robotDistanceTravelled = 0.0;
                auto maxSway    = 0.0;
                auto finalScore = 0.0;

                for (std::vector<double> score : pathScores) {
                    for (auto param : params) {
                        finalScore += (score.at(0) * param) + (score.at(1) * param);
                    }
                }

                NUClear::log<NUClear::DEBUG>("Final Score:", finalScore);
                return {
                    maxSway,          // For now, we want to reduce this
                    1.0 / finalScore  // 1/x since the NSGA2 optimiser is a minimiser
                };
            }

            std::vector<double> MultiPathEvaluator::calculateConstraints(double simTime) {
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

            std::vector<double> MultiPathEvaluator::constraintsNotViolated() {
                return {
                    0,  // Robot didn't fall
                    0   // Second constraint unused, fixed to 0
                };
            }


            bool MultiPathEvaluator::checkForFall(const RawSensors& sensors) {
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

            void MultiPathEvaluator::updateMaxFieldPlaneSway(const RawSensors& sensors) {
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
