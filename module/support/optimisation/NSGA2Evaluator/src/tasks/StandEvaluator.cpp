#include "StandEvaluator.hpp"

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

            void StandEvaluator::processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
                double sim_time = evaluator->sim_time;
                sim_time++;

                updateMaxFieldPlaneSway(sensors);
                current_sensors = sensors;
            }

            void StandEvaluator::processOptimisationRobotPosition(const OptimisationRobotPosition& position) {
                robot_position.x() = position.value.X;
                robot_position.y() = position.value.Y;
                robot_position.z() = position.value.Z;
            }

            void StandEvaluator::setUpTrial(const NSGA2EvaluationRequest& currentRequest) {
                loadScript(currentRequest.task_config_path);
                std::chrono::milliseconds limit_ms = std::chrono::milliseconds(0);
                for (size_t i = 0; i < currentRequest.parameters.real_params.size(); i++) {
                    int frame_time            = currentRequest.parameters.real_params[i];
                    limit_ms                  = limit_ms + std::chrono::milliseconds(frame_time);
                    script.frames[i].duration = std::chrono::milliseconds(frame_time);
                }
                saveScript(fmt::format("gen{:03d}_ind{:03d}_task-{}.yaml",
                                       currentRequest.generation,
                                       currentRequest.id,
                                       currentRequest.task));

                auto overhead =
                    std::chrono::seconds(2);  // Overhead tacked on to give the robot time to fall over if unstable
                trial_duration_limit = std::chrono::duration_cast<std::chrono::seconds>(limit_ms) + overhead;
            }

            void StandEvaluator::resetSimulation() {
                // Reset our local state
                trial_start_time     = 0.0;
                robot_position       = Eigen::Vector3d::Zero();
                max_field_plane_sway = 0.0;
            }

            void StandEvaluator::evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator) {
                NUClear::log<NUClear::DEBUG>("Running Script");
                runScript(subsumptionId, evaluator);
                NUClear::log<NUClear::DEBUG>("schedule expire");
                evaluator->ScheduleTrialExpiredMessage(0, trial_duration_limit);
            }

            std::unique_ptr<NSGA2FitnessScores> StandEvaluator::calculateFitnessScores(bool earlyTermination,
                                                                                       double sim_time,
                                                                                       int generation,
                                                                                       int individual) {
                double trial_duration = sim_time - trial_start_time;
                auto scores          = calculateScores(trial_duration);
                auto constraints = earlyTermination ? calculateConstraints() : calculateConstraints();

                NUClear::log<NUClear::DEBUG>("Trial ran for", trial_duration);
                NUClear::log<NUClear::DEBUG>("SendFitnessScores for generation", generation, "individual", individual);
                NUClear::log<NUClear::DEBUG>("    scores:", scores[0], scores[1]);
                NUClear::log<NUClear::DEBUG>("    constraints:", constraints[0], constraints[1]);

                // Create the fitness scores message based on the given results and emit it back to the Optimiser
                std::unique_ptr<NSGA2FitnessScores> fitness_scores = std::make_unique<NSGA2FitnessScores>();
                fitness_scores->id                                 = individual;
                fitness_scores->generation                         = generation;
                fitness_scores->obj_score                          = scores;
                fitness_scores->constraints                        = constraints;
                return fitness_scores;
            }

            std::vector<double> StandEvaluator::calculateScores(double trial_duration) {
                return {max_field_plane_sway,  // For now, we want to reduce this
                        trial_duration};
            }

            std::vector<double> StandEvaluator::calculateConstraints() {
                bool fallen             = checkForFall(current_sensors);
                double fallen_contraint = fallen ? -1.0 : 0;
                return {
                    fallen_contraint,
                    0  // Second constraint unused, fixed to 0
                };
            }

            bool StandEvaluator::checkForFall(const RawSensors& sensors) {
                bool fallen        = false;
                auto accelerometer = sensors.accelerometer;

                if ((std::fabs(accelerometer.x()) > 9.2 || std::fabs(accelerometer.y()) > 9.2)
                    && std::fabs(accelerometer.z()) < 0.5) {
                    fallen = true;
                }
                return fallen;
            }


            void StandEvaluator::loadScript(std::string script_path) {
                if (utility::file::exists(script_path)) {
                    NUClear::log<NUClear::DEBUG>("Loading script: ", script_path, '\n');
                    script = YAML::LoadFile(script_path).as<::extension::Script>();
                }
                else {
                    NUClear::log<NUClear::ERROR>("No script found at: ", script_path, '\n');
                }
            }

            void StandEvaluator::saveScript(std::string script_path) {
                NUClear::log<NUClear::DEBUG>("Saving as: ", script_path);
                YAML::Node n(script);
                utility::file::writeToFile(script_path, n);
            }

            void StandEvaluator::runScript(size_t subsumptionId, NSGA2Evaluator* evaluator) {
                evaluator->emit(
                    std::make_unique<extension::ExecuteScript>(subsumptionId, script, NUClear::clock::now()));
            }

            void StandEvaluator::updateMaxFieldPlaneSway(const RawSensors& sensors) {
                auto accelerometer = sensors.accelerometer;

                // Calculate the robot sway along the field plane (left/right, forward/backward)
                double field_plane_sway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
                if (field_plane_sway > max_field_plane_sway) {
                    max_field_plane_sway = field_plane_sway;
                }
            }

        }  // namespace optimisation
    }      // namespace support
}  // namespace module
