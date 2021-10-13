#include <fstream>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "StandEvaluator.hpp"

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

            void StandEvaluator::processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
                updateMaxFieldPlaneSway(sensors);
            }

            void StandEvaluator::processOptimisationRobotPosition(const OptimisationRobotPosition& position) {
                robotPosition.x() = position.value.X;
                robotPosition.y() = position.value.Y;
                robotPosition.z() = position.value.Z;
            }

            void StandEvaluator::setUpTrial(const NSGA2EvaluationRequest& currentRequest) {
                loadScript(currentRequest.task_config_path);
                trial_duration_limit = 0;
                for(size_t i = 0; i < currentRequest.parameters.real_params.size(); i++) {
                    int frame_time = currentRequest.parameters.real_params[i];
                    trial_duration_limit = trial_duration_limit + frame_time;
                    script.frames[i].duration = std::chrono::milliseconds(frame_time);
                }
            }

            void StandEvaluator::resetSimulation() {
                // Reset our local state
                trialStartTime         = 0.0;
                robotPosition = Eigen::Vector3d::Zero();
                maxFieldPlaneSway      = 0.0;
            }

            void StandEvaluator::evaluatingState(size_t subsumptionId, NSGA2Evaluator *evaluator) {
                runScript(subsumptionId, evaluator);
                evaluator->ScheduleTrialExpiredMessage(0, trial_duration_limit);
            }

            std::unique_ptr<NSGA2FitnessScores> StandEvaluator::calculateFitnessScores(bool earlyTermination, double simTime, int generation, int individual) {
                double trialDuration = simTime - trialStartTime;
                auto scores = calculateScores(trialDuration);
                auto constraints = earlyTermination ? calculateConstraints(simTime) : constraintsNotViolated();

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

            std::vector<double> StandEvaluator::calculateScores(double trialDuration) {
                return {
                    maxFieldPlaneSway,  // For now, we want to reduce this
                    trialDuration
                };
            }

            std::vector<double> StandEvaluator::calculateConstraints(double simTime) {

            }

            std::vector<double> StandEvaluator::constraintsNotViolated() {
                return {
                    0,  // Robot didn't fall
                    0   // Second constraint unused, fixed to 0
                };
            }

            bool StandEvaluator::checkForFall(const RawSensors& sensors) {
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


            void StandEvaluator::loadScript(std::string script_path) {
                if (utility::file::exists(script_path)) {
                    NUClear::log<NUClear::DEBUG>("Loading script: ", script_path, '\n');
                    script = YAML::LoadFile(script_path).as<::extension::Script>();
                } else {
                    NUClear::log<NUClear::ERROR>("No script found at: ", script_path, '\n');
                }
            }

            void StandEvaluator::saveScript(std::string script_path) {
                YAML::Node n(script);
                utility::file::writeToFile(script_path, n);
            }

            void StandEvaluator::runScript(size_t subsumptionId, NSGA2Evaluator *evaluator) {
                // emit(std::make_unique<ExecuteScriptByName>(subsumptionId, "StandUpFront.yaml")); //Which approach is better?
                evaluator->emit(std::make_unique<extension::ExecuteScript>(subsumptionId, script, NUClear::clock::now()));
            }

            // void StandEvaluator::CheckForStandDone(const RawSensorsMsg& sensors) {
            //     // The acceptable error margin for the target positions
            //     float epsilon = 0.015;

            //     bool l_elbow_ok = std::fabs(sensors.servo.l_elbow.present_position - arms_l_elbow) < epsilon;
            //     bool r_elbow_ok = std::fabs(sensors.servo.r_elbow.present_position - arms_r_elbow) < epsilon;
            //     bool l_shoulder_pitch_ok =
            //         std::fabs(sensors.servo.l_shoulder_pitch.present_position - arms_l_shoulder_pitch) < epsilon;
            //     bool r_shoulder_pitch_ok =
            //         std::fabs(sensors.servo.r_shoulder_pitch.present_position - arms_r_shoulder_pitch) < epsilon;

            //     if (l_elbow_ok && r_elbow_ok && l_shoulder_pitch_ok && r_shoulder_pitch_ok) {
            //         log<NUClear::INFO>("Stand done");
            //         emit(std::make_unique<Event>(Event::StandDone));
            //     }
            // }

            void StandEvaluator::updateMaxFieldPlaneSway(const RawSensors& sensors) {
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
