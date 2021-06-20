#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/WebotsTimeUpdate.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2EvaluationRequest.hpp"
#include "message/support/optimisation/NSGA2FitnessScores.hpp"
#include "message/support/optimisation/NSGA2Terminate.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;
            using extension::ExecuteScriptByName;

            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::motion::StopCommand;
            using message::motion::WalkCommand;
            using message::platform::RawSensors;
            using message::platform::webots::OptimisationCommand;
            using message::platform::webots::OptimisationRobotPosition;
            using message::platform::webots::WebotsTimeUpdate;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;

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

                    // Reset our state and the simulation since the config has changed
                    ResetWorld();
                });

                // Reset our state and the simulation on system startup
                on<Startup>().then([this]() { ResetWorld(); });

                on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
                    // Set our genration and individual identifiers from the request
                    generation = request.generation;
                    id         = request.id;

                    // Reset the simulation time for the new evaluation
                    ResetWorldTime();

                    // Read the QuinticWalk config and overwrite the config parameters with the current individual's
                    // parameters
                    YAML::Node walk_config = YAML::LoadFile("config/QuinticWalk.yaml");

                    auto walk                       = walk_config["walk"];
                    walk["freq"]                    = request.parameters.freq;
                    walk["double_support_ratio"]    = request.parameters.double_support_ratio;

                    auto foot          = walk["foot"];
                    foot["distance"]   = request.parameters.foot.distance;
                    foot["rise"]       = request.parameters.foot.rise;

                    auto trunk        = walk["trunk"];
                    trunk["height"]   = request.parameters.trunk.height;
                    trunk["pitch"]    = request.parameters.trunk.pitch;
                    trunk["x_offset"] = request.parameters.trunk.x_offset;
                    trunk["y_offset"] = request.parameters.trunk.y_offset;
                    trunk["swing"]    = request.parameters.trunk.swing;
                    trunk["pause"]    = request.parameters.trunk.pause;

                    auto pause        = walk["pause"];
                    pause["duration"] = request.parameters.pause.duration;

                    auto gains    = walk["gains"];
                    gains["legs"] = request.parameters.gains.legs;

                    // Write the updated config to disk
                    std::ofstream output_file_stream("config/QuinticWalk.yaml");
                    output_file_stream << YAML::Dump(walk_config);
                    output_file_stream.close();

                    // Ensure we are not in the terminating state
                    terminating = false;
                });

                on<Trigger<RawSensors>, Single>().then([this](const RawSensors& sensors) {
                    // Get the accelerometer sensor data
                    accelerometer[0] = sensors.accelerometer.x;
                    accelerometer[1] = sensors.accelerometer.y;
                    accelerometer[2] = sensors.accelerometer.z;

                    // Get the gyroscope sensor data
                    gyroscope[0] = sensors.gyroscope.x;
                    gyroscope[1] = sensors.gyroscope.y;
                    gyroscope[2] = sensors.gyroscope.z;

                    // log("gyro.x = " + std::to_string(gyroscope[0]));
                    // log("gyro.y = " + std::to_string(gyroscope[1]));
                    // log("gyro.z = " + std::to_string(gyroscope[2]));
                    // log("accl.x = " + std::to_string(accelerometer[0]));
                    // log("accl.y = " + std::to_string(accelerometer[1]));
                    // log("accl.z = " + std::to_string(accelerometer[2]));

                    // // Calculate the sway based on our gyroscope values over time, after the simulation has progressed
                    // // past some initial time since the beginning. This is likely used to stabalise the gyro before we
                    // // start reading its values.
                    // if (simTime > 0.25) {
                    //     sway[0] += gyroscope[0] * simTimeDelta; // multiplying by time integrates the gyro
                    //     sway[1] += gyroscope[1] * simTimeDelta;
                    //     sway[2] += gyroscope[2] * simTimeDelta;
                    // }

                    // Determine whether we've fallen over by looking at gravity (accelerometer[2])
                    if (!terminating && evaluating
                        && ((std::abs(accelerometer[0]) > 9.2 || std::abs(accelerometer[1]) > 9.2)
                            && std::abs(accelerometer[2]) < 0.5)
                        && simTime > 3.0) {
                        fallenOver = true;
                    }

                    // // Calculate the robot sway along the field plane (left/right, forward/backward)
                    // fieldPlaneSway = std::pow(std::pow(accelerometer[0], 2) + std::pow(accelerometer[1], 2), 0.5);

                    // // Update max plane sway if the new plane sway is bigger
                    // if (!terminating && evaluating && !finished && fieldPlaneSway > maxFieldPlaneSway
                    //     && simTime > 0.25) {
                    //     maxFieldPlaneSway = fieldPlaneSway;
                    // }

                    // Determine when we can terminate early: if we've fallen over
                    if (!terminating && evaluating && !finished && simTime > 4.0) {
                        if (fallenOver) {
                            BeginTermination();
                        }
                    }
                });

                on<Trigger<WebotsTimeUpdate>, Single>().then([this](const WebotsTimeUpdate& update) {
                    // Get the sim time
                    simTimeDelta = update.sim_time - simTime;
                    simTime      = update.sim_time;

                    // If there's been at least 1 second in the simulation but we're not evaluating or terminating,
                    // then execute the script (probably stand), reset the time, and start evaluating.
                    if (simTime > 1.0 && !evaluating & !terminating) {
                        emit(std::make_unique<ExecuteScriptByName>(subsumptionId, "Stand.yaml"));
                        ResetWorldTime();
                        evaluating = true;

                        // Create and send the walk command, which will be evaluated when we get simulation data
                        // Eigen::Affine2d walk_command;
                        // walk_command.linear()      = Eigen::Rotation2Dd(walk_command_rotation).toRotationMatrix();
                        // walk_command.translation() = walk_command_velocity;
                        log<NUClear::INFO>(fmt::format("Trialling with walk command: ({}) {}",
                                                       walk_command_velocity.transpose(),
                                                       walk_command_rotation));
                        emit(std::make_unique<WalkCommand>(subsumptionId,
                                                           Eigen::Vector3d(walk_command_velocity.x(),
                                                                           walk_command_velocity.y(),
                                                                           walk_command_rotation)));
                    }

                    // If we've been terminating for at least 2 seconds, but are not finished, clear our terminating
                    // status and send the fitness scores back to the Optimiser
                    if (terminating && !evaluating && !finished && (simTime - timeSinceTermination) > 2.0) {
                        terminating = false;
                        SendFitnessScores();
                    }
                });

                on<Trigger<NSGA2Terminate>, Single>().then([this]() {
                    // Set the finished state when we get the termination message
                    finished = true;
                });

                on<Trigger<OptimisationRobotPosition>, Single>().then([this](const OptimisationRobotPosition& position) {
                    robotDistanceTravelled += std::pow(
                        std::pow(position.value.X - robotPosition[0], 2) +
                        std::pow(position.value.Y - robotPosition[1], 2),
                        0.5
                    );

                    robotPosition[0] = position.value.X;
                    robotPosition[1] = position.value.Y;
                    robotPosition[2] = position.value.Z;
                });
            }

            void NSGA2Evaluator::SendFitnessScores() {
                // Create the fitness scores message based on the results we calculated in CalculateFitness
                // and emit it back to the Optimiser
                std::unique_ptr<NSGA2FitnessScores> fitnessScores = std::make_unique<NSGA2FitnessScores>();
                fitnessScores->id                                 = id;
                fitnessScores->generation                         = generation;
                fitnessScores->objScore                           = scores;
                fitnessScores->constraints                        = constraints;
                emit(fitnessScores);
            }

            void NSGA2Evaluator::ResetWorld() {
                // Tell Webots to reset the world
                std::unique_ptr<OptimisationCommand> resetMsg = std::make_unique<OptimisationCommand>();
                resetMsg->command                             = OptimisationCommand::CommandType::RESET_WORLD;
                emit(resetMsg);

                // Reset our local state
                robotDistanceTravelled = 0.0;
                timeSinceTermination   = 0.0;
                // sway                   = Eigen::Vector3d::Zero();
                fieldPlaneSway         = 0.0;
                maxFieldPlaneSway      = 0.0;
                simTimeDelta           = 0.0;
                constraints            = {0.0, 0.0};
                fallenOver             = false;
            }

            void NSGA2Evaluator::ResetWorldTime() {
                // Tell Webots to reset the time
                std::unique_ptr<OptimisationCommand> resetMsg = std::make_unique<OptimisationCommand>();
                resetMsg->command                             = OptimisationCommand::CommandType::RESET_TIME;
                emit(resetMsg);
            }

            void NSGA2Evaluator::BeginTermination() {
                // Set state to indicate we're starting to terminate
                terminating = true;
                evaluating  = false;

                if (fallenOver) {
                    log<NUClear::DEBUG>("fallen over...");
                }
                else {
                    log<NUClear::DEBUG>("terminating...");
                }

                // Keep track of when we started terminating
                timeSinceTermination = simTime;

                CalculateFitness();

                // Reset the world and make the robot stand still for the next evaluation
                ResetWorld();
                emit(std::make_unique<StopCommand>(subsumptionId));
            }

            void NSGA2Evaluator::CalculateFitness() {
                // // Ensure maxFieldPlaneSway is positive, since we only care about the magnitude
                // maxFieldPlaneSway = std::abs(maxFieldPlaneSway);

                // // If the robot didn't sway, or it swayed too much, set max sway to a fixed bad number
                // if (maxFieldPlaneSway == 0 || maxFieldPlaneSway > 1000.0) {
                //     maxFieldPlaneSway = 1000.0;
                // }

                // If the robot fell over, set the first constraint (fallen/not-fallen) to a fixed bad number,
                // and set max sway to a fixed bad number
                if (fallenOver) {
                    constraints[0]    = -10.0;
                    // maxFieldPlaneSway = 1000.0;
                }

                constraints[1] = 0;

                // // Ensure the sway along the second axis is positive, for use in calculating the second constraint
                // // (whether or not we've swayed too far)
                // sway[2] = std::abs(sway[2]);

                // // If the robot swayed more than 6.6 (a magic number), then it violated the second constraint
                // if (sway[2] > 6.66) {
                //     constraints[1] = -1.0 * (sway[2] - 6.66);  // Set the constraint based on how much it's over 6.6
                // }
                // // Otherwise the second constraint wasn't violated
                // else {
                //     constraints[1] = 0.0;
                // }

                // Set the calculated fitness scores
                scores[0] = 1; // fix the first score as we're trying to optimise only the distance travelled
                scores[1] = 1.0 / robotDistanceTravelled;

                // Log the scores
                log(scores[0]);
                log(scores[1]);
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
