#include "NSGA2Evaluator.hpp"

#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/support/optimisation/NSGA2EvaluationRequest.hpp"
#include "message/support/optimisation/NSGA2FitnessScores.hpp"
#include "message/support/optimisation/NSGA2Terminate.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;

            using message::behaviour::MotionCommand;
            using message::input::Sensors;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;

            using utility::support::Expression;

            NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

                // Read the NSGA2Evaluator.yaml config file and set the walk command parameters
                on<Configuration>("NSGA2Evaluator.yaml").then([this](const Configuration& config) {
                    velocity = config["walk_command"]["velocity"].as<Expression>();
                    rotation = config["walk_command"]["rotation"].as<Expression>();

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
                    walk["first_step_swing_factor"] = request.parameters.first_step_swing_factor;

                    auto foot          = walk["foot"];
                    foot["distance"]   = request.parameters.foot.distance;
                    foot["rise"]       = request.parameters.foot.rise;
                    foot["z_pause"]    = request.parameters.foot.z_pause;
                    foot["apex_phase"] = request.parameters.foot.apex_phase;

                    auto put_down           = foot["put_down"];
                    put_down["z_offset"]    = request.parameters.foot.put_down.z_offset;
                    put_down["phase"]       = request.parameters.foot.put_down.phase;
                    put_down["roll_offset"] = request.parameters.foot.put_down.roll_offset;

                    auto overshoot     = foot["overshoot"];
                    overshoot["ratio"] = request.parameters.foot.overshoot.ratio;
                    overshoot["phase"] = request.parameters.foot.overshoot.phase;

                    auto trunk        = walk["trunk"];
                    trunk["height"]   = request.parameters.trunk.height;
                    trunk["pitch"]    = request.parameters.trunk.pitch;
                    trunk["phase"]    = request.parameters.trunk.phase;
                    trunk["x_offset"] = request.parameters.trunk.x_offset;
                    trunk["y_offset"] = request.parameters.trunk.y_offset;
                    trunk["swing"]    = request.parameters.trunk.swing;
                    trunk["pause"]    = request.parameters.trunk.pause;

                    auto x_offset       = trunk["x_offset"];
                    x_offset["forward"] = request.parameters.trunk.x_offset_p_coef.forward;
                    x_offset["turn"]    = request.parameters.trunk.x_offset_p_coef.turn;

                    auto pitch       = trunk["pitch_p_coef"];
                    pitch["forward"] = request.parameters.trunk.pitch_p_coef.forward;
                    pitch["turn"]    = request.parameters.trunk.pitch_p_coef.turn;

                    auto kick        = walk["kick"];
                    kick["length"]   = request.parameters.kick.length;
                    kick["phase"]    = request.parameters.kick.phase;
                    kick["velocity"] = request.parameters.kick.vel;

                    auto pause        = walk["pause"];
                    pause["duration"] = request.parameters.pause.duration;

                    auto max_step  = walk["max_step"];
                    max_step["x"]  = request.parameters.max_step.x;
                    max_step["y"]  = request.parameters.max_step.y;
                    max_step["z"]  = request.parameters.max_step.z;
                    max_step["xy"] = request.parameters.max_step.xy;

                    auto imu                  = walk["imu"];
                    imu["active"]             = request.parameters.imu.active;
                    imu["pitch"]["threshold"] = request.parameters.imu.pitch.threshold;
                    imu["roll"]["threshold"]  = request.parameters.imu.roll.threshold;

                    auto gains    = walk["gains"];
                    gains["legs"] = request.parameters.gains.legs;

                    // Write the updated config to disk
                    std::ofstream output_file_stream("config/QuinticWalk.yaml");
                    output_file_stream << YAML::Dump(walk_config);
                    output_file_stream.close();

                    // Ensure we are not in the terminating state
                    terminating = false;

                    // Create and send the walk command, which will be evaluated when we get simulation data
                    Eigen::Affine2d walk_command;
                    walk_command.linear()      = Eigen::Rotation2Dd(rotation).toRotationMatrix();
                    walk_command.translation() = velocity;
                    emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(walk_command)));
                });

                // TODO(KipH):
                // ~This shouldn't be listening for sensors. that should be handled by something else~
                // Some of this needs to be handled here (e.g. getting sensor data to set this module's state), but we
                // can probably add a proxy that emits something like OptimisationSensors with just the data we need.
                // on<Trigger<Sensors>, Single>().then([this](const Sensors& sensors) {
                //     // Get the accelerometer sensor data
                //     accelerometer[0] = sensors.accelerometer[0];
                //     accelerometer[1] = sensors.accelerometer[1];
                //     accelerometer[2] = sensors.accelerometer[2];

                //     // Get the gyroscope sensor data
                //     gyroscope[0] = sensors.gyroscope[0];
                //     gyroscope[1] = sensors.gyroscope[1];
                //     gyroscope[2] = sensors.gyroscope[2];

                //     // log("gyroscope.x = " + std::to_string(gyroscope[0]));
                //     // log("gyroscope.y = " + std::to_string(gyroscope[1]));
                //     // log("gyroscope.z = " + std::to_string(gyroscope[2]));
                //     // log("accelerme.x = " + std::to_string(accelerometer[0]));
                //     // log("accelerme.y = " + std::to_string(accelerometer[1]));
                //     // log("accelerme.z = " + std::to_string(accelerometer[2]));

                //     // Calculate the sway based on our gyroscope values over time, after the simulation has
                //     progressed
                //     // past some initial time since the beginning. This is likely used to stabliise the gyro before
                //     we
                //     // start reading its values.
                //     if (simTime > 0.25) {
                //         sway[0] += gyroscope[0] * simTimeDelta;
                //         sway[1] += gyroscope[1] * simTimeDelta;
                //         sway[2] += gyroscope[2] * simTimeDelta;
                //     }

                //     // Determine whether we've fallen over by looking at gravity (accelerometer[2])
                //     if (!terminating && evaluating
                //         && ((std::abs(accelerometer[0]) > 9.2 || std::abs(accelerometer[1]) > 9.2)
                //             && std::abs(accelerometer[2]) < 0.5)
                //         && simTime > 3.0) {
                //         fallenOver = true;
                //     }

                //     // Calculate the robot sway along the field plane (left/right, forward/backward)
                //     fieldPlaneSway = std::pow(std::pow(accelerometer[0], 2) + std::pow(accelerometer[1], 2), 0.5);

                //     // Update max plane sway if the new plane sway is bigger
                //     if (!terminating && evaluating && !finished && fieldPlaneSway > maxFieldPlaneSway
                //         && simTime > 0.25) {
                //         maxFieldPlaneSway = fieldPlaneSway;
                //     }

                //     // Determine when we can terminate early: if we've fallen over, or if the ball is not moving
                //     if (!terminating && evaluating && !finished && simTime > 4.0) {
                //         if (fallenOver) {
                //             BeginTermination();
                //         } else if (ballVelocity[0] == 0.0 && ballVelocity[1] == 0.0 && ballVelocity[2] == 0.0) {
                //             BeginTermination();
                //         }
                //     }
                // });


                // TODO: ~Kip thinks this is dealt with by the webots module~ We need to keep this, to set the
                // simulation time, but we can probably rename GazeboWorldStatus to something more general, like
                // OptimisationTimeUpdate. on<Trigger<GazeboWorldStatus>, Single>().then([this](const GazeboWorldStatus&
                // status) {
                //     // Get the sim time
                //     simTimeDelta = 0;  // status.simTime - simTime;
                //     simTime      = 0;  // status.simTime;

                //     // if (simTime > 3.0 && !walking) {
                //     //    walking = true;
                //     //{std::cout << "running\n";
                //     // std::cout << "EMITTING\n";
                //     // emit(std::make_unique<ExecuteKick>());


                //     //}
                //     // if (simTime > 15.0)
                //     //    fallenOver = true;

                //     // If there's been at least 1 second in the simulation but we're not evaluating or terminating,
                //     // then execute the script (probably stand), reset the time, and start evaluating.
                //     if (simTime > 1.0 && !evaluating & !terminating) {
                //         emit(std::make_unique<ExecuteScript>(subsumptionId, script, NUClear::clock::now()));
                //         ResetWorldTime();
                //         evaluating = true;
                //     }

                //     // If we've been terminating for at least 2 seconds, but are not finished, clear our terminating
                //     // status and send the fitness scores back to the Optimiser
                //     if (terminating && !evaluating && !finished && (simTime - timeSinceTermination) > 2.0) {
                //         terminating = false;
                //         SendFitnessScores();
                //     }
                // });

                on<Trigger<NSGA2Terminate>, Single>().then([this]() {
                    // Set the finished state when we get the termination message
                    finished = true;
                });

                // TODO: ~these updates should be handled by the webots module~ Actually it makes sense to keep them
                // here since they're setting state here. We just need to rename the message to something generic like
                // OptimisationBallLocation.
                // on<Trigger<GazeboBallLocation>, Single>().then([this](const GazeboBallLocation& location) {
                //     // Get the sensory data
                //     // if fallen over, trigger end of evaluation
                //     // if (sensors.)

                //     ballLocation[0] = location.x;
                //     ballLocation[1] = location.y;
                //     ballLocation[2] = location.z;

                //     ballVelocity[0] = location.velx;
                //     ballVelocity[1] = location.vely;
                //     ballVelocity[2] = location.velz;
                //     // log(ballVelocity[0]);
                //     // log(ballVelocity[1]);
                //     // log(ballVelocity[2]);
                // });

                // on<Trigger<GazeboRobotLocation>, Single>().then([this](const GazeboRobotLocation& location) {
                //     // distanceTravelled += location.x + 1;//std::pow(std::pow(location.x - robotLocation[0], 2) +
                //     // std::pow(location.y - robotLocation[1], 2), 0.5);
                //     robotLocationDelta[0] = location.x - robotLocationDelta[0];
                //     robotLocationDelta[1] = location.y - robotLocationDelta[1];
                //     robotLocationDelta[2] = location.z - robotLocationDelta[2];

                //     robotLocation[0] = location.x;
                //     robotLocation[1] = location.y;
                //     robotLocation[2] = location.z;
                // });
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
                // TODO: change this to a webots message that resets the world
                // std::unique_ptr<GazeboWorldCtrl> command = std::make_unique<GazeboWorldCtrl>();
                // command->command                         = "RESET";
                // emit(command);

                // Reset our local state
                distanceTravelled    = 0.0;
                timeSinceTermination = 0.0;
                sway                 = Eigen::Vector3d::Zero();
                fieldPlaneSway       = 0.0;
                maxFieldPlaneSway    = 0.0;
                simTimeDelta         = 0.0;
                constraints          = {0.0, 0.0};
                fallenOver           = false;
            }

            void NSGA2Evaluator::ResetWorldTime() {
                // TODO: change this to a webots message that resets the simulation time
                // std::unique_ptr<GazeboWorldCtrl> command = std::make_unique<GazeboWorldCtrl>();
                // command->command                         = "RESETTIME";
                // emit(command);
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
                emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
            }

            void NSGA2Evaluator::CalculateFitness() {
                // Calculate the distance the ball travelled
                distanceTravelled = std::abs(ballLocation[0]);

                // If the robot fell over, or the ball didn't move much, set distance travelled to a fixed bad number
                if (fallenOver || distanceTravelled < 0.001) {
                    distanceTravelled = 0.001;
                }
                // Limit the distance travelled to 10m
                else if (distanceTravelled > 10.0) {
                    distanceTravelled = 10.0;
                }

                // Ensure maxFieldPlaneSway is positive, since we only care about the magnitude
                maxFieldPlaneSway = std::abs(maxFieldPlaneSway);

                // If the robot didn't sway, or it swayed too much, set max sway to a fixed bad number
                if (maxFieldPlaneSway == 0 || maxFieldPlaneSway > 1000.0) {
                    maxFieldPlaneSway = 1000.0;
                }

                // If the robot fell over, set the first constraint (fallen/not-fallen) to a fixed bad number,
                // and set max sway to a fixed bad number
                if (fallenOver) {
                    constraints[0]    = -10.0;
                    maxFieldPlaneSway = 1000.0;
                }

                // Ensure the sway along the second axis is positive, for use in calculating the second constraint
                // (whether or not we've swayed too far)
                sway[2] = std::abs(sway[2]);

                // If the robot swayed more than 6.6 (a magic number), then it violated the second constraint
                if (sway[2] > 6.66) {
                    constraints[1] = -1.0 * (sway[2] - 6.66);  // Set the constraint based on how much it's over 6.6
                }
                // Otherwise the second constraint wasn't violated
                else {
                    constraints[1] = 0.0;
                }

                // Set the calculated fitness scores
                scores[0] = maxFieldPlaneSway;
                scores[1] = 1.0 / (distanceTravelled);

                // Log the scores
                log(scores[0]);
                log(scores[1]);
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
