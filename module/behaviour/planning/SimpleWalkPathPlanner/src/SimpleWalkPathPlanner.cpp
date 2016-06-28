/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SimpleWalkPathPlanner.h"

#include <cmath>
#include "message/behaviour/KickPlan.h"
#include "message/support/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/KickCommand.h"
#include "message/behaviour/MotionCommand.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"


namespace module {
    namespace behaviour {
        namespace planning {

            using message::support::Configuration;
            using message::input::Sensors;
            using message::motion::WalkCommand;
            using message::behaviour::KickPlan;
            using message::behaviour::KickType;
            using message::behaviour::MotionCommand;
            using message::motion::WalkStartCommand;
            using message::motion::WalkStopCommand;
            using message::motion::KickFinished;
            using utility::localisation::transform::RobotToWorldTransform;
            using utility::math::matrix::Transform2D;
            using utility::nubugger::graph;
            using utility::nubugger::drawSphere;

            using message::vision::Ball;
            using message::localisation::Self;

            SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
             : Reactor(std::move(environment)),
             subsumptionId(size_t(this) * size_t(this) - size_t(this)),
             currentTargetPosition(arma::fill::zeros),
             currentTargetHeading(arma::fill::zeros),
             planType(message::behaviour::MotionCommand::Type::StandStill),
             targetHeading({arma::vec2({0,0}),KickType::SCRIPTED})
             {

                //do a little configurating
                on<Configuration>("SimpleWalkPathPlanner.yaml").then([this] (const Configuration& file){

                    turnSpeed = file.config["turnSpeed"].as<float>();
                    forwardSpeed = file.config["forwardSpeed"].as<float>();
                    a = file.config["a"].as<float>();
                    b = file.config["b"].as<float>();

                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    subsumptionId,
                    "Simple Walk Path Planner",
                    {
                        // Limb sets required by the walk engine:
                        std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                        std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                    },
                    [this] (const std::set<LimbID>& givenLimbs) {
                        if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                            // Enable the walk engine.
                            emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                        }
                    },
                    [this] (const std::set<LimbID>& takenLimbs) {
                        if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                            // Shut down the walk engine, since we don't need it right now.
                            emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                        }
                    },
                    [this] (const std::set<ServoID>&) {
                        // nothing
                    }
                }));


                on<Trigger<KickFinished>>().then([this] (const KickFinished&) {
                    emit(std::move(std::make_unique<WalkStartCommand>(1)));
                });

                on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this] (const MotionCommand& cmd) {
                    //save the plan
                    planType = cmd.type;
                });

                on<Every<20, Per<std::chrono::seconds>>
                 , With<std::vector<Ball>>
                 , With<std::vector<Self>>
                 , Sync<SimpleWalkPathPlanner>>().then([this] (
                    const std::vector<Ball>& ball,
                    const std::vector<Self>& selfs) {

                    if (planType == message::behaviour::MotionCommand::Type::StandStill) {

                        emit(std::make_unique<WalkStopCommand>(1));
                        return;

                    }
                    else if (planType == message::behaviour::MotionCommand::Type::DirectCommand) {
                        //TO DO, change to Bezier stuff

                        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(1, Transform2D({currentTargetPosition[0], currentTargetPosition[1], 0.5}));
                        command->command.xy()    = currentTargetPosition;
                        command->command.angle() = currentTargetHeading[0];
                        emit(std::move(command));
                        emit(std::move(std::make_unique<WalkStartCommand>(1)));
                        return;

                    }

                    

                    // TODO: support non-ball targets
                    arma::vec2 position = ball[0].measurements[0].position.rows(0,1);

                    float angle = std::atan2(position[1], position[0]);
                    angle = std::min(turnSpeed, std::max(angle, -turnSpeed));
                    // emit(graph("angle", angle));
                    // emit(graph("ball position", position));
                    // emit(graph("robot position", selfs.front().position));
                    // emit(graph("robot heading", selfs.front().heading));

                    //Euclidean distance to ball
                    float distanceToBall = arma::norm(position);
                    float scale = 2.0 / (1.0 + std::exp(-a * distanceToBall + b)) - 1.0;
                    float scale2 = angle / M_PI;
                    float finalForwardSpeed = forwardSpeed * scale * (1.0 - scale2);
                    // emit(graph("forwardSpeed1", forwardSpeed));
                    // emit(graph("scale", scale));
                    // emit(graph("distanceToBall", distanceToBall));
                    // emit(graph("forwardSpeed2", finalForwardSpeed));

                    std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(1, Transform2D({currentTargetPosition[0], currentTargetPosition[1], 0.5}));
                    command->command = Transform2D({finalForwardSpeed, 0, angle});

                    arma::vec2 ball_world_position = RobotToWorldTransform(selfs.front().position, selfs.front().heading, position);
                    arma::vec2 kick_target = 2 * ball_world_position - selfs.front().position;
                    emit(drawSphere("kick_target", arma::vec3({kick_target[0], kick_target[1], 0.0}), 0.1, arma::vec3({1, 0, 0}), 0));

                    emit(std::make_unique<KickPlan>(KickPlan{kick_target,KickType::SCRIPTED}));
                    emit(std::move(std::make_unique<WalkStartCommand>(1)));
                    emit(std::move(command));

                });

                on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this] (const MotionCommand& cmd) {
                    //save the plan
                    planType = cmd.type;
                    // targetHeading = cmd.kickTargetType;
                    // targetPosition = cmd.targetPositionType;
                    // currentTargetPosition = cmd.kickTarget;
                    currentTargetHeading = std::atan2(cmd.kickTarget[1],cmd.kickTarget[0]);
                });

            }

        }  // planning
    }  // behaviours
}  // modules
