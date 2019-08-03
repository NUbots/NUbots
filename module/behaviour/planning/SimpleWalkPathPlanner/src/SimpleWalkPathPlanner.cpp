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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "SimpleWalkPathPlanner.h"

#include <cmath>

#include "extension/Configuration.h"

#include "message/behaviour/KickPlan.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/Subsumption.h"
#include "message/input/Sensors.h"
#include "message/localisation/Ball.h"
#include "message/localisation/Field.h"
#include "message/motion/KickCommand.h"
#include "message/motion/WalkCommand.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Ball.h"

#include "utility/behaviour/Action.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"


namespace module {
namespace behaviour {
    namespace planning {

        using extension::Configuration;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        using message::input::Sensors;

        using message::behaviour::KickPlan;
        using message::behaviour::MotionCommand;
        using message::behaviour::WantsToKick;
        using message::motion::KickFinished;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;
        using VisionBalls = message::vision::Balls;
        using utility::localisation::fieldStateToTransform3D;
        using utility::math::matrix::Rotation2D;
        using utility::math::matrix::Transform2D;
        using utility::math::matrix::Transform3D;
        using utility::nusight::graph;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        using message::motion::DisableWalkEngineCommand;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::WalkStopped;

        using message::localisation::Ball;
        using message::localisation::Field;
        using message::support::FieldDescription;


        SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , latestCommand(utility::behaviour::StandStill())
            , subsumptionId(size_t(this) * size_t(this) - size_t(this))
            , currentTargetPosition(arma::fill::zeros)
            , currentTargetHeading(arma::fill::zeros)
            , targetHeading(Eigen::Vector2d::Zero(), KickPlan::KickType::SCRIPTED)
            , timeBallLastSeen(NUClear::clock::now()) {

            // do a little configurating
            on<Configuration>("SimpleWalkPathPlanner.yaml").then([this](const Configuration& file) {
                turnSpeed            = file.config["turnSpeed"].as<float>();
                forwardSpeed         = file.config["forwardSpeed"].as<float>();
                sideSpeed            = file.config["sideSpeed"].as<float>();
                a                    = file.config["a"].as<float>();
                b                    = file.config["b"].as<float>();
                search_timeout       = file.config["search_timeout"].as<float>();
                robot_ground_space   = file.config["robot_ground_space"].as<bool>();
                ball_approach_dist   = file.config["ball_approach_dist"].as<float>();
                slowdown_distance    = file.config["slowdown_distance"].as<float>();
                useLocalisation      = file.config["useLocalisation"].as<bool>();
                slow_approach_factor = file.config["slow_approach_factor"].as<float>();

                emit(std::make_unique<WantsToKick>(false));
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "Simple Walk Path Planner",
                {
                    // Limb sets required by the walk engine:
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                },
                [this](const std::set<LimbID>& givenLimbs) {
                    if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                        // Enable the walk engine.
                        emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<LimbID>& takenLimbs) {
                    if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                        // Shut down the walk engine, since we don't need it right now.
                        emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<ServoID>&) {
                    // nothing
                }}));

            on<Trigger<WalkStopped>>().then([this] {
                emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {0, 0}}));
            });

            // on<Trigger<std::vector<Ball>>>().then([this]{
            //     log("std::vector<Ball>");
            // });
            // on<Trigger<std::vector<Self>>>().then([this]{
            //     log("std::vector<Self>");
            // });
            // on<Trigger<KickPlan>>().then([this]{
            //     log("KickPlan");
            // });
            // on<Trigger<WantsToKick>>().then([this]{
            //     log("WantsToKick");
            // });
            // on<Trigger<Sensors>>().then([this]{
            //     log("Sensors");
            // });

            on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
                if (balls.balls.size() > 0) {
                    timeBallLastSeen = NUClear::clock::now();
                }
            });

            on<Every<20, Per<std::chrono::seconds>>,
               With<Ball>,
               With<Field>,
               With<Sensors>,
               With<WantsToKick>,
               With<KickPlan>,
               With<FieldDescription>,
               Sync<SimpleWalkPathPlanner>>()
                .then([this](const Ball& ball,
                             const Field& field,
                             const Sensors& sensors,
                             const WantsToKick& wantsTo,
                             const KickPlan& kickPlan,
                             const FieldDescription& fieldDescription) {
                    if (wantsTo.kick) {
                        emit(std::make_unique<StopCommand>(subsumptionId));
                        return;
                    }

                    if (latestCommand.type == message::behaviour::MotionCommand::Type::StandStill) {


                        emit(std::make_unique<StopCommand>(subsumptionId));
                        // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 40, 11 }}));

                        return;
                    }
                    else if (latestCommand.type == message::behaviour::MotionCommand::Type::DirectCommand) {
                        // TO DO, change to Bezier stuff
                        std::unique_ptr<WalkCommand> command =
                            std::make_unique<WalkCommand>(subsumptionId, latestCommand.walkCommand);
                        emit(std::move(command));
                        emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {40, 11}}));
                        return;
                    }

                    Transform3D Htw = convert(sensors.Htw);
                    auto now        = NUClear::clock::now();
                    float timeSinceBallSeen =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(now - timeBallLastSeen).count()
                        * (1 / std::nano::den);


                    arma::vec3 rBWw_temp = {ball.position[0], ball.position[1], fieldDescription.ball_radius};
                    rBWw                 = timeSinceBallSeen < search_timeout ? rBWw_temp :  // Place last seen
                               Htw.i().x() + Htw.i().translation();                          // In front of the robot
                    arma::vec3 pos = Htw.transformPoint(rBWw);
                    position       = pos.rows(0, 1);

                    // Hack Planner:
                    float headingChange = 0;
                    float sideStep      = 0;
                    float speedFactor   = 1;
                    if (useLocalisation) {

                        // Transform kick target to torso space
                        Transform3D Hfw = fieldStateToTransform3D(convert(field.position));
                        Transform3D Htf = (Htw * Hfw.i());
                        arma::vec3 kickTarget =
                            Htf.transformPoint(arma::vec3({kickPlan.target[0], kickPlan.target[1], 0}));

                        // //approach point:
                        arma::vec2 ballToTarget = arma::normalise(kickTarget.rows(0, 1) - position);
                        arma::vec2 kick_point   = position - ballToTarget * ball_approach_dist;

                        if (arma::norm(position) > slowdown_distance) {
                            position = kick_point;
                        }
                        else {
                            speedFactor   = slow_approach_factor;
                            headingChange = std::atan2(ballToTarget[1], ballToTarget[0]);
                            sideStep      = 1;
                        }
                    }
                    // arma::vec2 ball_world_position = WorldToRobotTransform(selfs.front().position,
                    // selfs.front().heading, position);


                    float angle = std::atan2(position[1], position[0]) + headingChange;
                    // log("ball bearing", angle);
                    angle = std::min(turnSpeed, std::max(angle, -turnSpeed));
                    // log("turnSpeed", turnSpeed);
                    // log("ball bearing", angle);
                    // log("ball position", position);
                    // log("loc position", selfs.front().position.t());
                    // log("loc heading", selfs.front().heading);

                    // Euclidean distance to ball
                    float scaleF            = 2.0 / (1.0 + std::exp(-a * std::fabs(position[0]) + b)) - 1.0;
                    float scaleF2           = angle / M_PI;
                    float finalForwardSpeed = speedFactor * forwardSpeed * scaleF * (1.0 - scaleF2);

                    float scaleS         = 2.0 / (1.0 + std::exp(-a * std::fabs(position[1]) + b)) - 1.0;
                    float scaleS2        = angle / M_PI;
                    float finalSideSpeed = -speedFactor * ((0 < position[1]) - (position[1] < 0)) * sideStep * sideSpeed
                                           * scaleS * (1.0 - scaleS2);
                    // log("forwardSpeed1", forwardSpeed);
                    // log("scale", scale);
                    // log("distanceToBall", distanceToBall);
                    // log("forwardSpeed2", finalForwardSpeed);


                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId, convert(Transform2D({0, 0, 0})));
                    command->command = convert(Transform2D({finalForwardSpeed, finalSideSpeed, angle}));

                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {40, 11}}));
                });

            on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
                // save the plan
                latestCommand = cmd;
            });
        }

    }  // namespace planning
}  // namespace behaviour
}  // namespace module
