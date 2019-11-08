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

#include "BezierWalkPathPlanner.h"

#include <cmath>

#include "extension/Configuration.h"
#include "message/behaviour/KickPlan.h"
#include "message/behaviour/MotionCommand.h"
#include "message/motion/KickCommand.h"
#include "message/motion/WalkCommand.h"
#include "message/vision/Ball.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/nusight/NUhelpers.h"


namespace module {
namespace behaviour {
    namespace planning {

        using extension::Configuration;

        using message::behaviour::KickPlan;
        using message::behaviour::KickType;
        using message::behaviour::MotionCommand;
        using message::motion::DisableWalkEngineCommand;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::KickFinished;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;
        using message::motion::WalkStopped;
        using LocalisationBall = message::localisation::Ball;
        using Self             = message::localisation::Self;
        using VisionBall       = message::vision::Ball;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        using utility::localisation::transform::RobotToWorldTransform;
        using utility::math::matrix::Transform2D;
        using utility::nusight::drawArrow;
        using utility::nusight::drawSphere;
        using utility::nusight::graph;

        BezierWalkPathPlanner::BezierWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , subsumptionId(size_t(this) * size_t(this) - size_t(this))
            , turnSpeed(0.0f)
            , forwardSpeed(0.0f)
            , a(0.0f)
            , b(0.0f)
            , VP(0.0f)
            , VS(0.0f)
            , d1(0.0f)
            , d2(0.0f)
            , ErMax(0.0f)
            , latestCommand(MotionCommand::StandStill()) {

            // log("LocalisationBall = ",LocalisationBall);
            // log("Self = ",Self);

            // do a little configurating
            on<Configuration>("BezierWalkPathPlanner.yaml").then([this](const Configuration& file) {
                log(__LINE__);
                turnSpeed    = file.config["turnSpeed"].as<float>();
                forwardSpeed = file.config["forwardSpeed"].as<float>();
                a            = file.config["a"].as<float>();
                b            = file.config["b"].as<float>();
                VP           = file.config["VP"].as<float>();
                VS           = file.config["VS"].as<float>();
                d1           = file.config["d1"].as<float>();
                d2           = file.config["d2"].as<float>();
                ErMax        = file.config["ErMax"].as<float>();
            });

            // Register this module with the subsumption system:
            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "Bezier Walk Path Planner",
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

            on<Trigger<KickFinished>>().then([this](const KickFinished&) {
                // May need to tweek this to resume walking after kick completed....
                std::unique_ptr<WalkCommand> command =
                    std::make_unique<WalkCommand>(subsumptionId, latestCommand.walkCommand);
                emit(std::move(command));
                emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));
            });

            on<Trigger<MotionCommand>, Sync<BezierWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
                // save the plan
                latestCommand = cmd;
            });


            on<Every<20, Per<std::chrono::seconds>>,
               With<message::localisation::Ball>,
               With<std::vector<message::localisation::Self>>,
               Sync<BezierWalkPathPlanner>,
               Single>()
                .then(
                    "Updates Bezier Plan",
                    [this](const LocalisationBall& ball,
                           const std::vector<Self>& selfs,
                           std::shared_ptr<const std::vector<VisionObstacle>> /*robots*/) {
                        if (latestCommand.type == MotionCommand::Type::StandStill) {
                            // log("Stand still motion command");

                            emit(std::make_unique<StopCommand>(subsumptionId));
                            emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));

                            return;
                        }
                        else if (latestCommand.type == MotionCommand::Type::DirectCommand) {
                            // TO DO, change to Bezier stuff
                            // log("Direct motion command");

                            std::unique_ptr<WalkCommand> command =
                                std::make_unique<WalkCommand>(subsumptionId, latestCommand.walkCommand);
                            emit(std::move(command));
                            emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));
                        }
                        else {
                            // log("MotionCommand:", int(latestCommand.type));

                            // TODO:use vision ball

                            arma::vec2 ball_world_position =
                                RobotToWorldTransform(selfs.front().position, selfs.front().heading, ball.position);
                            arma::vec2 kick_target =
                                latestCommand.kickTarget;  // 2 * ball_world_position - selfs.front().position;
                            emit(drawSphere("kick_target",
                                            arma::vec3({kick_target[0], kick_target[1], 0.0}),
                                            0.1,
                                            arma::vec3({1, 0, 0}),
                                            0));

                            // log("Kick Target = ",kick_target);


                            // TO DO, change to Bezier stuff
                            // Include direction of goals


                            // Algorithm to optimize d1, d2 for bezier curve
                            // From "A Bezier curve based path planning in a multi-agent robot soccer system without
                            // violating the acceleration limits" by K.G. Jolly, R. Sreerama Kumar, R. Vijayakumar

                            // obtain VP, VS, A0, B0, A3, B3
                            // VP = selfs.velocity[0]; //velocity at robot position
                            // VS = velocity at ball position, chosen as 0.15
                            float A0 = selfs.front().position[1];
                            // log("Self front position 1 = ", A0);

                            float B0 = selfs.front().position[0];
                            // log("Self front position 0 = ", B0);

                            float A3 = ball.position[1];
                            // log("Ball position 1 = ", A3);

                            float B3 = ball.position[0];
                            // log("Ball position 0 = ", B3);

                            float theta1 = 0.5;  // std::atan2(selfs.front().position[1],selfs.front().position[0]);
                                                 // //angle orientation of robot in space
                            float theta2 = std::atan2(
                                kick_target[1], kick_target[0]);  // angle wanting to stike ball, angle of ball to goal
                            // Calculate RP, RS
                            float RP = VP * VP / 4;  // Minimum radius of curvature required at point P (robot point), 4
                                                     // = ar is maximum radial acceleration
                            float RS = VS * VS / 4;  // Mimimum radius of curvature required at point S (Ball point)

                            // initialize d1,d2,ErMax
                            // d1 = Inital value
                            // d2 = Inital value
                            // ErMax = Assigned Value

                            // calculate rhoP, rhoS
                            float h0 = B3 - B0;
                            float g0 = A3 - A0;
                            float h1 = 2 * (h0 * std::cos(theta1) - g0 * std::sin(theta1));
                            float h2 = 2 * (h0 * std::cos(theta2) - g0 * std::sin(theta2));
                            float g1 = 2 * sin(theta2 - theta1);
                            float g2 = 2 * sin(theta2 - theta1);


                            // TO DO Fix algorithm
                            float rhoP  = (3 * d1 * d1) / (h1 + d2 * g1);
                            float rhoS  = (3 * d2 * d2) / (h2 + d1 * g2);
                            float Er1   = RP - rhoP;
                            float Er2   = RS - rhoS;
                            float error = std::max(std::abs(Er1), std::abs(Er2));
                            log("error = ", error, "\n ErMax = ", ErMax);

                            while (error >= ErMax) {
                                d1 = d1 + Er1 / RP;
                                d2 = d2 + Er2 / RS;
                                log("d1 = ", d1, "\n d2 = ", d2);

                                // calculate rhoP, rhoS
                                rhoP  = (3 * d1 * d1) / (h1 + d2 * g1);
                                rhoS  = (3 * d2 * d2) / (h2 + d1 * g2);
                                Er1   = RP - rhoP;
                                Er2   = RS - rhoS;
                                error = std::max(std::abs(Er1), std::abs(Er2));
                                // log("Error = ",error);
                                if (error > 10) {
                                    break;
                                }
                            }

                            d1 = std::min(float(2.0), std::max(d1, float(0.1)));

                            d2 = std::min(float(2.0), std::max(d2, float(0.1)));


                            log("d1 = ", d1, "\n d2 = ", d2);

                            float A1 = A0 + d1 * std::cos(theta1);
                            float B1 = B0 + d1 * std::sin(theta1);
                            float A2 = A3 + d2 * std::cos(M_PI + theta2);
                            float B2 = B3 + d2 * std::sin(M_PI + theta2);

                            emit(drawSphere("Tangent 1", arma::vec3({B1, A1, 0.0}), 0.1, arma::vec3({0, 1, 0}), 0));
                            emit(drawSphere("Tangent 2", arma::vec3({B2, A2, 0.0}), 0.1, arma::vec3({0, 0, 1}), 0));

                            // log("A1 = ", A1, "\n B1 = ", B1, "\n A2 = ", A2 , "\n B2 = ", B2);

                            // make 2 11 long arrays, to descretize the bezier and bezier derivatives
                            // float bezier_X_point;
                            // float bezier_Y_point;
                            // float bezXdash;
                            // float bezYdash;
                            double u =
                                0.1;  // variable determining how long along bezier curve robot looks, to move to config

                            // float bezXdashdash[11];
                            // float bezYdashdash[11];

                            float bezier_X_point = A0 * (1 - u) * (1 - u) * (1 - u) + 3 * A1 * u * (1 - u) * (1 - u)
                                                   + 3 * A2 * u * u * (1 - u) + A3 * u * u * u;
                            float bezier_Y_point = B0 * (1 - u) * (1 - u) * (1 - u) + 3 * B1 * u * (1 - u) * (1 - u)
                                                   + 3 * B2 * u * u * (1 - u) + B3 * u * u * u;
                            // float bezXdash = 3*(u*u*(-A0+3*A1-3*A2+A3)+ 2*u*(A0-2*A1+A2)-A0+A1);
                            // float bezYdash = 3*(u*u*(-B0+3*B1-3*B2+B3)+ 2*u*(B0-2*B1+B2)-B0+B1);
                            // bezXdashdash[i] = 6*(A0*(-u)+A0+3*A1*u-2*A1-3*A2 u+A2+A3*u);
                            // bezYdashdash[i] = 6*(B0*(-u)+B0+3*B1*u-2*A1-3*B2 u+B2+B3*u);

                            arma::fmat bez_matrix;
                            bez_matrix << bezier_X_point << bezier_Y_point << arma::endr << A0 << B0;


                            arma::fvec2 next_robot_position = arma::mean(bez_matrix).t();

                            // log("Robot next position = ", next_robot_position);

                            /* More complicated walk path follower, useful with more accurate locomotion
                            //Calculate radius of curvature at each point (using 5)
                            float radius_of_curvature[21];
                            for (int i=0;i<22;i++) {
                                radius_of_curvature[i] =
                            (bezXdash[i]*bezYdashdash[i]-bezYdash[i]*bezXdashdash[i])/pow((bezXdash[i]*bezXdash[i] +
                            bezYdash[i]*bezYdash[i]),3/2);
                            }

                            //Find which points are turning points (using 15), assuming 6 turning points
                            const int N = sizeof(radius_of_curvature) / sizeof(int);
                            int turning_point[6];
                            for (int i=0;i<6;i++) {
                                int turning_point[i] = distance(radius_of_curvature, min_element(radius_of_curvature,
                            radius_of_curvature + N));
                                radius_of_curvature[turning_point[i]] = 1000; //hack to make the point high so it is not
                            picked up again
                            } */

                            // PID controller, which spits out error values compared to use previous curves


                            // TODO: support non-ball targets

                            float angle = std::atan2(ball.position[1], ball.position[0]);

                            // log("Angle 1 = ", angle);

                            // float angle = std::atan2(bezier_X_point,bezier_Y_point);
                            angle = std::min(turnSpeed, std::max(angle, -turnSpeed));
                            // log("Angle 2 = ", angle);

                            // emit(graph("angle", angle));
                            // emit(graph("ball position", ball.position));
                            // emit(graph("robot position", selfs.front().position));
                            // emit(graph("robot heading", selfs.front().heading));

                            // Euclidean distance to ball

                            float distanceToPoint   = arma::norm(next_robot_position);
                            float scale             = 2.0 / (1.0 + std::exp(-a * distanceToPoint + b)) - 1.0;
                            float scale2            = angle / M_PI;
                            float finalForwardSpeed = forwardSpeed * scale * (1.0 - scale2);
                            // log("Final Forward Speed = ", finalForwardSpeed);

                            // emit(graph("forwardSpeed1", forwardSpeed));
                            // emit(graph("scale", scale));
                            // emit(graph("distanceToBall", distanceToBall));
                            // emit(graph("forwardSpeed2", finalForwardSpeed));

                            std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(
                                subsumptionId, Transform2D({finalForwardSpeed, 0, angle}));
                            // command->command = Transform2D({bezXdash[1], bezYdash[1], angle});
                            emit(std::move(command));
                            emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));
                        }
                    });
        }

    }  // namespace planning
}  // namespace behaviour
}  // namespace module
