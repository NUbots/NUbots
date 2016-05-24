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
#include "message/behaviour/KickPlan.h"
#include "message/support/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/KickCommand.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"


namespace module {
    namespace behaviour {
        namespace planning {

            using message::support::Configuration;
            using message::input::Sensors;
            using message::motion::WalkCommand;
            using message::behaviour::WalkTarget;
            using message::behaviour::WalkApproach;
            using message::behaviour::KickPlan;
            using message::behaviour::MotionCommand;
            using message::motion::WalkStartCommand;
            using message::motion::WalkStopCommand;
            using message::motion::KickFinished;
            using utility::localisation::transform::RobotToWorldTransform;
            using utility::math::matrix::Transform2D;
            using utility::nubugger::graph;
            using utility::nubugger::drawSphere;

            using LocalisationBall = message::localisation::Ball;
            using Self = message::localisation::Self;
            using VisionBall = message::vision::Ball;
            using VisionObstacle = message::vision::Obstacle;

            BezierWalkPathPlanner::BezierWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                //we will initially stand still
                planType = message::behaviour::WalkApproach::StandStill;

                //do a little configurating
                on<Configuration>("BezierWalkPathPlanner.yaml").then([this] (const Configuration& file){

                    turnSpeed = file.config["turnSpeed"].as<float>();
                    forwardSpeed = file.config["forwardSpeed"].as<float>();
                    a = file.config["a"].as<float>();
                    b = file.config["b"].as<float>();

                });


                on<Trigger<KickFinished>>().then([this] (const KickFinished&) {
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                });

                on<Every<20, Per<std::chrono::seconds>>
                 , With<message::localisation::Ball>
                 , With<std::vector<message::localisation::Self>>
                 , With<Optional<std::vector<message::vision::Obstacle>>>
                 , Sync<BezierWalkPathPlanner>>().then([this] (
                     const LocalisationBall& ball,
                     const std::vector<Self>& selfs,
                     std::shared_ptr<const std::vector<VisionObstacle>> robots) {

                    if (planType == message::behaviour::WalkApproach::StandStill) {

                        emit(std::make_unique<WalkStopCommand>());
                        return;

                    }
                    else if (planType == message::behaviour::WalkApproach::DirectCommand) {
                        //TO DO, change to Bezier stuff

                        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                        command->command.xy()    = currentTargetPosition;
                        command->command.angle() = currentTargetHeading[0];
                        emit(std::move(command));
                        emit(std::move(std::make_unique<WalkStartCommand>()));
                        return;

                    }

                    //TO DO, change to Bezier stuff
                    // Include direction of goals


                    //Algorithm to optimize d1, d2 for bezier curve
                    // From "A Bezier curve based path planning in a multi-agent robot soccer system without violating the acceleration limits" by K.G. Jolly, R. Sreerama Kumar, R. Vijayakumar

                    // obtain VP, VS, A0, B0, A3, B3
                    float VP = selfs.velocity[0]; //velocity at robot position
                    float VS = 0.1 //velocity at ball position, chosen as 0.1
                    float A0 = selfs.front().position[1];
                    float B0 = selfs.front().position[0];
                    float A3 = ball.position[1];
                    float B3 = ball.position[0];
                    float theta1 = selfs.front().heading; //angle orientation of robot in space
                    float theta2 = std::atan2(kick_target[1], kick_target[0]); //angle wanting to stike ball, angle of ball to goal
                    // Calculate RP, RS
                    float RP = VP^2 / 4; //Minimum radius of curvature required at point P (robot point), 4 = ar is maximum radial acceleration
                    float RS = RS^2 / 4; //Mimimum radius of curvature required at point S (Ball point)

                    // initialize d1,d2,ErMax
                    float d1=1; //Inital value, can be changed
                    float d2=1; //Inital value, can be changed
                    float ErMax = 2; //Assigned Value, can be changed

                    // calculate rhoP, rhoS
                    float h0 = B2-B0;
                    float g0 = A3-A0;
                    float h1 = 2*( h0*std::cos(theta1) - g0*std::sin(theta1) );
                    float h2 = 2*( h0*std::cos(theta2) - g0*std::sin(theta2) );
                    float g1 = 2*sin(theta2 - theta1);
                    float g1 = 2*sin(theta2 - theta1);

                    float rhoP = (3*d1^2)/(h1+d2*g1);
                    float rhoS = (3*d2^2)/(h2+d1*g2);
                    float Er1 = RP - rhoP;
                    float Er2 = RS - rhoS;
                    float error = std::max(std::abs(Er1), std::abs(Er2));
                    while (error >= ErMax) {
                        d1 = d1+Er1/RP;
                        d2 = d2+Er2/RS;
                        // calculate rhoP, rhoS
                        rhoP = (3*d1^2)/(h1+d2*g1);
                        rhoS = (3*d2^2)/(h2+d1*g2);
                        Er1 = RP - rhoP;
                        Er2 = RS - rhoS;
                        error = std::max(std::abs(Er1), std::abs(Er2));
                    }
                    float A1 = A0 +d1 * std::cos(theta1);
                    float B1 = B0 +d1 * std::sin(theta1);
                    float A2 = A3 +d2 * std::cos(M_PI +theta2);
                    float B2 = B3 +d2 *std::sin(M_PI + theta2);


                    //make 2 11 long arrays, to descretize the bezier and bezier derivatives
                    float bezier_X_points[11];
                    float bezier_Y_points[11];
                    float bezXdash[11];
                    float bezYdash[11];
                    //float bezXdashdash[11];
                    //float bezYdashdash[11];
                    for (float u=0;u<=1;u+=0.1) {
                        int i = int(10*u);
                        bezier_X_points[i] = A0*(1-u)*(1-u)*(1-u) +3*A1*u*(1-u)*(1-u) +3*A2*u*u *(1-u) +A3*u*u*u;
                        bezier_Y_points[i] = B0*(1-u)*(1-u)*(1-u) +3*B1*u*(1-u)*(1-u) +3*B2*u*u *(1-u) +B3*u*u*u;
                        bezXdash[i] = 3*(u*u*(-A0+3*A1-3*A2+A3)+ 2*u*(A0-2*A1+A2)-A0+A1);
                        bezYdash[i] = 3*(u*u*(-B0+3*B1-3*B2+B3)+ 2*u*(B0-2*B1+B2)-B0+B1);
                        //bezXdashdash[i] = 6*(A0*(-u)+A0+3*A1*u-2*A1-3*A2 u+A2+A3*u);
                        //bezYdashdash[i] = 6*(B0*(-u)+B0+3*B1*u-2*A1-3*B2 u+B2+B3*u);
                    }

                    arma::vec2 next_robot_position = arma::vec2(bezier_X_points[1], bezier_Y_points[1]);

                    
                    /* More complicated walk path follower, useful with more accurate locomotion
                    //Calculate radius of curvature at each point (using 5)
                    float radius_of_curvature[21];
                    for (int i=0;i<22;i++) {
                        radius_of_curvature[i] = (bezXdash[i]*bezYdashdash[i]-bezYdash[i]*bezXdashdash[i])/pow((bezXdash[i]*bezXdash[i] + bezYdash[i]*bezYdash[i]),3/2);
                    }

                    //Find which points are turning points (using 15), assuming 6 turning points
                    const int N = sizeof(radius_of_curvature) / sizeof(int);
                    int turning_point[6];
                    for (int i=0;i<6;i++) {
                        int turning_point[i] = distance(radius_of_curvature, min_element(radius_of_curvature, radius_of_curvature + N));
                        radius_of_curvature[turning_point[i]] = 1000; //hack to make the point high so it is not picked up again
                    } */

                    //PID controller, which spits out error values compared to use previous curves



                    // TODO: support non-ball targets

                    //float angle = std::atan2(ball.position[1], ball.position[0]);

                    float angle = std::atan2(bezier_X_points[1],bezier_Y_points[1]);
                    angle = std::min(turnSpeed, std::max(angle, -turnSpeed));
                    // emit(graph("angle", angle));
                    // emit(graph("ball position", ball.position));
                    // emit(graph("robot position", selfs.front().position));
                    // emit(graph("robot heading", selfs.front().heading));

                    //Euclidean distance to ball

                    //float distanceToPoint = arma::norm(next_robot_position);
                    //float scale = 2.0 / (1.0 + std::exp(-a * distanceToPoint + b)) - 1.0;
                    //float scale2 = angle / M_PI;
                    //float finalForwardSpeed = forwardSpeed * scale * (1.0 - scale2);
                    
                    // emit(graph("forwardSpeed1", forwardSpeed));
                    // emit(graph("scale", scale));
                    // emit(graph("distanceToBall", distanceToBall));
                    // emit(graph("forwardSpeed2", finalForwardSpeed));

                    

                    std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                    //command->command = Transform2D({finalForwardSpeed, 0, angle});
                    command->command = Transform2D({bezXdash[1], bezYdash[1], angle});

                    arma::vec2 ball_world_position = RobotToWorldTransform(selfs.front().position, selfs.front().heading, ball.position);
                    arma::vec2 kick_target = 2 * ball_world_position - selfs.front().position;
                    emit(drawSphere("kick_target", arma::vec3({kick_target[0], kick_target[1], 0.0}), 0.1, arma::vec3({1, 0, 0}), 0));

                    emit(std::make_unique<KickPlan>(KickPlan{kick_target}));
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                    emit(std::move(command));

                });

                on<Trigger<MotionCommand>, Sync<BezierWalkPathPlanner>>().then([this] (const MotionCommand& cmd) {
                    //save the plan
                    planType = cmd.walkMovementType;
                    targetHeading = cmd.targetHeadingType;
                    targetPosition = cmd.targetPositionType;
                    currentTargetPosition = cmd.target;
                    currentTargetHeading = cmd.heading;
                });

            }

        }  // planning
    }  // behaviours
}  // modules
