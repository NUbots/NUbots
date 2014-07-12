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

#include "WalkPathPlanner.h"

#include <cmath>
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "utility/nubugger/NUgraph.h"


namespace modules {
    namespace behaviour {
        namespace planning {

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::motion::WalkCommand;
            using messages::behaviour::WalkTarget;
            using messages::behaviour::WalkApproach;
            using messages::motion::WalkStartCommand;
            using messages::motion::WalkStopCommand;
            using messages::motion::KickFinished;
            using utility::nubugger::graph;

            using LocalisationBall = messages::localisation::Ball;
            using Self = messages::localisation::Self;
            using VisionBall = messages::vision::Ball;
            using VisionObstacle = messages::vision::Obstacle;

            //using namespace messages;

            //using messages::input::ServoID;
            //using messages::motion::ExecuteScriptByName;
            //using messages::behaviour::RegisterAction;
            //using messages::behaviour::ActionPriorites;
            //using messages::behaviour::LimbID;

            WalkPathPlanner::WalkPathPlanner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                //we will initially stand still
                planType = messages::behaviour::WalkApproach::StandStill;

                //do a little configurating
                on<Trigger<Configuration<WalkPathPlanner>>>([this] (const Configuration<WalkPathPlanner>& file){

                    turnSpeed = file.config["turnSpeed"].as<float>();
                    forwardSpeed = file.config["forwardSpeed"].as<float>();
                    footSeparation = file.config["footSeparation"].as<float>();

                    footSize = file.config["footSize"].as<float>();

                    //timers for starting turning and walking
                    walkStartTime = file.config["walkStartTime"].as<double>();
                    walkTurnTime = file.config["walkTurnTime"].as<double>();

                    //walk accel/deccel controls
                    accelerationTime = file.config["accelerationTime"].as<double>();
                    accelerationFraction = file.config["accelerationFraction"].as<float>();

                    //approach speeds
                    closeApproachSpeed = file.config["closeApproachSpeed"].as<float>();
                    closeApproachDistance = file.config["closeApproachDistance"].as<float>();
                    midApproachSpeed = file.config["midApproachSpeed"].as<float>();
                    midApproachDistance = file.config["midApproachDistance"].as<float>();

                    //turning values
                    turnDeviation = file.config["turnDeviation"].as<float>();

                    //hystereses
                    distanceHysteresis = file.config["distanceHysteresis"].as<float>();
                    turningHysteresis = file.config["turningHysteresis"].as<float>();
                    positionHysteresis = file.config["positionHysteresis"].as<float>();

                    //ball lineup
                    //XXX: add these once we know what they do
                    //vector<float> ballApproachAngle = file.config["ballApproachAngle"];
                    //svector<int> ballKickFoot = file.config["ballKickFoot"];
                    ballLineupDistance = file.config["ballLineupDistance"].as<float>();
                    ballLineupMinDistance = file.config["ballLineupMinDistance"].as<float>();

                    //extra config options
                    useAvoidance = file.config["useAvoidance"].as<bool>();
                    assumedObstacleWidth = file.config["assumedObstacleWidth"].as<float>();
                    avoidDistance = file.config["avoidDistance"].as<float>();

                    bearingSensitivity = file.config["bearingSensitivity"].as<float>();
                    ApproachCurveFactor = file.config["ApproachCurveFactor"].as<float>();
                });


                on<Trigger<KickFinished>>([this] (const KickFinished&) {
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                });

                // add in fake walk localisation
                /*
                on<Trigger<Every<3, Per<std::chrono::seconds>>>, With<Optional<WalkCommand>>>
                    ([this](const time_t&, const std::shared_ptr<const WalkCommand>& w) {
                    static size_t ctr = 0;

                    const double ballx = 0.0;
                    const double bally = 0.0;
                    const double targetx = 3.0;
                    const double targety = 0.0;

                    static double x,y,h;

                    if ( (ctr % 1000000000) == 0) {
                        //set position
                        x = -1.0;
                        y = -2.0;
                        h = -1.5;

                        //emit ball pos
                        std::unique_ptr<messages::localisation::Ball> b = std::make_unique<messages::localisation::Ball>();
                        b->position = arma::vec({ballx,bally});
                        emit(std::move(b));

                        //emit robots:
                        auto o = std::make_unique<std::vector<messages::vision::Obstacle>>();
                        emit(std::move(o));
                    }

                    //update self pos
                    if (w != NULL) {
                        x += (w->velocity[0]*cos(h) - w->velocity[1]*sin(h)) * 0.015;
                        y += (w->velocity[0]*sin(h) + w->velocity[1]*cos(h)) * 0.015;
                        h += (w->rotationalSpeed) * 0.1;
                    }




                    auto robot_msg = std::make_unique<std::vector<messages::localisation::Self>>();

                    messages::localisation::Self robot_model;
                    robot_model.position = arma::vec({x,y});
                    robot_model.heading = arma::vec({cos(h),sin(h)});
                    robot_model.sr_xx = 0.1;
                    robot_model.sr_xy = 0.1;
                    robot_model.sr_yy = 0.1;
                    robot_msg->push_back(robot_model);

                    emit(std::move(robot_msg));

                    ++ctr;
                    });*/

                on<Trigger<Startup>>([this](const Startup&) {
                    emit(std::make_unique<WalkStartCommand>());
                });

                on<Trigger<Every<20, Per<std::chrono::seconds>>>,
                    With<messages::localisation::Ball>,
                    With<std::vector<messages::localisation::Self>>,
                    With<Optional<std::vector<messages::vision::Obstacle>>>,
                    With<std::vector<messages::vision::Ball>>,
                    Options<Sync<WalkPathPlanner>>
                   >([this] (
                     const time_t&,
                     const LocalisationBall& ball,
                     const std::vector<Self>& selfs,
                     const std::shared_ptr<const std::vector<VisionObstacle>>& robots,
                     const std::vector<VisionBall>&
                    ) {
                    /*if(visionBalls.size()>0){
                        arma::vec ballPosition = ball.position;

                        //Jake walk path planner:
                        auto self = selfs[0];

                        arma::vec goalPosition = arma::vec3({-3,0,1});

                        arma::vec normed_heading = arma::normalise(self.heading);
                        arma::mat worldToRobotTransform = arma::mat33{      normed_heading[0],  normed_heading[1],         0,
                                                                             -normed_heading[1],  normed_heading[0],         0,
                                                                                              0,                 0,         1};

                        worldToRobotTransform.submat(0,2,1,2) = -worldToRobotTransform.submat(0,0,1,1) * self.position;

                        arma::vec homogeneousKickTarget = worldToRobotTransform * goalPosition;
                        arma::vec kickTarget_robot = homogeneousKickTarget.rows(0,1);    //In robot coords
                        arma::vec kickDirection = arma::normalise(kickTarget_robot-ballPosition);    //In robot coords
                        arma::vec kickDirectionNormal = arma::vec2({-kickDirection[1], kickDirection[0]});

                        //float kickTargetBearing_robot = std::atan2(kickTarget_robot[1],kickTarget_robot[0]);
                        float ballBearing = std::atan2(ballPosition[1],ballPosition[0]);

                        //calc self in kick coords
                        arma::vec moveTarget = ballPosition - ballLineupDistance * kickDirection;

                        arma::mat robotToKickFrame = arma::mat33{      kickDirection[0],  kickDirection[1],         0,
                                                                        -kickDirection[1],  kickDirection[0],         0,
                                                                                              0,                 0,         1};
                        robotToKickFrame.submat(0,2,1,2) = -robotToKickFrame.submat(0,0,1,1) * moveTarget;

                        arma::vec selfInKickFrame = robotToKickFrame * arma::vec3({0,0,1});

                        //Hyperboal x >a*sqrt(y^2/a^2 + 1)
                        if(selfInKickFrame[0] > ballLineupDistance * std::sqrt(selfInKickFrame[1]*selfInKickFrame[1]/(ApproachCurveFactor*ApproachCurveFactor) + 1)){   //Inside concave part
                            arma::vec moveTargetA = ballPosition + ballLineupDistance * kickDirectionNormal;
                            arma::vec moveTargetB = ballPosition - ballLineupDistance * kickDirectionNormal;
                            if(arma::norm(moveTargetA) < arma::norm(moveTargetB)){
                                moveTarget = moveTargetA;
                            } else {
                                moveTarget = moveTargetB;
                            }
                        }

                        if(arma::norm(moveTarget) < closeApproachDistance) {
                            moveTarget = moveTarget * closeApproachSpeed;
                        }

                        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();

                        command->velocity = arma::normalise(arma::vec2{moveTarget[0],moveTarget[1]});
                        command->rotationalSpeed = ballBearing;  //vx,vy, alpha
                        emit(graph("Walk command:", command->velocity[0], command->velocity[1], command->rotationalSpeed));
                        emit(std::move(command));//XXX: emit here


                        //emit(std::move(std::make_unique<WalkStartCommand>()));

                    } else {
                        float ballBearing = std::atan2(ball.position[1],ball.position[0]);
                        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                        command->velocity = arma::vec({0,0});
                        command->rotationalSpeed = turnSpeed * (ballBearing > 0 ? 1 : -1 );  //vx,vy, alpha
                        emit(graph("Walk command:", command->velocity[0], command->velocity[1], command->rotationalSpeed));
                        emit(std::move(command));//XXX: emit here
                    }*/

                    //std::cout << "starting path planning" << std::endl;
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    arma::vec normed_heading = arma::normalise(selfs.front().heading);
                    arma::mat robotToWorldRotation;
                    robotToWorldRotation << normed_heading[0] << -normed_heading[1] << arma::endr
                                         << normed_heading[1] <<  normed_heading[0];
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    arma::vec ballPos = robotToWorldRotation * arma::vec(ball.position) + arma::vec(selfs.front().position);
                    //std::cout << "ball pos found" << std::endl;
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    arma::vec targetPos,targetHead;
                    //work out where we're going
                    if (targetPosition == messages::behaviour::WalkTarget::Robot) {
                        //XXX: check if robot is visible
                    } else if (targetPosition ==messages::behaviour::WalkTarget::Ball) {
                        targetPos = ballPos;
                    } else { //other types default to position/waypoint location
                        targetPos = currentTargetPosition;
                    }
                    //work out where to face when we get there
                    if (targetHeading == messages::behaviour::WalkTarget::Robot) {
                        //XXX: check if robot is visible
                    } else if (targetHeading == messages::behaviour::WalkTarget::Ball) {
                        targetHead = arma::normalise(ballPos-targetPos);
                    } else { //other types default to position/waypoint bearings
                        targetHead = arma::normalise(arma::vec(currentTargetHeading)-targetPos);
                    }
                    //calculate the basic movement plan
                    arma::vec movePlan;

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    switch (planType) {
                        case messages::behaviour::WalkApproach::ApproachFromDirection:
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                            movePlan = approachFromDirection(selfs.front(),targetPos,targetHead);
                            break;
                        case messages::behaviour::WalkApproach::WalkToPoint:
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                            movePlan = goToPoint(selfs.front(),targetPos,targetHead);
                            break;
                        case messages::behaviour::WalkApproach::OmnidirectionalReposition:
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                            movePlan = goToPoint(selfs.front(),targetPos,targetHead);
                            break;
                        case messages::behaviour::WalkApproach::StandStill:
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                            emit(std::make_unique<WalkStopCommand>());
                            return;
                    }
                    //std::cout << "Target Position: " << targetPos[0] << ", " << targetPos[1] << std::endl;
                    //std::cout << "Self Position: " << selfs.front().position[0] << ", " << selfs.front().position[1] << std::endl;
                    //std::cout << "Self Heading: " << selfs.front().heading[0] << ", " << selfs.front().heading[1] << std::endl;

                    //std::cout << "Move Plan: " << movePlan[0] << ", " << movePlan[1] << ", " << movePlan[2] << std::endl;
                    //work out if we have to avoid something
                    if (useAvoidance && (robots != NULL)) {
                        //this is a vision-based temporary for avoidance
                        movePlan = avoidObstacles(*robots,movePlan);
                    }
                    //NUClear::log("Move Plan:", movePlan[0],movePlan[1],movePlan[2]);
                    // NUClear::log("Move Plan:", movePlan[0],movePlan[1],movePlan[2]);
                    //this applies acceleration/deceleration and hysteresis to movement
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    movePlan = generateWalk(movePlan,
                               planType == messages::behaviour::WalkApproach::OmnidirectionalReposition);
                    std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                    command->velocity = arma::vec({movePlan[0],movePlan[1]});
                    command->rotationalSpeed = movePlan[2];
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                    // NUClear::log("Self Position:", selfs[0].position[0],selfs[0].position[1]);
                    // NUClear::log("Target Position:", targetPos[0],targetPos[1]);
                    emit(graph("Walk command:", command->velocity[0], command->velocity[1], command->rotationalSpeed));
                    //std::cout << "Walk command: " << command->velocity[0] << ", " << command->velocity[1] << ", " << command->rotationalSpeed << std::endl;
                    // NUClear::log("Ball Position:", ball.position[0],ball.position[1]);
                    //std::cout << command->velocity << std::endl;
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                    emit(std::move(command));//XXX: emit here

                });

                on<Trigger<messages::behaviour::WalkStrategy>, Options<Sync<WalkPathPlanner>>>([this] (const messages::behaviour::WalkStrategy& cmd) {
                    //reset hysteresis variables when a command changes
                    turning = 0;
                    distanceIncrement = 3;

                    //save the plan
                    planType = cmd.walkMovementType;
                    targetHeading = cmd.targetHeadingType;
                    targetPosition = cmd.targetPositionType;
                    currentTargetPosition = cmd.target;
                    currentTargetHeading = cmd.heading;
                });

                //Walk planning testing: Walk to ball face to goal
/*
                auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
                approach->targetPositionType = WalkTarget::Ball;
                approach->targetHeadingType = WalkTarget::WayPoint;
                approach->walkMovementType = WalkApproach::ApproachFromDirection;
                approach->heading = arma::vec({-3,0});
                approach->target = arma::vec({0,0});
                emit(std::move(approach));
*/

            }

            arma::vec WalkPathPlanner::generateWalk(const arma::vec& move, bool omniPositioning) {

                //this uses hystereses to provide a stable, consistent positioning and movement
                double walk_speed = 0.0;
                double walk_bearing = 0.0;
                double walk_rotation = 0.0;

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //check what distance increment we're in and swap/set speed accordingly:
                if ((move[0] > midApproachDistance + distanceHysteresis and distanceIncrement < 3)
                    or
                    (move[0] > midApproachDistance and distanceIncrement >= 3)) {

                    distanceIncrement = 3;
                    walk_speed = forwardSpeed;
                }
                else if ((move[0] > closeApproachDistance+distanceHysteresis and distanceIncrement < 2)
                         or
                         (move[0] > closeApproachDistance and distanceIncrement >= 2)) {

                    distanceIncrement = 2;
                    walk_speed = midApproachSpeed;
                }
                else if ((move[0] > ballLineupDistance+distanceHysteresis and distanceIncrement < 1)
                         or
                         (move[0] > ballLineupDistance and distanceIncrement >= 1)) {

                    distanceIncrement = 1;
                    walk_speed = closeApproachSpeed;
                }
                else if (move[0] < ballLineupDistance + distanceHysteresis) {
                    distanceIncrement = 0;
                    walk_speed = 0.f;
                }

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //decide between heading and bearing
                if ((distanceIncrement > 1) && (!omniPositioning)) {
                    walk_rotation = move[1];
                } else {
                    walk_bearing = move[1];
                    walk_rotation = move[2];
                }

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //make sure our rotation is normalised to our turning limits
                walk_rotation = fmin(turnSpeed,fmax(walk_rotation,-turnSpeed));

                //apply turning hysteresis
                /*if (turning < 0 and walk_bearing < -turnDeviation) {
                    //walk_speed = std::min(walk_bearing,turnSpeed);
                } else if (m_turning > 0 and walk_bearing > turnDeviation) {
                    //walk_speed = std::min(walk_bearing,turnSpeed);
                } else {
                    walk_bearing = 0;
                }*/

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //Replacing turn hysteresis with a unicorn equation
                float g = 1./(1.+std::exp(-4.*walk_rotation*walk_rotation));

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                return arma::vec({walk_speed*(1.-g)*cos(walk_bearing),walk_speed*(1.-g)*sin(walk_bearing),walk_rotation*g});
            }

            arma::vec WalkPathPlanner::approachFromDirection(const Self& self,
                                                             const arma::vec2& target,
                                                             const arma::vec2& direction) {

                //this method calculates the possible ball approach commands for the robot
                //and then chooses the lowest cost action
                std::vector<arma::vec2> waypoints(3);

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //calculate the values we need to set waypoints
                const double ballDistance = arma::norm(target - self.position, 2);
                const double selfHeading = atan2(self.heading[1], self.heading[0]);

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //create our waypoints - these are approach vectors offset from the ball
                waypoints[0] = -direction * ballDistance * ApproachCurveFactor;
                waypoints[1] = arma::vec2({waypoints[0][1], -waypoints[0][0]});
                waypoints[2] = arma::vec2({-waypoints[0][1], waypoints[0][0]});

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //offset the waypoints by the foot separation so we are aiming at the ball with the correct foot
                arma::vec2 footOffset = arma::normalise(waypoints[0]) * (footSeparation + footSize) * 0.5;

                //do a foot offset for the straight approach case
                if (arma::dot(waypoints[1], waypoints[1]) < arma::dot(waypoints[1], waypoints[1])) {
                    waypoints[0] += arma::normalise(waypoints[1]) * (footSeparation + footSize) * 0.5;
                } else {
                    waypoints[0] += arma::normalise(waypoints[2]) * (footSeparation + footSize) * 0.5;
                }

                //add the foot offsets for sidekicks to the side approach case
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                waypoints[1] += footOffset;
                waypoints[2] += footOffset;

                //fill target headings and distances, and movement bearings and costs
                std::vector<double> headings(3);
                std::vector<double> distances(3);
                std::vector<double> bearings(3);
                std::vector<double> costs(3);

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                for (size_t i = 0; i < 3; ++i) {
                    //store the directions we want to face when we get to the ball in headings
                    const double waypointHeading = atan2(-waypoints[i][1], -waypoints[i][0]) - selfHeading;
                    headings[i] = atan2(sin(waypointHeading), cos(waypointHeading));

                    //calculate the estimated distance to destination
                    arma::vec waypointPos = waypoints[i] + target - self.position;
                    distances[i] = arma::norm(waypointPos, 2);

                    //calculate the angle between the current direction and the bearing of the destination
                    const double waypointBearing = atan2(waypointPos[1], waypointPos[0]) - selfHeading;
                    bearings[i] = atan2(sin(waypointBearing), cos(waypointBearing));

                    //costs defines which move plan is the most appropriate - the sensitivity gives us control over whether the robots prefers translation or rotation
                    costs[i] = bearings[i] * bearings[i] * bearingSensitivity + distances[i] * distances[i];

                }

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                //decide which approach to the ball is cheapest
                size_t selected;
                if (costs[0] < costs[1] and costs[0] < costs[2]) {
                    selected = 0;
                } else if (costs[1] < costs[2]) {
                    selected = 1;
                } else {
                    selected = 2;
                }

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                return arma::vec({distances[selected], bearings[selected], headings[selected]});
            }

            arma::vec WalkPathPlanner::goToPoint(const Self& self,
                                                  const arma::vec2& target,
                                                  const arma::vec2& direction) {
                //quick and dirty go to point calculator
                //gets position and heading difference and returns walk params for it
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                const arma::vec2 posdiff = target - self.position;
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                const double targetDistance = arma::norm(posdiff);
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                const double selfHeading = std::atan2(self.heading[1],self.heading[0]);
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                const double targetHeading = std::atan2(posdiff[1],posdiff[0])-selfHeading;
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                const double targetBearing = std::atan2(direction[1],direction[0])-selfHeading;

//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                arma::vec result(3);
//std::cerr << __FILE__ << ":" << __func__ << " - " << __LINE__ << std::endl;
                result[0] = targetDistance;
                result[1] = atan2(sin(targetHeading),cos(targetHeading));
                result[2] = atan2(sin(targetBearing),cos(targetBearing));

//std::cerr << __func__ << ": selfPosition - (" << self.position[0] << ", " << self.position[1] << ")" << std::endl;
//std::cerr << __func__ << ": selfheading - (" << self.heading[0] << ", " << self.heading[1] << ")" << std::endl;
//std::cerr << __func__ << ": target - (" << target[0] << ", " << target[1] << ")" << std::endl;
//std::cerr << __func__ << ": direction - (" << direction[0] << ", " << direction[1] << ")" << std::endl;
//std::cerr << __func__ << ": selfHeading - " << selfHeading << std::endl;
//std::cerr << __func__ << ": targetHeading - " << targetHeading << std::endl;
//std::cerr << __func__ << ": targetBearing - " << targetBearing << std::endl;
//std::cerr << __func__ << ": targetDistance - " << result[0] << std::endl;
//std::cerr << __func__ << ": angleToTarget - " << result[1] << std::endl;
//std::cerr << __func__ << ": angleToCenter - " << result[2] << std::endl;

                //apply turning hysteresis
                return result;
            }

            arma::vec WalkPathPlanner::avoidObstacles(const std::vector<VisionObstacle>&,
                                                  const arma::vec3& movePlan) {
                return movePlan; //XXX:unimplemented
                //double heading = 0.0;
                /*float new_bearing = relative_bearing;
                float avoid_distance = min(m_avoid_distance,distance);
                vector<Object> obstacles;



                //use either localised or visual avoidance
                if (m_use_localisation_avoidance) {
                    //XXX: localisation based avoidance not implemented


                } else {
                    obstacles = NavigationLogic::getVisibleObstacles();
                    for(unsigned int i=0; i < obstacles.size(); i++) { //for each object
                        if (obstacles[i].measuredDistance() < avoid_distance) { //if we are an obstacle
                            if (obstacles[i].measuredBearing() > relative_bearing and obstacles[i].measuredBearing()-obstacles[i].arc_width < relative_bearing) {
                                //if we are on the right and occluding
                                new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()-obstacles[i].arc_width);
                            } else if (obstacles[i].measuredBearing() < relative_bearing and obstacles[i].measuredBearing()+obstacles[i].arc_width > relative_bearing) {
                                //if we are on the left and occluding
                                new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()+obstacles[i].arc_width);
                            }
                        }
                    }
                }*/
            }



        }  // planning
    }  // behaviours
}  // modules
