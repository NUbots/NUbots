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

#include "LookAt.h"

#include "messages/input/ServoID.h"
#include "messages/behaviour/Look.h"
#include "messages/behaviour/Action.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/input/CameraParameters.h"

namespace modules {
    namespace behaviour {
        namespace skills {
            using messages::input::CameraParameters;
            using messages::input::ServoID;
            using messages::input::Sensors;
            using messages::behaviour::Look;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::LimbID;
            using messages::support::Configuration;
            using messages::behaviour::ServoCommand;

            //internal only callback messages to start and stop our action
            struct ExecuteLook {};

            LookAt::LookAt(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<LookAt>>>([this] (const Configuration<LookAt>& config){

                    //pan speeds
                    FAST_SPEED = config["speed"]["fast"].as<double>();
                    SLOW_SPEED = config["speed"]["slow"].as<double>();

                    HEAD_PITCH_MIN = config["limits"]["pitch"][0].as<double>();
                    HEAD_PITCH_MAX = config["limits"]["pitch"][1].as<double>();
                    HEAD_YAW_MAX   = config["limits"]["yaw"][0].as<double>();
                    HEAD_YAW_MIN   = config["limits"]["yaw"][1].as<double>();
                    SCREEN_EDGE_PADDING = config["screen_edge_padding"].as<double>();
                });


                on<Trigger<Every<30, Per<std::chrono::seconds>>>,
                   With<Last<5,Sensors>>,
                   Options<Sync<LookAt>>>([this](const time_t&,
                                                  const LastList<Sensors>& sensors) {

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;

                    // Find our most recent frame without errors
                    const auto& sensor = **std::find_if(sensors.rbegin(), sensors.rend(), [] (const std::shared_ptr<const Sensors>& a) {
                        return a->servos[uint(ServoID::HEAD_YAW)].errorFlags == 0 && a->servos[uint(ServoID::HEAD_PITCH)].errorFlags == 0;
                    });
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;


                    arma::fvec2 lastVelocity = arma::fvec2({sensor.servos[uint(ServoID::HEAD_YAW)].presentVelocity,
                                                          sensor.servos[uint(ServoID::HEAD_PITCH)].presentVelocity});
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    arma::fvec2 lastPosition = arma::fvec2({sensor.servos[uint(ServoID::HEAD_YAW)].presentPosition,
                                                          sensor.servos[uint(ServoID::HEAD_PITCH)].presentPosition});

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    //emit the servo positions
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    //push back some fake waypoints to clear our commands
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW, lastPosition[0], 0.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH, lastPosition[1], 0.f});

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    arma::fvec2 targetPoint;

                    //update appropriately for the current movement
                    if (currentPoints.size() == 1) {

                        //clip the head angles
                        const arma::vec2 targetPoint = arma::vec2({
                                                            std::fmin(std::fmax(currentPoints[0][0],HEAD_YAW_MIN),HEAD_YAW_MAX),
                                                            std::fmin(std::fmax(currentPoints[0][1],HEAD_PITCH_MIN),HEAD_PITCH_MAX)});
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;


                    } else if (currentPoints.size() > 0 and saccading) { //do saccades

                    } else if (currentPoints.size() > 0 and not saccading) { //do pans
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;

                        size_t currentSelection = currentPoints.size() - 1;
                        double currentGoodness = arma::norm(lastPosition-currentPoints.back()) +
                                                 arma::dot(lastVelocity,currentPoints.front()-currentPoints.back());
                        for (size_t i = 0; i < currentPoints.size()-1; ++i) {
                            double newGoodness = arma::norm(lastPosition-currentPoints[i]) +
                                                 arma::dot(lastPosition,currentPoints[i+1]-currentPoints[i]);
                            if (newGoodness < currentGoodness) {
                                currentGoodness = newGoodness;
                                currentSelection = i;
                            }
                        }

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                        targetPoint = (currentSelection + 1) % currentPoints.size();
                    }
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;

                    //get the approximate distance of movement
                    const double panDist = arma::norm(targetPoint - lastPosition) + std::numeric_limits<double>::epsilon();

                    //calculate how long the movement should take
                    double panTime = panDist/FAST_SPEED;
                    if (panDist < 0.15) { //XXX: configurate the slow to fast switch distance
                        panTime = panDist/SLOW_SPEED;
                    }
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(uint(std::nano::den * panTime));

                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    //push back the new points
                    waypoints->push_back({id, time, ServoID::HEAD_YAW,      targetPoint[0],  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    targetPoint[1], 30.f});
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                    for (auto& waypoint : *waypoints) {
                        std::cout << static_cast<uint>(waypoint.id) << ", " << waypoint.position << std::endl;
                    }


                    emit(std::move(waypoints));
                    std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
                });


                on<Trigger<std::vector<Look::Fixation>>,
                   With<CameraParameters>,
                   Options<Sync<LookAt>>>([this](const std::vector<Look::Fixation>& fixations,
                                                  const CameraParameters& cameraParams) {

                    //start with the most permissive settings possible and add items incrementally
                    arma::vec2 angleMin = fixations[0].angle-cameraParams.FOV + SCREEN_EDGE_PADDING;
                    arma::vec2 angleMax = fixations[0].angle+cameraParams.FOV - SCREEN_EDGE_PADDING;

                    for (size_t i = 1; i < fixations.size(); ++i) {
                        if (fixations[i].angle[0] > angleMin[0] and
                            fixations[i].angle[1] > angleMin[1] and
                            fixations[i].angle[0] < angleMax[0] and
                            fixations[i].angle[1] < angleMax[1]) { //if this item is in the permissible range

                            const arma::vec2 minVisible = fixations[i].angle-cameraParams.FOV;
                            const arma::vec2 maxVisible = fixations[i].angle+cameraParams.FOV;

                            angleMin = arma::vec2({std::fmax(minVisible[0], angleMin[0]), std::fmax(minVisible[1], angleMin[1])});
                            angleMax = arma::vec2({std::fmin(minVisible[0], angleMax[0]), std::fmin(minVisible[1], angleMax[1])});

                        }
                    }

                    //get the centre of the current focus
                    currentPoints.clear();
                    currentPoints.push_back((angleMin+angleMax) * 0.5);

                });

                on<Trigger<std::vector<Look::Pan>>, Options<Sync<LookAt>>>([this](const std::vector<Look::Pan>& pan) {
                    //copy the pan into the currentpoints
                    //XXX: actually we can simplify this a lot later on using angular sizes
                    saccading = false;
                    currentPoints.clear();
                    for (size_t i = 0; i < pan.size(); ++i) {
                        currentPoints.push_back(pan[i].angle);
                        std::cout << __FILE__ << "," << __LINE__ << ", " << pan[i].angle << std::endl;
                    }
                });

                on<Trigger<std::vector<Look::Saccade>>, Options<Sync<LookAt>>>([this](const std::vector<Look::Saccade>& saccade) {
                    (void)saccade;
                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "Look",
                    { std::pair<float, std::set<LimbID>>(30.0, { LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteLook>());
                    },
                    [this] (const std::set<LimbID>&) { },
                    [this] (const std::set<ServoID>&) { }
                }));
            }
        }  // reflexes
    }  // behaviours
}  // modules
