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

#include "Look.h"

#include "messages/input/ServoID.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/Action.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/input/CameraParameters.h"

namespace modules {
    namespace behaviour {
        namespace skills {
            using messages::input::CameraParameters
            using messages::input::ServoID;
            using messages::input::Sensors;
            using messages::behaviour::Look;
            using messages::behaviour::LimbID;
            using messages::support::Configuration;
            using messages::behaviour::ServoCommand;

            //internal only callback messages to start and stop our action
            struct ExecuteLook {};

            LookAt::LookAt(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<Look>>>([this] (const Configuration<Look>& config){

                    lastPanEnd = NUClear::clock::now();
                    //load fast and slow panspeed settings

                    //pan speeds
                    FAST_SPEED = config["speed"]["fast"].as<double>();
                    SLOW_SPEED = config["speed"]["slow"].as<double>();

                    HEAD_PITCH_MIN = config["limits"]["pitch"][0].as<double>();
                    HEAD_PITCH_MAX = config["limits"]["pitch"][1].as<double>();
                    HEAD_YAW_MAX   = config["limits"]["yaw"][0].as<double>();
                    HEAD_YAW_MIN   = config["limits"]["yaw"][1].as<double>();
                    SCREEN_EDGE_PADDING = config["screen_edge_padding"].as<double>();
                });

                on<Trigger<std::vector<Look::Fixation>>,
                   With<Sensors>,
                   With<CameraParameters>>([this](const std::vector<Look::Fixation>& fixations,
                                                  const Sensors& sensors,
                                                  const CameraParameters& cameraParams) {
                    
                    //start with the most permissive settings possible and add items incrementally
                    arma::vec2 angleMin = fixations[0].angle-cameraParams.FOV + SCREEN_EDGE_PADDING;
                    arma::vec2 angleMax = fixations[0].angle+cameraParams.FOV - SCREEN_EDGE_PADDING;
                    
                    for (size_t i = 1; i < fixations.size(); ++i) {
                        if (fixations[i].angle[0] > angleMin[0] and
                            fixations[i].angle[1] > angleMin[1] and
                            fixations[i].angle[0] < angleMax[0] and
                            fixations[i].angle[1] < angleMax[1] and) { //if this item is in the permissible range
                            
                            const arma::vec2 minVisible = fixations[i].angle-cameraParams.FOV;
                            const arma::vec2 maxVisible = fixations[i].angle+cameraParams.FOV;
                            
                            angleMin = arma::vec2({std::fmax(minVisible[0], angleMin[0]), std::fmax(minVisible[1], angleMin[1])});
                            angleMax = arma::vec2({std::fmin(minVisible[0], angleMax[0]), std::fmin(minVisible[1], angleMax[1])});
                        
                        }
                    }
                    
                    //get the centre of the current focus
                    currentPoint = (angleMin+angleMax)*0.5;
                    
                    
                    //HEAD MOVEMENT EMIT - MAY BE MOVED TO ANOTHER TRIGGER FOR IMU SPACE TRACKING -----------------------------------------------------------------
                    
                    //clip the head angles
                    const arma::vec2 headPosition = arma::vec2({
                                                        std::fmin(std::fmax(currentPoint[0],HEAD_YAW_MIN),HEAD_YAW_MAX),
                                                        std::fmin(std::fmax(currentPoint[1],HEAD_PITCH_MIN),HEAD_PITCH_MAX)});
                    
                    
                    //get the approximate distance of movement
                    const double panDist = arma::norm(currentPoint - 
                                            arma::vec2({sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition,
                                                        sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition}));
                    
                    //calculate how long the movement should take
                    double panTime = panDist/FAST_SPEED;
                    if (panDist < 0.15) { //XXX: configurate the slow to fast switch distance
                        panTime = panDist/SLOW_SPEED;
                    }
                    
                    //emit the servo positions
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);
                    
                    //push back some fake waypoints to clear our commands
                    //XXX: what if the sensor data is broken?
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  0.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 0.f});
                    
                    //push back the new points
                    waypoints->push_back({id, time, ServoID::HEAD_YAW,     headPosition[0],  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    headPosition[1], 30.f});
                    emit(std::move(waypoints));
                    
                });

                on<Trigger<std::vector<Look::Pan>>>([this](const std::vector<Look::Pan>& pan) {

                });

                on<Trigger<std::vector<Look::Saccade>>>([this](const std::vector<Look::Saccade>& saccade) {

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
