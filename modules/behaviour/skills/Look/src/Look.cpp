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

namespace modules {
    namespace behaviour {
        namespace skills {

            using messages::input::ServoID;
            using messages::input::Sensors;
            using messages::behaviour::LookAtAngle;
            using messages::behaviour::LookAtPosition;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::LimbID;
            using messages::support::Configuration;
            using messages::behaviour::ServoCommand;

            //internal only callback messages to start and stop our action
            struct ExecuteLook {};

            Look::Look(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<Look>>>([this] (const Configuration<Look>& file){

                    lastPanEnd = NUClear::clock::now();
                    //load fast and slow panspeed settings

                    //pan speeds
                    fastSpeed = file.config["FastSpeed"].as<double>();
                    slowSpeed = file.config["SlowSpeed"].as<double>();

                    //head limits
                    minYaw = file.config["minYaw"].as<double>();
                    maxYaw = file.config["maxYaw"].as<double>();
                    minPitch = file.config["minPitch"].as<double>();
                    maxPitch = file.config["maxPitch"].as<double>();
                    screenPadding = file.config["screenPadding"].as<double>();
                });

                on<Trigger<ExecuteLook>>([this] (const ExecuteLook& e) {
                    //we are active!

                });

                //look at a single visible object using the angular offsets in the image
                on<Trigger<LookAtAngle>, With<Sensors>>([this] (const LookAtAngle& look, const Sensors& sensors) {
                    lastPanEnd = NUClear::clock::now();
                    //speeds should take into account the angle delta
                    double distance = sqrt(look.pitch*look.pitch+look.yaw*look.yaw);
                    if (distance < 0.15) { //XXX: configurate
                        panTime = distance/slowSpeed;
                    } else {
                        panTime = distance/fastSpeed;
                    }
                    headYaw = std::fmin(std::fmax(look.yaw+sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition,minYaw),maxYaw);
                    headPitch = std::fmin(std::fmax(look.pitch+sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition,minPitch),maxPitch);


                    //this might find a better location eventually - it is the generic "gotopoint" code
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  30.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_YAW,     float(std::fmin(std::fmax(headYaw,minYaw),maxYaw)),  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    float(std::fmin(std::fmax(headPitch,minPitch),maxPitch)), 30.f});
                    emit(std::move(waypoints));
                });

                //look at multiple visible objects using the angular offsets in the image
                on<Trigger<std::vector<LookAtAngle>>, With<Sensors>>([this] (const std::vector<LookAtAngle>& look, const Sensors& sensors) {
                    lastPanEnd = NUClear::clock::now();
                    double pitchLow = 0.0,
                           pitchHigh = 0.0,
                           yawLeft = 0.0,
                           yawRight = 0.0;

                    //XXX: config this; it is the space between the edge of the screen and the object
                    double offset = 0.15;

                    //loop through and get the yaw/pitch bounds
                    for (const auto& l : look) {
                        pitchLow = fmin(pitchLow,l.pitch-offset);
                        pitchHigh = fmax(pitchHigh,l.pitch+offset);

                        //XXX: we need to add object width metadata
                        yawLeft = fmin(yawLeft,l.yaw-offset);
                        yawRight = fmax(yawRight,l.yaw+offset);
                        offset = 0.0;
                    }

                    double pitch = (pitchLow+pitchHigh)/2.0;
                    double yaw = (yawLeft+yawRight)/2.0;
                    //std::cout << pitch << ", " << yaw << ", " << look.size() << std::endl;
                    //smoothing to reduce jerky movement
                    //pitch = prevPitch = (pitch+prevPitch)/2.0;
                    //yaw = prevYaw = (yaw+prevYaw)/2.0;

                    //speeds should take into account the angle delta
                    double distance = sqrt(pitch*pitch+yaw*yaw);
                    if (distance < 0.15) { //XXX: configurate
                        panTime = distance/slowSpeed;
                    } else {
                        panTime = distance/fastSpeed;
                    }

                    headYaw = std::fmin(std::fmax(yaw+sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition,minYaw),maxYaw);
                    headPitch = std::fmin(std::fmax(pitch+sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition,minPitch),maxPitch);


                    //this might find a better location eventually - it is the generic "gotopoint" code
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  30.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 30.f});

                    waypoints->push_back({id, time, ServoID::HEAD_YAW,     float(std::fmin(std::fmax(headYaw,minYaw),maxYaw)),  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    float(std::fmin(std::fmax(headPitch,minPitch),maxPitch)), 30.f});
                    emit(std::move(waypoints));
                });


                //look at multiple visible objects using the angular offsets in the image
                on<Trigger<std::vector<LookAtPosition>>, With<Sensors>>([this] (const std::vector<LookAtPosition>& look, const Sensors& sensors) {

                    if (NUClear::clock::now() >= lastPanEnd) {
                        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                        waypoints->reserve(2+2*look.size());
                        waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  30.f});
                        waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 30.f});

                        //this might find a better location eventually - it is the generic "gotopoint" code
                        time_t time = NUClear::clock::now();
                        for (size_t i = 0; i < look.size(); ++i) {
                            double pitch = look[i].pitch;
                            double yaw = look[i].yaw;
                            //speeds should take into account the angle delta
                            double distance = sqrt(pitch*pitch+yaw*yaw);
                            panTime = distance/fastSpeed;
                            time += std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                            headYaw = std::fmin(std::fmax(yaw,minYaw),maxYaw);
                            headPitch = std::fmin(std::fmax(pitch,minPitch),maxPitch);
                            waypoints->push_back({id, time, ServoID::HEAD_YAW,     float(std::fmin(std::fmax(headYaw,minYaw),maxYaw)),  30.f});
                            waypoints->push_back({id, time, ServoID::HEAD_PITCH,    float(std::fmin(std::fmax(headPitch,minPitch),maxPitch)), 30.f});

                        }
                        lastPanEnd = time;
                        emit(std::move(waypoints));
                    }
                });

                /*
                on<Trigger<LookAtPoint>, With<Sensors>>([this] (const LookAtPoint& look, const Sensors& sensors) {
                    //speeds should take into account the angle delta
                    double distance = look.pitch*look.pitch+look.yaw*look.yaw;
                    panTime = distance/fastSpeed;
                    headYaw = std::fmin(std::fmax(look.yaw+sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition,minYaw),maxYaw);
                    headPitch = std::fmin(std::fmax(look.pitch+sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition,minPitch),maxPitch);


                    //this might find a better location eventually - it is the generic "gotopoint" code
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  30.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_YAW,     float(std::fmin(std::fmax(headYaw,minYaw),maxYaw)),  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    float(std::fmin(std::fmax(headPitch,minPitch),maxPitch)), 30.f});
                    emit(std::move(waypoints));
                });

                on<Trigger<std::vector<LookAtPoint>>, With<Sensors>>([this] (const std::vector<LookAtPoint>& look, const Sensors& sensors) {
                    //speeds should take into account the angle delta
                    double distance = look.pitch*look.pitch+look.yaw*look.yaw;
                    panTime = distance/fastSpeed;
                    headYaw = std::fmin(std::fmax(look.yaw+sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition,minYaw),maxYaw);
                    headPitch = std::fmin(std::fmax(look.pitch+sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition,minPitch),maxPitch);


                    //this might find a better location eventually - it is the generic "gotopoint" code
                    time_t time = NUClear::clock::now() + std::chrono::nanoseconds(size_t(std::nano::den*panTime));
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(4);
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_YAW,     float(sensors.servos[size_t(ServoID::HEAD_YAW)].presentPosition),  30.f});
                    waypoints->push_back({id, NUClear::clock::now(), ServoID::HEAD_PITCH,    float(sensors.servos[size_t(ServoID::HEAD_PITCH)].presentPosition), 30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_YAW,     float(std::fmin(std::fmax(headYaw,minYaw),maxYaw)),  30.f});
                    waypoints->push_back({id, time, ServoID::HEAD_PITCH,    float(std::fmin(std::fmax(headPitch,minPitch),maxPitch)), 30.f});
                    emit(std::move(waypoints));
                });*/
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
