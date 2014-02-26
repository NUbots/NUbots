/*
 * This file is part of Getup.
 *
 * ScriptRunner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptRunner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptRunner.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Getup.h"

#include <cmath>
#include "messages/motion/Script.h"
#include "messages/motion/ServoWaypoint.h"
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"

namespace modules {
    namespace behaviour {
        namespace reflexes {

            //internal only callback messages to start and stop our action
            struct ExecuteGetup{ std::vector<LimbID> limbs; };
            struct KillGetup{ std::vector<LimbID> limbs; };

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::motion::AllServoWaypointsComplete;

            Getup::Getup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), fallenDetector(nullptr), getupDetector(nullptr), currentPriority(0) {
                    //do a little configurating
                    on<Trigger<Configuration<Getup>>>([this](const Configuration<Getup>& file){

                        //encode fallen angle as a cosine so we can compare it directly to the z axis value
                        double fallenAngleConfig = file.config["FALLEN_ANGLE"];
                        FALLEN_ANGLE = cos(fallenAngleConfig*M_PI/180.0);

                        //load priorities for the getup
                        GETUP_PRIORITY = file.config["GETUP_PRIORITY"];
                        EXECUTION_PRIORITY = file.config["EXECUTION_PRIORITY"];
                    });

                fallenDetector = on<Trigger<Sensors>, Options<Single>>([this](const Sensors& sensors) {

                    //check if the orientation is smaller than the cosine of our fallen angle
                    if (currentPriority != GETUP_PRIORITY and fabs(sensors.orientation(2,2)) < FALLEN_ANGLE) {
                        updateAction(GETUP_PRIORITY);
                    } else  if (currentPriority > 0.0) {
                        updateAction(0.0);
                    }

                });

                getupDetector = on<Trigger<AllServoWaypointsComplete>, Options<Single>>([this](const AllServoWaypointsComplete&) {
                    //when everything is complete, then switch back to event listener mode and change pririties
                    updateAction(0.0);
                    fallenDetector.enable();
                    getupDetector.disable();
                });

                //Execute the kick action
                on<Trigger<ExecuteGetup>, With<Sensors>, Options<Sync<Getup>>>([this](const ExecuteGetup& a, const Sensors& sensors) {
                    fallenDetector.disable();
                    getupDetector.enable();
                    //check with side we're getting up from
                    if (sensors.orientation(0,2) < 0.0) {
                        emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpFront.json"));
                    } else {
                        emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpBack.json"));
                    }
                    updateAction(EXECUTION_PRIORITY);
                });

                on<Trigger<KillGetup>, Options<Sync<Getup>>>([this](const KillGetup& a) {
                    fallenDetector.enable();
                    getupDetector.disable();
                    updateAction(0);
                });


                //start with the kill switch turned off
                getupDetector.disable();

                //register our callbacks with the controller
                emit(std::make_unique<messages::behaviour::RegisterAction>(

                    size_t(this)*size_t(this)-size_t(this), //unique identifier based on this

                    //Initial limb priorities
                    {{0,{LimbID::HEAD,
                         LimbID::LEFT_ARM,LimbID::RIGHT_ARM,
                         LimbID::LEFT_LEG,LimbID::RIGHT_LEG}}},

                    [this](const std::vector<LimbID> limbs) { //make a function to start the action
                        emit(std::make_unique<ExecuteGetup>(limbs));
                    }),

                    [this](const std::vector<LimbID> limbs) { //make a function to end the action
                        emit(std::make_unique<KillGetup>(limbs));
                    }));



            }

            Getup::updateAction(const double& priority) {
                currentPriority = priority;
                emit(std::make_unique<messages::behaviour::UpdateAction>(priority));
            }



        }  // tools
    }  // behaviours
}  // modules
