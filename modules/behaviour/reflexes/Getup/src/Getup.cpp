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

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::motion::AllServoWaypointsComplete;

            Getup::Getup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), fallenDetector(nullptr), getupDetector(nullptr) {
                    //do a little configurating
                    on<Trigger<Configuration<Getup>>>([this](const Configuration<Getup>& file){
                        double fallenAngleConfig = file.config["FALLEN_ANGLE"];

                        FALLEN_ANGLE = cos(fallenAngleConfig*M_PI/180.0);
                        GETUP_PRIORITY = file.config["GETUP_PRIORITY"];
                        EXECUTION_PRIORITY = file.config["EXECUTION_PRIORITY"];
                    });

                fallenDetector = on<Trigger<Sensors>, Options<Single>>([this](const Sensors& sensors) {
                    // if we think we have fallen, get up
                    //NUClear::log(fabs(sensors.orientation(2,2)),FALLEN_ANGLE);

                    if (fabs(sensors.orientation(2,2)) < FALLEN_ANGLE) {
                        fallenDetector.disable();
                        getupDetector.enable();
                        //check if we're on our front or back
                        if (sensors.orientation(0,2) < 0.0) {
                            //XXX: replace with priorities
                            emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpFront.json"));
                        } else {
                            emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpBack.json"));
                        }
                    }

                });

                getupDetector = on<Trigger<AllServoWaypointsComplete>>([this](const AllServoWaypointsComplete&) {
                    getupDetector.disable();
                    fallenDetector.enable();
                });

                getupDetector.disable();
            }

        }  // tools
    }  // behaviours
}  // modules
