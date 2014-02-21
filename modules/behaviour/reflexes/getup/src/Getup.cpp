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

namespace modules {
    namespace behaviour {
        namespace reflexes {
            
            Getup::Getup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                //do a little configurating
                on<Trigger<Configuration<SensorFilter>>>([this](const Configuration<SensorFilter>& file){
                        FALLEN_ANGLE = cos(file.config["FALLEN_ANGLE"]*M_PI/180.0);
                        GETUP_PRIORITY = file.config["GETUP_PRIORITY"];
                        EXECUTION_PRIORITY = file.config["EXECUTION_PRIORITY"];
                    });
                });

            Getup::Getup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                
                on<Trigger<Sensors>, Options<Single>>([this](const Sensors& sensors) {
                    
                    // if we think we have fallen, get up
                    if (abs(sensors.orientation(2,2)) < FALLEN_ANGLE) {
                        
                        //check if we're on our front or back
                        if (sensors.orientation(0,2) > 0.0) {
                            //XXX: replace with priorities
                            emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpFront.json"));
                        } else {
                            emit(std::make_unique<messages::motion::ExecuteScriptByName>("StandUpBack.json"));
                        }
                    }
                    
                });
            }
            
        }  // tools
    }  // behaviours
}  // modules
