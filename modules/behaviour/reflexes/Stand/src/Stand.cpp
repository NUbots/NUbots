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

#include "Stand.h"

#include <cmath>
#include "messages/motion/Script.h"
#include "messages/motion/ServoWaypoint.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace behaviour {
        namespace reflexes {

            //internal only callback messages to start and stop our action
            struct ExecuteStand{ std::vector<LimbID> limbs; };
            struct KillStand{ std::vector<LimbID> limbs; };

            using messages::support::Configuration;
            using messages::motion::AllServoWaypointsComplete;

            Stand::Stand(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                    //do a little configurating
                    on<Trigger<Configuration<Stand>>>([this](const Configuration<Getup>& file){

                        //we don't have any config data yet
                        
                    });

                //Execute the kick action
                on<Trigger<ExecuteStand>, Options<Sync<Stand>>>([this](const ExecuteGetup& a) {
                    
                });

                on<Trigger<KillStand>, Options<Sync<Stand>>>([this](const KillGetup& a) {
                    
                });


                //register our callbacks with the controller
                emit(std::make_unique<messages::behaviour::RegisterAction>(

                    size_t(this)*size_t(this)-size_t(this), //unique identifier based on this

                    //Initial limb priorities
                    {{1,{LimbID::LEFT_LEG,LimbID::RIGHT_LEG}},
                     {0.5,{LimbID::LEFT_ARM,LimbID::RIGHT_ARM}}
                         },

                    [this](const std::vector<LimbID> limbs) { //make a function to start the action
                        emit(std::make_unique<ExecuteStand>(limbs));
                    }),

                    [this](const std::vector<LimbID> limbs) { //make a function to end the action
                        emit(std::make_unique<KillStand>(limbs));
                    }));
            }
        }  // tools
    }  // behaviours
}  // modules
