/*
 * This file is part of ScriptOptimizer.
 *
 * ScriptOptimizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptOptimizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptOptimizer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ScriptOptimizer.h"
#include "messages/Script.h"
#include "messages/OptimizeScript.pb.h"
#include "messages/DarwinSensors.h"

namespace modules {
    namespace Research {
        ScriptOptimizer::ScriptOptimizer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Network<messages::OptimizeScript>>, With<NUClear::extensions::NetworkingConfiguration>>([this]
                    (const Network<messages::OptimizeScript>& task, const NUClear::extensions::NetworkingConfiguration config) {

                // Check if this script is for us
                if (config.deviceName == task.data->target()) {
                    
                    log<NUClear::DEBUG>("Script ", task.data->iteration(), " was delivered to be executed");

                    auto script = std::make_unique<messages::Script>();
                    
                    // Make a script from the frames
                    for (const auto& frame : task.data->frames()) {
                        messages::Script::Frame f;
                        for (const auto& target : frame.targets()) {
                            messages::Script::Frame::Target t;
                            //t.id;
                            t.position = target.position();
                            t.gain = target.gain();
                            
                            f.targets.push_back(std::move(t));
                        }
                        
                        script->frames.push_back(std::move(f));
                    }
                }
                
                // Emit the frame
            });
        }
    }
}
