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
#include "messages/motion/Script.h"
#include "messages/research/scriptoptimizer/OptimizeScript.pb.h"
#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
    namespace research {
        
        using messages::platform::darwin::DarwinSensors;
        using messages::research::scriptoptimizer::OptimizeScript;
        using messages::motion::ExecuteScript;
        using messages::motion::Script;
        
        ScriptOptimizer::ScriptOptimizer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Network<OptimizeScript>>, With<NUClear::extensions::NetworkingConfiguration>>([this]
                    (const Network<OptimizeScript>& task, const NUClear::extensions::NetworkingConfiguration config) {

                // Check if this script is for us
                if (config.deviceName == task.data->target()) {
                    
                    log<NUClear::DEBUG>("Script ", task.data->iteration(), " was delivered to be executed");

                    auto script = std::make_unique<ExecuteScript>();
                    
                    // Make a script from the frames
                    for (const auto& frame : task.data->frames()) {
                        Script::Frame f;
                        for (const auto& target : frame.targets()) {
                            Script::Frame::Target t;
                            
                            t.id = messages::DarwinSensors::Servo::idFromString(target.id());
                            t.position = target.position();
                            t.gain = target.gain();
                            
                            f.targets.push_back(std::move(t));
                        }
                        
                        script->script.frames.push_back(std::move(f));
                    }
                    
                    // Emit our script
                    emit(std::move(script));
                    
                    // Start recording
                    this->recording = true;
                }
            });
            
            on<Trigger<Raw<messages::DarwinSensors>>>([this](const std::shared_ptr<const messages::DarwinSensors>& frame) {
                
                // While we are recording, store all the frames in a vector
                if(this->recording) {
                    sensors->push_back(frame);
                }
            });
            
            on<Trigger<messages::AllServoWaypointsComplete>>([this](const messages::AllServoWaypointsComplete&) {
                
                // If we were recording
                if(this->recording) {
                    
                    // Stop recording
                    this->recording = false;
                    
                    // Make a response message to the optimizer
                    auto result = std::make_unique<messages::OptimizeScriptResult>();
                    
                    // Store all the sensor values for the script
                    for(const auto& sensor : sensors) {
                        // Add it to the list
                    }
                    
                    // Store our metadata about what we executed
                    
                    // Return our result to the optimizer
                    emit<Scope::NETWORK>(std::move(result));
                }
            });
            
        }
    }
}
