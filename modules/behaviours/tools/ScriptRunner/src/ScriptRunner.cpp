/*
 * This file is part of ScriptRunner.
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

#include "ScriptRunner.h"

#include "messages/motion/Script.h"
#include "messages/motion/ServoWaypoint.h"

namespace modules {
    namespace behaviours {
        namespace tools {
            
            void ScriptRunner::executeNextScript() {
                
                // If we have a script to execute
                if(!scripts.empty()) {
                    
                    // Get it and emit it
                    auto script = scripts.front();
                    emit(std::make_unique<messages::motion::ExecuteScriptByName>(script));
                    scripts.pop();
                }
                // Otherwise we are done, shutdown
                else {
                    powerPlant->shutdown();
                }
            }

            ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                
                // Get the scripts to run from the command line
                on<Trigger<CommandLineArguments>>([this](const std::vector<std::string>& args) {
                    NUClear::log<NUClear::INFO>("Executing: ", args.size(), " scripts");
                    
                    for(size_t i = 1; i < args.size(); ++i) {
                        NUClear::log<NUClear::INFO>("Queueing script ", args[i]);
                        scripts.push(args[i]);
                    }

                    executeNextScript();
                });

                // When we finish with a script, execute the next one
                on<Trigger<messages::motion::AllServoWaypointsComplete>>([this](const messages::motion::AllServoWaypointsComplete& complete) {
                    executeNextScript();
                });
            }
            
        }  // tools
    }  // behaviours
}  // modules
