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

#include "ScriptEngine.h"

#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"

namespace modules {
    namespace motion {

        using messages::support::Configuration;
        using messages::behaviour::ServoCommand;
        using messages::motion::Script;
        using messages::motion::ExecuteScriptByName;
        using messages::motion::ExecuteScript;

        struct Scripts {
            // For scripts we want updates on the whole scripts directory
            static constexpr const char* CONFIGURATION_PATH = "scripts/";
        };

        ScriptEngine::ScriptEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Configuration<Scripts>>>([this](const Configuration<Scripts>& script) {
                // Add this script to our list of scripts
                scripts.insert(std::make_pair(script.name, script.config.as<Script>()));
            });

            on<Trigger<ExecuteScriptByName>>([this](const ExecuteScriptByName& command) {
                std::vector<Script> scriptList;
                for(const auto& scriptName : command.scripts) {
                    auto script = scripts.find(scriptName);

                    if(script == std::end(scripts)) {
                        throw std::runtime_error("The script " + scriptName + " is not loaded in the system");
                    }
                    else {
                        scriptList.push_back(script->second);
                    }
                }
                emit<Scope::DIRECT>(std::make_unique<ExecuteScript>(command.sourceId, scriptList, command.start));
            });

            on<Trigger<ExecuteScript>>([this](const ExecuteScript& command) {

                auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                auto time = command.start;
                for(const auto& script : command.scripts){
                    for(const auto& frame : script.frames) {
                        // Move along our duration in time
                        time += frame.duration;

                        // Loop through all the motors and make a servo waypoint for it
                        for(const auto& target : frame.targets) {
                            waypoints->push_back({
                                command.sourceId,
                                time,
                                target.id,
                                target.position,
                                target.gain,
                                target.torque
                            });
                        }
                    }
                }

                // Emit our waypoints
                emit(std::move(waypoints));
            });
        }

    }  // motion
}  // modules
