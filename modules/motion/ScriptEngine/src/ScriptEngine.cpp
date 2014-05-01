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

        struct Scripts {
            // For scripts we want updates on the whole scripts directory
            static constexpr const char* CONFIGURATION_PATH = "scripts/";
        };

        ScriptEngine::ScriptEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<messages::support::Configuration<Scripts>>>([this](const messages::support::Configuration<Scripts>& script) {
                // Add this script to our list of scripts
                scripts.insert(std::make_pair(script.name, script.config));
            });

            on<Trigger<messages::motion::ExecuteScriptByName>>([this](const messages::motion::ExecuteScriptByName& command) {
                NUClear::clock::duration offset(0);
                for(const auto& scriptName : command.scripts) {
                    auto script = scripts.find(scriptName);

                    if(script == std::end(scripts)) {
                        throw std::runtime_error("The script " + scriptName + " is not loaded in the system");
                    }
                    else {
                        emit<Scope::DIRECT>(std::make_unique<messages::motion::ExecuteScript>(command.sourceId, script->second, command.start + offset));

                        for(const auto& f : script->second.frames) {
                            offset += f.duration;
                        }
                    }
                }
            });

            on<Trigger<messages::motion::ExecuteScript>>([this](const messages::motion::ExecuteScript& command) {

                auto waypoints = std::make_unique<std::vector<messages::behaviour::ServoCommand>>();

                auto time = command.start;
                for(const auto& frame : command.script.frames) {
                    // Move along our duration in time
                    time += frame.duration;

                    // Loop through all the motors and make a servo waypoint for it
                    for(const auto& target : frame.targets) {
                        waypoints->push_back({
                            command.sourceId,
                            time,
                            target.id,
                            target.position,
                            target.gain
                        });
                    }
                }

                // Emit our waypoints
                emit(std::move(waypoints));
            });
        }

    }  // motion
}  // modules
