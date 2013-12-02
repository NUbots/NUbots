/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ScriptEngine.h"

#include "messages/Configuration.h"
#include "messages/ServoWaypoint.h"
#include "messages/DarwinServoCommand.h"

namespace modules {
    struct Scripts {
        // For scripts we want updates on the whole scripts directory
        static constexpr const char* CONFIGURATION_PATH = "scripts/";
    };

    ScriptEngine::ScriptEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<messages::Configuration<Scripts>>>([this](const messages::Configuration<Scripts>& script) {
            // Add this script to our list of scripts
            scripts.insert(std::make_pair(script.name, script.config));
        });

        on<Trigger<messages::ExecuteScriptByName>>([this](const messages::ExecuteScriptByName& command) {
            auto script = scripts.find(command.script);

            if(script == std::end(scripts)) {
                throw std::runtime_error("The script " + command.script + " is not loaded in the system");
            }
            else {
                emit(std::make_unique<messages::ExecuteScript>(script->second, command.start));
            }
        });

        on<Trigger<messages::ExecuteScript>>([this](const messages::ExecuteScript& command) {

            auto waypoints = std::make_unique<std::vector<messages::ServoWaypoint>>();

            auto time = command.start;
            for(const auto& frame : command.script.frames) {
                // Move along our duration in time
                time += frame.duration;

                // Loop through all the motors and make a servo waypoint for it
                for(const auto& target : frame.targets) {
                    waypoints->push_back({
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
}
