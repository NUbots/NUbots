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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "ScriptEngine.hpp"

#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"

namespace module::motion {

    using extension::ExecuteScript;
    using extension::ExecuteScriptByName;
    using extension::Script;

    using message::behaviour::ServoCommands;


    ScriptEngine::ScriptEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Script>("").then([this](const Script& script) {
            // Add this script to our list of scripts
            try {
                scripts.insert(std::pair(pathSplit(script.fileName).second, script));
            }
            catch (const std::exception& e) {
                log<NUClear::ERROR>("Script is bad conversion:", script.fileName, e.what());
            }
        });

        on<Trigger<ExecuteScriptByName>>().then([this](const ExecuteScriptByName& command) {
            std::vector<Script> scriptList;

            for (const auto& scriptName : command.scripts) {
                const auto& script = scripts.find(scriptName);

                if (script == std::end(scripts)) {
                    throw std::runtime_error("The script " + scriptName + " is not loaded in the system");
                }
                scriptList.push_back(script->second);
            }
            emit<Scope::DIRECT>(std::make_unique<ExecuteScript>(command.sourceId,
                                                                scriptList,
                                                                command.duration_modifier,
                                                                command.start));
        });

        on<Trigger<ExecuteScript>>().then([this](const ExecuteScript& command) {
            auto waypoints = std::make_unique<ServoCommands>();

            auto time = command.start;
            for (size_t i = 0; i < command.scripts.size(); i++) {
                const auto& script = command.scripts[i];

                for (const auto& frame : script.frames) {
                    // Move along our duration in time
                    time += std::chrono::duration_cast<NUClear::clock::time_point::duration>(
                        frame.duration * command.duration_modifier[i]);

                    // Loop through all the motors and make a servo waypoint for it
                    for (const auto& target : frame.targets) {
                        waypoints->commands.emplace_back(command.sourceId,
                                                         time,
                                                         target.id,
                                                         target.position,
                                                         target.gain,
                                                         target.torque);
                    }
                }
            }

            // Emit our waypoints
            emit(std::move(waypoints));
        });
    }

    std::pair<std::string, std::string> ScriptEngine::path_split(const std::string& input) {

        size_t lastSlash = input.rfind('/');

        // There was no slash
        if (lastSlash == std::string::npos) {
            return {".", input};
        }
        // The slash was the last character
        if (lastSlash + 1 == input.size()) {
            // If all we had was a slash
            if (input.size() == 1) {
                return {"/", "/"};
            }
            // Otherwise remove the slash and call recursively
            return path_split(input.substr(0, input.size() - 1));
        }
        // Else, the slash was not the last character
        return {input.substr(0, lastSlash), input.substr(lastSlash + 1, input.size())};
    }

}  // namespace module::motion
