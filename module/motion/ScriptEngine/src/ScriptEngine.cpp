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

#include "ScriptEngine.h"

#include <fmt/format.h>

#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"

#include "utility/file/fileutil.h"

namespace module {
namespace motion {

    using message::behaviour::ServoCommand;
    using message::extension::ExecuteScript;
    using message::extension::ExecuteScriptByName;


    ScriptEngine::ScriptEngine(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), scripts() {

        on<::extension::Script>("").then([this](const ::extension::Script& script) {
            // Add this script to our list of scripts
            try {
                scripts.insert(
                    std::make_pair(utility::file::pathSplit(script.filename).second, std::move(script.script)));
            }
            catch (const std::exception& e) {
                log<NUClear::ERROR>("Script is bad conversion:", script.filename, e.what());
            }
        });

        on<Trigger<ExecuteScriptByName>>().then([this](const ExecuteScriptByName& command) {
            std::vector<message::extension::Script> script_list;

            for (size_t i = 0; i < command.scripts.size(); i++) {
                const std::string& script_name = command.scripts[i];
                auto script                    = scripts.find(script_name);

                if (script == std::end(scripts)) {
                    throw std::runtime_error(fmt::format("The script {} is not loaded in the system", script_name));
                }
                else {
                    script_list.push_back(script->second);
                }
            }
            emit<Scope::DIRECT>(std::make_unique<ExecuteScript>(
                command.source_id, script_list, command.duration_modifier, command.start));
        });

        on<Trigger<ExecuteScript>>().then([this](const ExecuteScript& command) {
            auto waypoints = std::make_unique<std::vector<ServoCommand>>();

            auto time = command.start;
            for (size_t i = 0; i < command.scripts.size(); i++) {
                const auto& script = command.scripts[i];

                for (const auto& frame : script.frames) {
                    // Move along our duration in time
                    time += std::chrono::duration_cast<NUClear::clock::time_point::duration>(
                        frame.duration * command.duration_modifier[i]);

                    // Loop through all the motors and make a servo waypoint for it
                    for (const auto& target : frame.targets) {
                        waypoints->push_back(
                            {command.source_id, time, target.id, target.position, target.gain, target.torque});
                    }
                }
            }

            // Emit our waypoints
            emit(std::move(waypoints));
        });
    }

}  // namespace motion
}  // namespace module
