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

#include "messages/Script.h"
#include "messages/ServoWaypoint.h"

namespace modules {
    void ScriptRunner::executeNextScript() {
        if(!scripts.empty()) {
            auto script = scripts.front();
            emit(std::make_unique<messages::ExecuteScriptByName>(script));
            scripts.pop();
        }
        else {
            powerPlant->shutdown();
        }
    }

    ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Trigger<CommandLineArguments>>([this](const std::vector<std::string>& args) {
            std::cout << "Args: " << args.size() << std::endl;
            for(size_t i = 1; i < args.size(); ++i) {
                std::cout << "Found script to run: " << args[i] << std::endl;
                scripts.push(args[i]);
            }

            executeNextScript();
        });

        on<Trigger<messages::AllServoWaypointsComplete>>([this](const messages::AllServoWaypointsComplete& complete) {
            executeNextScript();
        });
    }
}
