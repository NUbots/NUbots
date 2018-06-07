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

#include "ScriptRunner.h"

#include "message/platform/darwin/DarwinSensors.h"

#include "extension/Configuration.h"
#include "extension/Script.h"


#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"


namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;

        using extension::ExecuteScriptByName;

        using message::platform::darwin::DarwinSensors;
        using NUClear::message::CommandLineArguments;

        using message::platform::darwin::ButtonMiddleDown;

        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        struct ExecuteNextScript {};

        void ScriptRunner::executeNextScript() {

            // If we have a script to execute
            if (!scripts.empty()) {
                // Get it and emit it
                emit(std::make_unique<ExecuteScriptByName>(id, scripts));
            }
            // Otherwise we are done, shutdown
            else {
            }
        }

        ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , sensorHandle()
            , scripts()
            , id(size_t(this) * size_t(this) - size_t(this)) {

            // Get the scripts to run from the command line
            on<Configuration, With<CommandLineArguments>>("ScriptRunner.yaml")
                .then([this](const Configuration& config, const CommandLineArguments& args) {
                    script_delay = config["script_delay"].as<uint>();

                    // Check for scripts entered in the command line
                    if (args.size() > 1) {
                        NUClear::log<NUClear::INFO>("Executing: ", args.size() - 1, " scripts");
                        std::copy(std::next(args.begin(), 1), args.end(), scripts.begin());
                        script_delay = 0;
                    }

                    // If scripts are in the config file
                    else if (scripts.size() > 0) {
                        NUClear::log<NUClear::INFO>("Executing: ", scripts.size(), " scripts");
                        scripts = config["scripts"].as<std::vector<std::string>>();
                    }

                    // No default scripts or commandline scripts
                    else {
                        NUClear::log<NUClear::WARN>("No scripts loaded");
                    }
                });

            sensorHandle = on<Trigger<DarwinSensors>, Single>().then([this] {
                executeNextScript();
                sensorHandle.disable();
                sensorHandle.unbind();
            });

            on<Trigger<ExecuteNextScript>>().then([this] { executeNextScript(); });

            emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "Script Runner",
                {std::pair<float, std::set<LimbID>>(
                    1, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {
                    // We should always be the only running thing
                },
                [this](const std::set<ServoID>&) {
                    on<Trigger<ButtonMiddleDown>>().then([this] {
                        std::this_thread::sleep_for(std::chrono::seconds(script_delay));
                        emit(std::make_unique<ExecuteNextScript>());
                    });
                }}));
        }

    }  // namespace tools
}  // namespace behaviour
}  // namespace module
