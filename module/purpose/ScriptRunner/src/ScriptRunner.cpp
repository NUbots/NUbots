/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ScriptRunner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/input/Buttons.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;

    using message::actuation::BodySequence;
    using message::input::ButtonMiddleDown;
    using NUClear::message::CommandLineArguments;

    using utility::skill::load_script;

    ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        // Get the scripts to run from the command line
        on<Configuration, With<CommandLineArguments>>("scriptrunner.yaml")
            .then([this](const Configuration& config, const CommandLineArguments& args) {
                scripts = config["scripts"].as<std::vector<std::string>>();

                // Check for scripts entered in the command line
                if (args.size() > 1) {
                    log<NUClear::INFO>("Executing: ", args.size() - 1, " script from argument");
                    scripts.clear();
                    std::copy(std::next(args.begin(), 1), args.end(), std::back_inserter(scripts));
                }

                // If scripts are in the config file
                else if (!scripts.empty()) {
                    log<NUClear::INFO>("Executing: ", scripts.size(), " script from config");
                }

                // No default scripts or command line scripts
                else {
                    log<NUClear::WARN>("No scripts loaded");
                }
            });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            log<NUClear::INFO>("Middle button pressed, running scripts: ");
            for (auto& script : scripts) {
                log<NUClear::INFO>("\n", script);
            }
            emit<Task>(load_script<BodySequence>(scripts));
        });
    }
}  // namespace module::purpose
