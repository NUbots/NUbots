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
#include "Say.hpp"

#include <cstdlib>
#include <string>
#include <thread>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/skill/Say.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::HeadSequence;
    using utility::skill::load_script;
    using SayTask = message::skill::Say;

    Say::Say(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Say.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Say.yaml
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.voice        = config["voice"].as<std::string>();
            cfg.device_name  = config["device_name"].as<std::string>();
            cfg.startup_text = config["startup_text"].as<std::string>();
        });

        on<Startup>().then([this] {
            if (!cfg.startup_text.empty()) {
                emit<Task>(std::make_unique<SayTask>(cfg.startup_text, false));
            }
        });

        on<Provide<SayTask>>().then([this](const SayTask& say, const RunInfo& info) {
            // Only say text if it is a new task
            if (info.run_reason == RunInfo::NEW_TASK) {
                // Play the requested audio using python command-line tool mimic3 and aplay
                // Sanitize the text to remove special characters which could break the command
                std::string sanitized_text         = say.text;
                const std::string chars_to_replace = "'\"`|;&$\\";
                for (char c : chars_to_replace) {
                    sanitized_text.erase(std::remove(sanitized_text.begin(), sanitized_text.end(), c),
                                         sanitized_text.end());
                }
                log<DEBUG>("Saying: ", sanitized_text);
                system(std::string("mimic3 '" + sanitized_text + "' --voice '" + cfg.voice + "' | aplay -D"
                                   + cfg.device_name)
                           .c_str());

                // Nod head to indicate that the robot is "talking"
                if (say.nod) {
                    emit<Task>(load_script<HeadSequence>("Say.yaml"));
                }
            }
        });
    }

}  // namespace module::skill
