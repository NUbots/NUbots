/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "NBSPlayback.hpp"

#include "extension/Configuration.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::eye::ScrubberLoadRequest;
    using message::eye::ScrubberPlaybackFinished;
    using message::eye::ScrubberPlayRequest;
    using message::eye::ScrubberSetModeRequest;
    using message::eye::ScrubberState;

    using NUClear::message::CommandLineArguments;

    NBSPlayback::NBSPlayback(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("NBSPlayback.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            config.mode              = cfg["playback_mode"].as<std::string>();
            config.progress_bar_mode = cfg["progress_bar_mode"].as<std::string>();

            // Update which types we will be playing
            for (const auto& setting : cfg["messages"]) {
                auto name    = setting.first.as<std::string>();
                bool enabled = setting.second.as<bool>();
                if (enabled) {
                    config.messages.push_back(name);
                }
            }
        });

        on<Startup, With<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            // Set playback mode
            auto set_mode_request  = std::make_unique<ScrubberSetModeRequest>();
            set_mode_request->mode = config.mode;
            emit<Scope::INLINE>(set_mode_request);

            // Load the files
            auto load_request      = std::make_unique<ScrubberLoadRequest>();
            load_request->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load_request->messages = config.messages;
            emit<Scope::INLINE>(load_request);

            // Start playback
            emit<Scope::INLINE>(std::make_unique<ScrubberPlayRequest>());
        });

        on<Trigger<ScrubberState>>().then([this](const ScrubberState& scrubber_state) {
            // Update progress bar
            if (config.progress_bar_mode == "COUNT") {
                progress_bar.update(scrubber_state.current_message,
                                    scrubber_state.total_messages,
                                    " msgs",
                                    "NBS Playback");
            }
            else if (config.progress_bar_mode == "TIME") {
                progress_bar.update(
                    std::chrono::duration_cast<std::chrono::seconds>(scrubber_state.timestamp.time_since_epoch()
                                                                     - scrubber_state.start.time_since_epoch())
                        .count(),
                    std::chrono::duration_cast<std::chrono::seconds>(scrubber_state.end.time_since_epoch()
                                                                     - scrubber_state.start.time_since_epoch())
                        .count(),
                    " seconds",
                    "NBS Playback");
            }
            else {
                log<ERROR>("Invalid progress bar mode");
            }
        });

        on<Trigger<ScrubberPlaybackFinished>>().then([this] {
            log<INFO>("Playback finished.");
            progress_bar.close();
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
