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

    using message::nbs::player::Finished;
    using message::nbs::player::LoadRequest;
    using message::nbs::player::PauseRequest;
    using message::nbs::player::PlaybackState;
    using message::nbs::player::PlayRequest;
    using message::nbs::player::SetModeRequest;


    using message::nbs::player::PlaybackMode::FAST;
    using message::nbs::player::PlaybackMode::REALTIME;
    using message::nbs::player::PlaybackMode::SEQUENTIAL;

    using NUClear::message::CommandLineArguments;

    NBSPlayback::NBSPlayback(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("NBSPlayback.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            auto playback_mode = cfg["playback_mode"].as<std::string>();
            if (playback_mode == "FAST") {
                config.mode = FAST;
            }
            else if (playback_mode == "SEQUENTIAL") {
                config.mode = SEQUENTIAL;
            }
            else if (playback_mode == "REALTIME") {
                config.mode = REALTIME;
            }
            else {
                log<NUClear::ERROR>("Playback mode is invalid, stopping playback");
                powerplant.shutdown();
            }

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
            auto set_mode_request  = std::make_unique<SetModeRequest>();
            set_mode_request->mode = config.mode;
            emit<Scope::INLINE>(set_mode_request);

            // Load the files
            auto load_request      = std::make_unique<LoadRequest>();
            load_request->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load_request->messages = config.messages;
            emit<Scope::INLINE>(std::move(load_request));

            // Start playback
            emit<Scope::INLINE>(std::make_unique<PlayRequest>());
        });

        on<Trigger<PlaybackState>>().then([this](const PlaybackState& playback_state) {
            // Update progress bar
            progress_bar.update(playback_state.current_message, playback_state.total_messages, "", "NBS Playback");
        });

        on<Trigger<Finished>>().then([this] {
            log<NUClear::INFO>("Finished playback");
            progress_bar.close();
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
