/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "UncertaintyResetTester.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::nbs::player::LoadRequest;
    using message::nbs::player::PlaybackFinished;
    using message::nbs::player::PlaybackMode::FAST;
    using message::nbs::player::PlaybackMode::REALTIME;
    using message::nbs::player::PlaybackMode::SEQUENTIAL;
    using message::nbs::player::PlayRequest;
    using message::nbs::player::SetModeRequest;

    using message::localisation::FinishReset;
    using message::localisation::PenaltyReset;
    using message::localisation::UncertaintyResetFieldLocalisation;

    using utility::nusight::graph;
    using utility::support::Expression;

    using NUClear::message::CommandLineArguments;

    UncertaintyResetTester::UncertaintyResetTester(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("UncertaintyResetTester.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            const auto playback_mode = cfg["playback_mode"].as<std::string>();
            if (playback_mode == "FAST") {
                config.playback_mode = FAST;
            }
            else if (playback_mode == "REALTIME") {
                config.playback_mode = REALTIME;
            }
            else if (playback_mode == "SEQUENTIAL") {
                config.playback_mode = SEQUENTIAL;
            }
            else {
                log<ERROR>("Invalid playback_mode, shutting down");
                powerplant.shutdown();
                return;
            }

            config.messages.clear();
            for (const auto& setting : cfg["messages"]) {
                if (setting.second.as<bool>()) {
                    config.messages.push_back(setting.first.as<std::string>());
                }
            }

            config.force_reset_period   = cfg["force_reset_period"].as<double>();
            config.force_reset_position = Eigen::Vector3d(cfg["force_reset_position"].as<Expression>());
        });

        // Reset detection: capture the moment the localisation module emits the start-of-reset signal.
        on<Trigger<UncertaintyResetFieldLocalisation>>().then([this] {
            reset_started     = std::chrono::steady_clock::now();
            reset_in_progress = true;
            log<INFO>("UncertaintyResetFieldLocalisation observed; timer started");
        });

        // Reset completion: compute the wall-clock duration and record/log/graph it.
        on<Trigger<FinishReset>>().then([this] {
            if (!reset_in_progress) {
                log<WARN>("FinishReset observed without a preceding UncertaintyResetFieldLocalisation; ignoring");
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            const double duration_s =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - reset_started).count();
            durations_s.push_back(duration_s);
            reset_in_progress = false;

            log<INFO>("Uncertainty reset completed in", duration_s, "s (sample", durations_s.size(), ")");
            emit(graph("Uncertainty reset duration (s)", duration_s));
        });

        // Optional periodic provocation: push the filter to a known-bad pose so cost-driven reset fires.
        on<Every<1, std::chrono::seconds>>().then([this] {
            if (config.force_reset_period <= 0.0) {
                return;
            }
            static auto last_force = std::chrono::steady_clock::now();
            const auto now         = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - last_force).count();
            if (elapsed >= config.force_reset_period) {
                last_force = now;
                log<INFO>("Forcing filter into bad pose to provoke an uncertainty reset");
                auto reset                   = std::make_unique<PenaltyReset>();
                reset->penalty_kick_position = config.force_reset_position;
                emit(std::move(reset));
            }
        });

        // Drive the NBS player to start once the system is up.
        on<Startup, With<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            auto set_mode  = std::make_unique<SetModeRequest>();
            set_mode->mode = config.playback_mode;
            emit<Scope::INLINE>(std::move(set_mode));

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            auto load      = std::make_unique<LoadRequest>();
            load->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load->messages = config.messages;
            emit<Scope::INLINE>(std::move(load));

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            emit<Scope::INLINE>(std::make_unique<PlayRequest>());
        });

        // End-of-playback summary.
        on<Trigger<PlaybackFinished>>().then([this] {
            if (durations_s.empty()) {
                std::cout << "UncertaintyResetTester: no uncertainty resets observed during playback" << std::endl;
                powerplant.shutdown();
                return;
            }

            const double sum = std::accumulate(durations_s.begin(), durations_s.end(), 0.0);
            const double mean = sum / static_cast<double>(durations_s.size());
            const double mn   = *std::min_element(durations_s.begin(), durations_s.end());
            const double mx   = *std::max_element(durations_s.begin(), durations_s.end());

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "UncertaintyResetTester: " << durations_s.size() << " resets observed" << std::endl;
            std::cout << "  mean duration: " << mean << " s" << std::endl;
            std::cout << "  min  duration: " << mn << " s" << std::endl;
            std::cout << "  max  duration: " << mx << " s" << std::endl;

            powerplant.shutdown();
        });
    }

}  // namespace module::tools
