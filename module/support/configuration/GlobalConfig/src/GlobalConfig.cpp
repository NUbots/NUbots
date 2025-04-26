/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#include "GlobalConfig.hpp"

#include "extension/Configuration.hpp"

#include "message/support/GlobalConfig.hpp"

namespace module::support::configuration {
    using extension::Configuration;
    using NUClear::message::CommandLineArguments;

    GlobalConfig::GlobalConfig(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration, Trigger<CommandLineArguments>>("GlobalConfig.yaml")
            .then([this](const Configuration& config, const CommandLineArguments& args) {
                auto msg       = std::make_unique<message::support::GlobalConfig>();
                msg->player_id = config["player_id"].as<uint32_t>();
                msg->team_id   = config["team_id"].as<uint32_t>();

                // Check if command line arguments contain "--player_id" and "--team_id"
                // Find "--player_id" and get the value after
                if (std::find(args.begin(), args.end(), "--player_id") != args.end()) {
                    // Get the index of the player_id
                    auto it = std::find(args.begin(), args.end(), "--player_id");
                    if (it + 1 != args.end()) {
                        try {
                            msg->player_id = static_cast<uint32_t>(std::stoul(*(it + 1)));
                        }
                        catch (const std::exception& e) {
                            // Handle invalid conversion
                            log<ERROR>("Invalid value for --player_id:", *(it + 1));
                        }
                    }
                }

                // Find "--team_id" and get the value after
                if (std::find(args.begin(), args.end(), "--team_id") != args.end()) {
                    // Get the index of the team_id
                    auto it = std::find(args.begin(), args.end(), "--team_id");
                    if (it + 1 != args.end()) {
                        try {
                            msg->team_id = static_cast<uint32_t>(std::stoul(*(it + 1)));
                        }
                        catch (const std::exception& e) {
                            // Handle invalid conversion
                            log<ERROR>("Invalid value for --team_id:", *(it + 1));
                        }
                    }
                }

                emit(msg);
            });
    }
}  // namespace module::support::configuration
