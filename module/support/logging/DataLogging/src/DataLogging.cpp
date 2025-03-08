/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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
#include "DataLogging.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <zstr.hpp>

#include "extension/Configuration.hpp"

#include "utility/nbs/Index.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::logging {

    using extension::Configuration;
    using NUClear::message::CommandLineArguments;
    using utility::support::Expression;

    std::string formatted_time() {
        std::time_t now     = time(nullptr);
        std::tm system_time = *localtime(&now);
        std::stringstream time;
        time << std::put_time(&system_time, "%Y%m%dT%H_%M_%S");
        return time.str();
    }

    DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        /// This receives every DataLog message as a Sync operation (one at a time) and writes it to the file
        on<Trigger<DataLog>, Sync<DataLog>>().then([this](const DataLog& data) {
            // NBS File Format
            // Name      | Type               |  Description
            // ------------------------------------------------------------
            // header    | char[3]            | NBS packet header ☢ { 0xE2, 0x98, 0xA2 }
            // length    | uint32_t           | Length of this packet after this value
            // timestamp | uint64_t           | Timestamp the data was emitted in microseconds
            // hash      | uint64_t           | the 64bit hash for the payload type
            // payload   | char[length - 16]  | the data payload

            // Convert the timestamp to a 64bit microsecond timestamp
            uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(
                                        data.timestamp.time_since_epoch())
                                        .count();

            // The size of our output timestamp hash and data
            uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

            // If the file isn't open, or writing this message will exceed our max size, make a new file
            // If we are encrypting the encoder can actually add a few extra bytes over this so it is possible we end up
            // with more bytes of data than our maximum.
            if (!encoder || !encoder->is_open()
                || (encoder->get_bytes_written() + size + 3 + sizeof(timestamp_us) + sizeof(data.hash))
                       >= config.output.split_size) {
                if (encoder) {
                    encoder->close();
                }

                // Creates directory for output
                std::filesystem::create_directories(config.output.directory / config.output.binary);

                // Creates the output ".nbs" file path.
                output_file_path = std::filesystem::path();

                // If we are encrypting it's an "nbe" file otherwise it's "nbs"
                std::string ext = config.output.passphrase.empty() ? ".nbs" : ".nbe";
                output_file_path +=
                    config.output.directory / config.output.binary / fmt::format("{}{}", formatted_time(), ext);

                // Creates the output ".idx" file path.
                index_file_path = output_file_path;
                index_file_path += ".idx";

                // Make a new encoder
                if (config.output.passphrase.empty()) {
                    encoder = std::make_unique<utility::nbs::Encoder>(output_file_path);
                }
                else {
                    encoder = std::make_unique<utility::nbs::Encoder>(output_file_path, config.output.passphrase);
                }
            }

            encoder->write(data.timestamp, data.message_timestamp, data.hash, data.subtype, data.data);
        });

        on<Startup>().then([this] {
            // Make a pass through the output directory and ensure there are no stale lock files
            std::error_code ec;
            for (const auto& dir_entry :
                 std::filesystem::directory_iterator{config.output.directory / config.output.binary, ec}) {
                if (dir_entry.path().extension() == ".lock") {
                    log<WARN>(fmt::format("Removing old lock file: {}", dir_entry.path().string()));
                    std::filesystem::remove(dir_entry.path());
                }
            }

            // Check for any errors that occurred and report them
            if (ec) {
                log<ERROR>(fmt::format("Error iterating through '{}': {}",
                                       (config.output.directory / config.output.binary).string(),
                                       ec.message()));
            }
        });

        on<Shutdown>().then([this] {
            if (encoder == nullptr) {
                return;
            }

            encoder->close();
        });

        on<Configuration, Trigger<CommandLineArguments>, Sync<DataLog>>("DataLogging.yaml")
            .then([this](const Configuration& cfg, const CommandLineArguments& argv) {
                // Get the details we need to generate a log file name
                config.output.directory  = cfg["output"]["directory"].as<std::string>();
                config.output.split_size = uint64_t(cfg["output"]["split_size"].as<Expression>());

                // If they don't provide a passphrase we assume it's not encrypted
                config.output.passphrase =
                    bool(cfg["output"]["passphrase"]) ? cfg["output"]["passphrase"].as<std::string>() : "";

                // Get the name of the currently running binary
                std::vector<char> data(argv[0].cbegin(), argv[0].cend());
                data.push_back('\0');
                const auto* base     = basename(data.data());
                config.output.binary = std::string(base);

                // Rescue any existing recorders that we want to keep
                std::map<std::string, ReactionHandle> new_handles;
                for (const auto& setting : cfg["messages"]) {
                    auto name    = setting.first.as<std::string>();
                    bool enabled = setting.second.as<bool>();

                    // If it was enabled and we are keeping it enabled, keep it and remove it from the old list
                    if (handles.contains(name) && enabled) {
                        new_handles.insert(std::make_pair(name, handles[name]));
                        handles.erase(handles.find(name));
                    }
                }

                // Unbind any recorders we didn't save
                for (auto& handle : handles) {
                    log<INFO>("Data logging for type", handle.first, "disabled");
                    handle.second.unbind();
                }

                // Add any new recorders that we don't have yet
                for (const auto& setting : cfg["messages"]) {
                    auto name    = setting.first.as<std::string>();
                    bool enabled = setting.second.as<bool>();

                    // If we are enabling this, and it wasn't already enabled, enable it
                    if (!new_handles.contains(name) && enabled) {
                        log<INFO>("Data logging for type", name, "enabled");
                        new_handles.insert(std::make_pair(name, activate_recorder(name)));
                    }
                }

                // New handles become the handles
                handles = std::move(new_handles);
            });
    }

}  // namespace module::support::logging
