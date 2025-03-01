/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
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
#include "DataPlayback.hpp"

#include <filesystem>

#include "read_packet.hpp"

#include "extension/Configuration.hpp"
namespace module::support::logging {

    using extension::Configuration;
    using NUClear::message::CommandLineArguments;
    using NUClear::util::serialise::xxhash64;

    DataPlayback::DataPlayback(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), first_timecode(0) {

        // Register players for all the message types we know about
        register_players();

        // Loop two times per second to check we are buffered enough
        playback_handle = on<Every<2, Per<std::chrono::seconds>>, Sync<DataPlayback>, Single>().then([this] {
            bool buffered = false;
            while (!buffered) {
                try {
                    // If we haven't loaded a file yet we need to load one
                    if (!input_file) {
                        throw std::runtime_error("No file loaded yet");
                    }

                    // Read our next packet from the stream
                    Packet p = read_packet(*input_file);

                    // If our first_timecode is 0, this is our first packet for this cycle
                    if (first_timecode.count() == 0) {
                        start_time     = NUClear::clock::now();
                        first_timecode = p.timecode;
                    }

                    // Work out when we should emit this packet
                    auto emit_time = start_time + p.timecode - first_timecode;

                    // Since it's possible for NBS files to be a little out of order, we also ensure that we
                    // always take the most distant emit time as our time
                    last_emit_time = last_emit_time > emit_time ? last_emit_time : emit_time;

                    // Check if we care about this packet
                    if (players.find(p.hash) != players.end() && players[p.hash].enabled) {

                        // Emit this packet at the appropriate time
                        players[p.hash].emit(emit_time, p.payload);
                    }

                    // If this time is further in the future than our buffer time we can finish for this round
                    buffered = emit_time - NUClear::clock::now() > buffer_time;
                }

                catch (const std::exception& ex) {

                    // If we reached the end of the file we either terminate or loop depending on the setting
                    if (!input_file || input_file->eof()) {

                        // Don't print this on the first loop
                        if (file_index >= 0) {
                            log<INFO>("Playback of file", files[file_index], "finished");
                        }

                        // If we have more files to go in our list
                        if (file_index + 1 < int(files.size())) {
                            // Go to the next file in the list
                            file_index += 1;
                            // This resets the timer offset for the new file
                            first_timecode = std::chrono::microseconds(0);
                            // This makes sure we start emitting the new data after the old is finished
                            start_time = last_emit_time;

                            // Open the first file again
                            if (std::filesystem::exists(files[file_index])) {
                                input_file = std::make_unique<std::ifstream>(files[file_index]);
                                log<INFO>("Starting playback of file", files[file_index]);
                            }
                            else {
                                log<ERROR>("The file", files[file_index], "does not exist!");
                            }
                        }
                        // We are done and should shutdown the system now
                        else if (on_end == SHUTDOWN_ON_END) {
                            playback_handle.disable();
                            powerplant.shutdown();
                        }
                        else if (on_end == LOOP_ON_END) {
                            // Go back to the first file in the list
                            file_index = 0;
                            // This resets the timer offset for the new file
                            first_timecode = std::chrono::microseconds(0);
                            // This makes sure we start emitting the new data after the old is finished
                            start_time = last_emit_time;

                            // Open the first file again
                            if (std::filesystem::exists(files[file_index])) {
                                input_file = std::make_unique<std::ifstream>(files[file_index]);
                                log<INFO>("Restarting playback with file", files[file_index]);
                            }
                            else {
                                log<ERROR>("Cannot restart with file", files[file_index], "as it does not exist!");
                            }
                        }
                        // Just stop playing back the recording
                        else {
                            playback_handle.disable();
                            input_file = nullptr;
                            buffered   = true;
                        }
                    }
                    // We tried to read something funny (possibly tried to read too many bytes)
                    else if (!input_file->good()) {
                        input_file->clear();
                    }
                    else {
                        // We don't know what's up, throw the exception again
                        throw ex;
                    }
                }
            }
        });
        playback_handle.disable();

        on<Configuration, Trigger<CommandLineArguments>, Sync<DataPlayback>>("DataPlayback.yaml")
            .then([this](const Configuration& config, const CommandLineArguments& args) {
                // Update which types we will be playing
                for (const auto& setting : config["messages"].config) {
                    // Get the name of the type
                    auto name = setting.first.as<std::string>();
                    // Hash our type to work out our type on the wire
                    uint64_t hash = xxhash64(name.c_str(), name.size(), 0x4e55436c);
                    bool enabled  = setting.second.as<bool>();

                    // Message if we have enabled/disabled a particular message type
                    if (players.find(hash) != players.end()) {
                        auto& player = players[hash];

                        if (enabled && !player.enabled) {
                            player.enabled = true;
                            log<INFO>("Playback for message type", name, "enabled");
                        }
                        else if (!enabled && player.enabled) {
                            player.enabled = false;
                            log<INFO>("Playback for message type", name, "disabled");
                        }
                    }
                    else {
                        log<WARN>("The playback system does not know about the message type", name);
                    }
                }

                // Get which files the configuration file said to play
                files = config["files"].as<std::vector<std::string>>();

                // If we are provided no files in the configuration file, try to read them from the command line
                if (files.empty()) {
                    files.insert(files.end(), std::next(args.begin()), args.end());
                }

                // Work out what to do on the end of the file
                auto end_string = config["on_end"].as<std::string>();
                if (end_string == "STOP") {
                    on_end = STOP_ON_END;
                }
                else if (end_string == "LOOP") {
                    on_end = LOOP_ON_END;
                }
                else if (end_string == "SHUTDOWN") {
                    on_end = SHUTDOWN_ON_END;
                }
                else {
                    throw std::runtime_error("Unknown on end option " + end_string
                                             + " must be one of STOP, LOOP or SHUTDOWN");
                }

                buffer_time = std::chrono::milliseconds(config["buffer_time"].as<uint64_t>());


                // If we still have no files
                if (files.empty()) {
                    log<WARN>("No files were provided for playback, stopping");
                    playback_handle.disable();
                }
                else {
                    // Start at -1 so we skip ahead and start at 0
                    file_index = -1;
                    playback_handle.enable();
                }
            });
    }

}  // namespace module::support::logging
