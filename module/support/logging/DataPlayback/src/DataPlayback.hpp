/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef MODULE_SUPPORT_LOGGING_DATAPLAYBACK_HPP
#define MODULE_SUPPORT_LOGGING_DATAPLAYBACK_HPP

#include <fstream>
#include <memory>
#include <nuclear>

namespace module::support::logging {

    class DataPlayback : public NUClear::Reactor {
    private:
        // FILE FORMAT
        // TYPE       DATA
        // char[3]    RADIATION SYMBOL {{ 0xE2, 0x98, 0xA2 }}
        // uint32_t   SIZE OF NEXT PACKET
        // uint64_t   TIMESTAMP DATA WAS EMITTED IN MICROSECONDS
        // uint64_t   DATA TYPE HASH

        struct Player {
            // The function that will queue this message to be emitted
            std::function<void(const NUClear::clock::time_point&, const std::vector<uint8_t>&)> emit;
            // If this player should execute and send messages
            bool enabled = false;
        };

        template <typename T>
        void add_player() {
            uint64_t hash = NUClear::util::serialise::Serialise<T>::hash();
            Player p;
            p.emit = [this](const NUClear::clock::time_point& emit_time, const std::vector<uint8_t>& payload) {
                // Deserialise our type
                auto msg = std::make_unique<T>(NUClear::util::serialise::Serialise<T>::deserialise(payload));

                // Emit it after the delay
                emit<Scope::DELAY>(msg, emit_time - NUClear::clock::now());
            };

            // Default to not emitting this type
            p.enabled = false;

            // Add to our players list
            players.insert(std::make_pair(hash, p));
        }

    public:
        /// @brief Called by the powerplant to build and setup the DataPlayback reactor.
        explicit DataPlayback(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Register the functions that will decode and emit the message types
        void register_players();

        // Our deserialisation functions that convert the messages and emit them
        std::map<uint64_t, Player> players;

        // Our reaction that controls the main every loop that queues up actions
        ReactionHandle playback_handle;

        // Our input file that we read from
        std::unique_ptr<std::ifstream> input_file;

        // The list of files we are playing
        std::vector<std::string> files;

        // Which file number we are up to
        int file_index = 0;

        // The first time that appears in the file
        std::chrono::microseconds first_timecode;

        // The amount of time into the future to buffer for
        std::chrono::milliseconds buffer_time{};

        // When we started playing this file
        NUClear::clock::time_point start_time;

        // The last time that we emitted a packet for
        NUClear::clock::time_point last_emit_time;

        // If we should stop, loop or shutdown when we finish the files
        enum { SHUTDOWN_ON_END, STOP_ON_END, LOOP_ON_END } on_end = STOP_ON_END;
    };

}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_DATAPLAYBACK_HPP
