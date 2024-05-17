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
#ifndef MODULE_NBS_PLAYER_HPP
#define MODULE_NBS_PLAYER_HPP

#include <filesystem>
#include <fstream>
#include <memory>
#include <nuclear>

#include "message/nbs/player/Player.hpp"

#include "utility/nbs/Decoder.hpp"

namespace module::nbs {

    class Player : public NUClear::Reactor {
    private:
        /**
         * @brief Registers an emitter for a specific message type `T`.
         *
         * This function will add a callback function for the message type `T` to the decoder such that  the decoder
         * will emit the message when it is iterated over.
         *
         * @tparam T The type of message to register the emitter for.
         */
        template <typename T>
        void register_emitter() {
            uint64_t hash  = NUClear::util::serialise::Serialise<T>::hash();
            emitters[hash] = [this](utility::nbs::Decoder& decoder) {
                decoder.on<T>([this](const T& msg) { emit(std::make_unique<T>(msg)); });
            };
        }

    public:
        /// Called by the powerplant to build and setup the Player reactor.
        explicit Player(std::unique_ptr<NUClear::Environment> environment);

        // Our deserialisation functions that convert the messages and emit them
        std::map<uint64_t, std::function<void(utility::nbs::Decoder&)>> emitters;

        /// Register the functions that will decode and emit the message types
        void register_emitters();

        /// NBS file decoder
        utility::nbs::Decoder decoder;

        /// Playback mode
        message::nbs::player::PlaybackMode mode = message::nbs::player::PlaybackMode::REALTIME;

        /// Playback speed (clock rtf)
        double playback_speed = 1;

        /// NBS file iterator
        utility::nbs::Decoder::Iterator decoder_iterator = decoder.begin();

        /// Mutex for the decoder and idle mutex logic
        std::mutex decoder_mutex;

        /// Idle handle
        NUClear::threading::ReactionHandle idle_handle;

        /// On always main loop handle
        NUClear::threading::ReactionHandle main_loop_handle;

        /// Target time for the next message emit
        NUClear::clock::time_point target_emit_time;

        /// Condition variable to signal when the decoder loop should continue
        std::condition_variable cv;

        /// The total number of messages in the NBS file being played
        uint64_t total_messages = 0;

        /// The earliest timestamp across all files in the player
        NUClear::clock::time_point start_time = NUClear::clock::time_point::max();

        /// The latest timestamp across all files in the player
        NUClear::clock::time_point end_time = NUClear::clock::time_point::min();

        /// Tries to emit the next message from the decoder if it is time
        void emit_next_message();

        /// Adds and enables idle handle, which will run when the system is idle and try to emit the next message
        void enable_idle_handle();

        /// Adds and enable main loop handle, which will always run trying to emit the next message
        void enable_main_loop_handle();
    };

}  // namespace module::nbs

#endif  // MODULE_NBS_PLAYER_HPP
