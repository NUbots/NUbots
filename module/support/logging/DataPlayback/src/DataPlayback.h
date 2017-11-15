#ifndef MODULE_SUPPORT_LOGGING_DATAPLAYBACK_H
#define MODULE_SUPPORT_LOGGING_DATAPLAYBACK_H

#include <fstream>
#include <nuclear>

namespace module {
namespace support {
    namespace logging {

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
                std::function<void(const NUClear::clock::time_point&, const std::vector<char>&)> emit;
                // If this player should execute and send messages
                bool enabled = false;
            };

            // Our deserialisation functions that convert the messages and emit them
            std::map<uint64_t, Player> players;

            // Our reaction that controls the main every loop that queues up actions
            ReactionHandle playback_handle;

            // Our input file that we read from
            std::ifstream input_file;

            // The file we are currently playing
            std::string current_file;

            // The first time that appears in the file
            std::chrono::microseconds first_timecode;

            // The amount of time into the future to buffer for
            std::chrono::milliseconds buffer_time;

            // When we started playing this file
            NUClear::clock::time_point start_time;

            // If we should loop this file after we finish or just disable ourself
            bool loop_playback = false;

            template <typename T>
            void add_player() {
                uint64_t hash = NUClear::util::serialise::Serialise<T>::hash();
                Player p;
                p.emit = [this](const NUClear::clock::time_point& emit_time, const std::vector<char>& payload) {
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
        };

    }  // namespace logging
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_LOGGING_DATAPLAYBACK_H
