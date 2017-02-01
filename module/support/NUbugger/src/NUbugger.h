/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_SUPPORT_NUBUGGER_H
#define MODULES_SUPPORT_NUBUGGER_H

#include <nuclear>
#include <fstream>

#include "extension/Configuration.h"

#include "message/behaviour/Subsumption.h"
#include "message/localisation/FieldObject.h"
#include "message/input/GameEvents.h"
#include "message/input/GameState.h"
#include "message/support/nubugger/Overview.h"

namespace module {
    namespace support {

        /**
         * Intelligent debugging, logging and graphing for the NUbots system.
         *
         * @author Brendan Annable
         * @author Trent Houliston
         */
        class NUbugger : public NUClear::Reactor {
        private:

            template <typename T>
            struct NUsightMessage {

                NUsightMessage()
                : proto()
                , filterid()
                , timestamp() {}

                NUsightMessage(const T& proto
                               , uint8_t filterid
                               , uint64_t timestamp)
                : proto(proto)
                , filterid(filterid)
                , timestamp(timestamp) {}

                T proto;
                uint8_t filterid;
                uint64_t timestamp;
            };

            uint pubPort = 0;
            uint subPort = 0;

            NUClear::clock::duration max_image_duration;
            NUClear::clock::time_point last_image = NUClear::clock::now();
            NUClear::clock::duration max_classified_image_duration;
            NUClear::clock::time_point last_classified_image = NUClear::clock::now();

            bool listening = true;

            // Reaction Handles
            std::map<std::string, std::vector<ReactionHandle>> handles;

            std::map<std::string, uint> dataPointFilterIds;
            uint dataPointFilterId = 1;

            // Send control
            bool networkEnabled = false;
            bool fileEnabled = false;

            message::support::nubugger::Overview overview;
            std::map<uint, message::behaviour::Subsumption::ActionRegister> actionRegisters;

            std::ofstream outputFile;

            std::mutex networkMutex;
            std::mutex fileMutex;


            void provideOverview();
            void provideDataPoints();
            void provideDrawObjects();
            void provideSubsumption();
            void provideGameController();
            void provideLocalisation();
            void provideReactionStatistics();
            void provideSensors();
            void provideVision();

            void sendReactionHandles();

            void sendGameState(std::string event, const message::input::GameState& gameState);

            void saveConfigurationFile(std::string path, const YAML::Node& root);
            void sendSubsumption();

            template <typename T>
            void send(T&& proto, uint8_t filterId = 0, bool reliable = false, NUClear::clock::time_point time = NUClear::clock::now()) {
                using ProtobufType = std::remove_reference_t<T>;

                // Wrap our protobuf in a message with filter information
                auto msg = std::make_unique<NUsightMessage<ProtobufType>>(std::forward<T>(proto), 
                                            filterId, std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count());

                // Send the message over the network
                if(networkEnabled) {
                    emit<Scope::NETWORK>(msg, "nusight", reliable);
                }
            }

            void EmitLocalisationModels(
                const std::unique_ptr<message::localisation::FieldObject>& robot_model,
                const std::unique_ptr<message::localisation::FieldObject>& ball_model);

            // message::support::nubugger::Message::Type getMessageTypeFromString(std::string type_name);
            // std::string getStringFromMessageType(message::support::nubugger::Message::Type type);
        public:
            static constexpr const char* IGNORE_TAG = "IGNORE";
            explicit NUbugger(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // support
}  // modules

// Serialisation for NUsight messages
namespace NUClear {
    namespace util {
        namespace serialise {
            template <typename T>
            struct Serialise<module::support::NUbugger::NUsightMessage<T>, module::support::NUbugger::NUsightMessage<T>> {

                using Type = module::support::NUbugger::NUsightMessage<T>;
                using protobuf_type = typename T::protobuf_type;

                static inline std::vector<char> serialise(const Type& in) {

                    constexpr int metasize = sizeof(uint8_t) + sizeof(uint64_t);

                    protobuf_type proto = in.proto;

                    // Allocate our buffer
                    std::vector<char> output(proto.ByteSize() + metasize);

                    // Get the pointers for our metadata
                    uint8_t*  filterid  = reinterpret_cast<uint8_t*>(output.data());
                    uint64_t* timestamp = reinterpret_cast<uint64_t*>(output.data() + sizeof(uint8_t));

                    // Write our metadata
                    *filterid = in.filterid;
                    *timestamp = in.timestamp;

                    // Write our actual protocol buffer
                    proto.SerializeToArray(output.data() + metasize, output.size() - metasize);

                    return output;
                }

                static inline Type deserialise(const std::vector<char>& in) {

                    // Make a buffer
                    protobuf_type out;

                    // Get the pointers for our metadata
                    uint8_t*  filterid  = reinterpret_cast<uint8_t*>(in.data());
                    uint64_t* timestamp = reinterpret_cast<uint64_t*>(in.data() + 1);

                    // Deserialize it
                    out.filterid  = *filterid;
                    out.timestamp = *timestamp;

                    // Deserialize it
                    if (out.ParseFromArray(in.data(), in.size())) {
                        return out;
                    }
                    else {
                        throw std::runtime_error("Message failed to deserialise.");
                    }
                }

                static inline std::array<uint64_t, 2> hash() {

                    // We have to construct an instance to call the reflection functions
                    protobuf_type type;

                    auto name = "NUsight<" + type.GetTypeName().substr(9) + ">";

                    // We base the hash on the name of the protocol buffer
                    return murmurhash3(name.c_str(), name.size());
                }
            };
        }
    }
}

#endif  // MODULES_SUPPORT_NUBUGGER_H
