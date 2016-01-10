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
#include "message/localisation/FieldObject.h"
#include "message/input/gameevents/GameEvents.h"
#include "message/support/nubugger/proto/Overview.pb.h"
#include "message/behaviour/proto/Subsumption.pb.h"
#include "message/support/nubugger/proto/ConfigurationState.pb.h"

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

                NUsightMessage() {};

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

            message::support::nubugger::proto::Overview overview;
            std::map<uint, message::behaviour::proto::Subsumption::ActionRegister> actionRegisters;

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

            void sendGameState(std::string event, const message::input::gameevents::GameState& gameState);
            message::input::proto::GameState::Data::Phase getPhase(const message::input::gameevents::Phase& phase);
            message::input::proto::GameState::Data::Mode getMode(const message::input::gameevents::Mode& phase);
            message::input::proto::GameState::Data::PenaltyReason getPenaltyReason(const message::input::gameevents::PenaltyReason& penaltyReason);

            void sendConfigurationState();
            void sendSubsumption();

            template <typename T>
            void send(T&& proto, uint8_t filterId = 0, bool reliable = false, NUClear::clock::time_point time = NUClear::clock::now()) {
                using ProtobufType = std::remove_reference_t<T>;

                // Wrap our protobuf in a message with filter information
                auto msg = std::make_unique<NUsightMessage<ProtobufType>>(std::forward<T>(proto), filterId, std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count());

                // Send the message over the network
                if(networkEnabled) {
                    emit<Scope::NETWORK>(msg, "nusight", reliable);
                }
                // If we are writing to a file
                if(fileEnabled && outputFile) {
                    // Lock the file mutex
                    std::lock_guard<std::mutex> lock(fileMutex);

                    // Get the details
                    uint32_t size = msg->proto.ByteSize() + (sizeof(uint64_t) * 3);
                    std::array<uint64_t, 2> hash = NUClear::util::serialise::Serialise<ProtobufType>::hash();

                    // Write the size
                    outputFile.write(reinterpret_cast<char*>(&size), sizeof(uint32_t));
                    // Write the timestamp
                    outputFile.write(reinterpret_cast<char*>(&msg->timestamp), sizeof(uint64_t));
                    // Write the hash
                    outputFile.write(reinterpret_cast<char*>(hash.data()), sizeof(hash));
                    // Write the protocol buffer
                    msg->proto.SerializeToOstream(&outputFile);
                }
            }

            // void recvMessage(const message::support::nubugger::proto::Message& message);
            // void recvCommand(const message::support::nubugger::proto::Message& message);
            // void recvLookupTable(const message::support::nubugger::proto::Message& message);
            // void recvReactionHandles(const message::support::nubugger::proto::Message& message);
            void recvConfigurationState(const message::support::nubugger::proto::ConfigurationState& state);

            void EmitLocalisationModels(
                const std::unique_ptr<message::localisation::FieldObject>& robot_model,
                const std::unique_ptr<message::localisation::FieldObject>& ball_model);

            // message::support::nubugger::proto::Message::Type getMessageTypeFromString(std::string type_name);
            // std::string getStringFromMessageType(message::support::nubugger::proto::Message::Type type);
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

                static inline std::vector<char> serialise(const Type& in) {

                    constexpr int metasize = sizeof(uint8_t) + sizeof(uint64_t);

                    // Allocate our buffer
                    std::vector<char> output(in.proto.ByteSize() + metasize);

                    // Get the pointers for our metadata
                    uint8_t* filterid = reinterpret_cast<uint8_t*>(output.data());
                    uint64_t* timestamp = reinterpret_cast<uint64_t*>(output.data() + sizeof(uint8_t));

                    // Write our metadata
                    *filterid = in.filterid;
                    *timestamp = in.timestamp;

                    // Write our actual protocol buffer
                    in.proto.SerializeToArray(output.data() + metasize, output.size() - metasize);

                    return output;
                }

                static inline Type deserialise(const std::vector<char>& in) {
                    // Make an output
                    Type out;

                    // Get the pointers for our metadata
                    uint8_t* filterid = reinterpret_cast<uint8_t*>(in.data());
                    uint64_t* timestamp = reinterpret_cast<uint64_t*>(in.data() + 1);

                    // Deserialize it
                    out.filterid = *filterid;
                    out.timestamp = *timestamp;
                    out.proto.ParseFromArray(in.data(), in.size());

                    return out;
                }

                static inline std::array<uint64_t, 2> hash() {

                    // We have to construct an instance to call the reflection functions
                    T type;

                    auto name = "NUsight<" + type.GetTypeName() + ">";

                    // We base the hash on the name of the protocol buffer
                    return murmurHash3(name.c_str(), name.size());
                }
            };
        }
    }
}

#endif  // MODULES_SUPPORT_NUBUGGER_H
