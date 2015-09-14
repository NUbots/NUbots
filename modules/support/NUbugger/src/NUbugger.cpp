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

#include "NUbugger.h"

#include "messages/vision/LookUpTable.h"
#include "messages/support/Configuration.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/time/time.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace support {

    using utility::nubugger::graph;

    using messages::support::Configuration;
    using messages::support::nubugger::proto::Message;

    using messages::vision::LookUpTable;
    using messages::vision::SaveLookUpTable;
    using messages::vision::Colour;

    using utility::time::getUtcTimestamp;
    using utility::time::durationFromSeconds;

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {
        // , pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB)
        // , sub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_SUB) {

        // powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&NUbugger::run), this), std::bind(std::mem_fn(&NUbugger::kill), this)));

        on<Configuration>("NUbugger.yaml").then([this] (const Configuration& config) {

            max_image_duration = durationFromSeconds(1.0 / config["output"]["network"]["max_image_fps"].as<double>());
            max_classified_image_duration = durationFromSeconds(1.0 / config["output"]["network"]["max_classified_image_fps"].as<double>());

            // TODO if network disables then we should unbind

            // TODO if the file disables we should close the file

            // TODO if the file location is different, close the file and open a new one

            // If we are using the network
            if (config["output"]["network"]["enabled"].as<bool>()) {

                uint newPubPort = config["output"]["network"]["pub_port"].as<uint>();
                uint newSubPort = config["output"]["network"]["sub_port"].as<uint>();

                if (newPubPort != pubPort) {
                    if (networkEnabled) {
                        // pub.unbind(("tcp://*:" + std::to_string(pubPort)).c_str());
                    }
                    pubPort = newPubPort;
                    // pub.bind(("tcp://*:" + std::to_string(pubPort)).c_str());
                }

                if (newSubPort != subPort) {
                    if (networkEnabled) {
                        // sub.unbind(("tcp://*:" + std::to_string(subPort)).c_str());
                    }
                    subPort = newSubPort;
                    // sub.bind(("tcp://*:" + std::to_string(subPort)).c_str());
                }

                // Set our high water mark
                int hwm = config["output"]["network"]["high_water_mark"].as<int>();
                // pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
                // sub.setsockopt(ZMQ_SUBSCRIBE, 0, 0);

                networkEnabled = true;
            }
            // If we were enabled and now we are not
            else if (networkEnabled && !config["output"]["network"]["enabled"].as<bool>()) {
                // Unbind the network when we disable
                // pub.unbind(("tcp://*:" + std::to_string(pubPort)).c_str());
                // sub.unbind(("tcp://*:" + std::to_string(subPort)).c_str());

                networkEnabled = false;
            }

            // If we are using files and haven't set one up yet
            if (!fileEnabled && config["output"]["file"]["enabled"].as<bool>()) {

                // Lock the file
                std::lock_guard<std::mutex> lock(fileMutex);

                // Get our timestamp
                std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now().time_since_epoch()).count());

                // Open a file using the file name and timestamp
                outputFile.close();
                outputFile.clear();
                outputFile.open(config["output"]["file"]["path"].as<std::string>()
                                + "/"
                                + timestamp
                                + ".nbs", std::ios::binary);

                fileEnabled = true;
            }
            else if (fileEnabled && !config["output"]["file"]["enabled"].as<bool>()) {

                // Lock the file
                std::lock_guard<std::mutex> lock(fileMutex);

                // Close the file
                outputFile.close();
                fileEnabled = false;
            }

            for (auto& setting : config["reaction_handles"]) {
                std::string name = setting.first.as<std::string>();
                bool enabled = setting.second.as<bool>();

                bool found = false;
                for (auto& handle : handles[getMessageTypeFromString(name)]) {
                    if (enabled && !handle.enabled()) {
                        handle.enable();
                        found = true;
                    }

                    else if (!enabled && handle.enabled()) {
                        handle.disable();
                        found = true;
                    }
                }

                if (found) {
                    if (enabled) {
                        log<NUClear::INFO>("Enabled:", name);
                    } else {
                        log<NUClear::INFO>("Disabled:", name);
                    }
                }
            }
        });

        on<Every<1, std::chrono::seconds>, Single, Priority::LOW>().then([this] {
            send(createMessage(Message::PING));
        });

        provideOverview();
        provideDataPoints();
        provideDrawObjects();
        provideSubsumption();
        provideGameController();
        provideLocalisation();
        provideReactionStatistics();
        provideSensors();
        provideVision();

        sendReactionHandles();

        // When we shutdown, close our publisher and our file if we have one
        on<Shutdown>().then([this] {
            // pub.close();

            // Close the file if it exists
            fileEnabled = false;
            outputFile.close();
            // TODO DO THIS
        });
    }

    void NUbugger::sendReactionHandles() {
        Message message = createMessage(Message::REACTION_HANDLES);

        auto* reactionHandles = message.mutable_reaction_handles();

        for (auto& handle : handles) {
            auto* objHandles = reactionHandles->add_handles();
            auto& value = handle.second;
            objHandles->set_type(handle.first);
            objHandles->set_enabled(value.empty() ? true : value.front().enabled());
        }

        send(message);
    }

    void NUbugger::run() {
        // TODO: fix this - still blocks on last recv even if listening = false
        // while (listening) {
        //     // zmq::message_t message;
        //     // sub.recv(&message);

        //     // If our message size is 0, then it is probably our termination message
        //     if (message.size() > 0) {

        //         // Parse our message
        //         Message proto;
        //         proto.ParseFromArray(message.data(), message.size());
        //         recvMessage(proto);
        //     }
        // }
    }

    Message NUbugger::createMessage(Message::Type type, uint filterId) {
        Message message;
        message.set_type(type);
        message.set_filter_id(filterId);
        message.set_utc_timestamp(getUtcTimestamp());
        return message;
    }

    void NUbugger::recvMessage(const Message& message) {
        log<NUClear::DEBUG>("Received message of type:", message.type());
        switch (message.type()) {
            case Message::COMMAND:
                recvCommand(message);
                break;
            case Message::LOOKUP_TABLE:
                recvLookupTable(message);
                break;
            case Message::REACTION_HANDLES:
                recvReactionHandles(message);
                break;
            case Message::CONFIGURATION_STATE:
                recvConfigurationState(message);
                break;
            default:
                return;
        }
    }

    void NUbugger::recvCommand(const Message& message) {
        std::string command = message.command().command();
        log<NUClear::INFO>("Received command:", command);
        if (command == "download_lut") {
            std::shared_ptr<LookUpTable> lut;
            // try {
            //     lut = powerplant.get<LookUpTable>();
            // }
            // catch (NUClear::metaprogramming::NoDataException err) {
            //     log<NUClear::ERROR>("There is no LUT loaded");
            //     return;
            // }

            Message message = createMessage(Message::LOOKUP_TABLE);

            auto* api_lookup_table = message.mutable_lookup_table();

            api_lookup_table->set_table(lut->getData());
            api_lookup_table->set_bits_y(lut->BITS_Y);
            api_lookup_table->set_bits_cb(lut->BITS_CB);
            api_lookup_table->set_bits_cr(lut->BITS_CR);

            send(message);
        } else if (command == "get_configuration_state") {
            sendConfigurationState();
        } else if (command == "get_reaction_handles") {
            sendReactionHandles();
        } else if (command == "get_subsumption") {
            sendSubsumption();
        }
    }

    void NUbugger::recvLookupTable(const Message& message) {
        auto lookuptable = message.lookup_table();
        const std::string& lutData = lookuptable.table();

        log<NUClear::INFO>("Loading LUT");
        std::vector<messages::vision::Colour> data;
        data.reserve(lutData.size());
        for (auto& s : lutData) {
            data.push_back(messages::vision::Colour(s));
        }
        auto lut = std::make_unique<LookUpTable>(lookuptable.bits_y(), lookuptable.bits_cb(), lookuptable.bits_cr(), std::move(data));
        emit<Scope::DIRECT>(std::move(lut));

        if (lookuptable.save()) {
            log<NUClear::INFO>("Saving LUT to file");
            emit<Scope::DIRECT>(std::make_unique<SaveLookUpTable>());
        }
    }

    void NUbugger::recvReactionHandles(const Message& message) {
        // TODO: Fix
        // auto currentConfig = powerplant.get<Configuration<NUbugger>>();

        // auto config = std::make_unique<SaveConfiguration>();
        // config->config = currentConfig->config;

        // for (const auto& command : message.reaction_handles().handles()) {

        //     Message::Type type = command.type();
        //     bool enabled = command.enabled();
        //     std::string key = getStringFromMessageType(type);

        //     std::transform(key.begin(), key.end(), key.begin(), ::tolower);

        //     config->path = CONFIGURATION_PATH;
        //     config->config["reaction_handles"][key] = enabled;
        //     for (auto& handle : handles[type]) {
        //         handle.enable(enabled);
        //     }
        // }

        // emit(std::move(config));
    }

    void NUbugger::kill() {
        listening = false;
    }

    /**
     * This method needs to be used over pub.send as all calls to
     * pub.send need to be synchronized with a concurrency primitive
     * (such as a mutex)
     */
    // void NUbugger::send(zmq::message_t& packet) {
    //     std::lock_guard<std::mutex> lock(networkMutex);
    //     pub.send(packet);
    // }

    void NUbugger::send(Message message) {

        if (networkEnabled) {
            // Make a ZMQ packet and send it
            // size_t messageSize = message.ByteSize();
            // zmq::message_t packet(messageSize + 2);
            // char* dataPtr = static_cast<char*>(packet.data());
            // message.SerializeToArray(dataPtr + 2, messageSize);
            // dataPtr[0] = uint8_t(message.type());
            // dataPtr[1] = uint8_t(message.filter_id());
            // send(packet);
        }
        if (fileEnabled && outputFile) {
            // Lock the file mutex
            std::lock_guard<std::mutex> lock(fileMutex);

            // Get the number of bytes
            uint32_t size = message.ByteSize();

            // Write it to the stream
            outputFile.write(reinterpret_cast<char*>(&size), sizeof(uint32_t));

            // Append the protocol buffer to the file
            message.SerializeToOstream(&outputFile);
        }

    }


    Message::Type NUbugger::getMessageTypeFromString(std::string type_name) {
        std::transform(type_name.begin(), type_name.end(), type_name.begin(), ::toupper);
        Message::Type type;
        if (type_name == "PING") {
            type = Message::PING;
        } else if (type_name == "SENSOR_DATA") {
            type = Message::SENSOR_DATA;
        } else if (type_name == "IMAGE") {
            type = Message::IMAGE;
        } else if (type_name == "CLASSIFIED_IMAGE") {
            type = Message::CLASSIFIED_IMAGE;
        } else if (type_name == "VISION_OBJECT") {
            type = Message::VISION_OBJECT;
        } else if (type_name == "LOCALISATION") {
            type = Message::LOCALISATION;
        } else if (type_name == "DATA_POINT") {
            type = Message::DATA_POINT;
        } else if (type_name == "DRAW_OBJECTS") {
            type = Message::DRAW_OBJECTS;
        } else if (type_name == "REACTION_STATISTICS") {
            type = Message::REACTION_STATISTICS;
        } else if (type_name == "LOOKUP_TABLE") {
            type = Message::LOOKUP_TABLE;
        } else if (type_name == "LOOKUP_TABLE_DIFF") {
            type = Message::LOOKUP_TABLE_DIFF;
        } else if (type_name == "SUBSUMPTION") {
            type = Message::SUBSUMPTION;
        } else if (type_name == "COMMAND") {
            type = Message::COMMAND;
        } else if (type_name == "REACTION_HANDLES") {
            type = Message::REACTION_HANDLES;
        } else if (type_name == "GAME_STATE") {
            type = Message::GAME_STATE;
        } else if (type_name == "CONFIGURATION_STATE") {
            type = Message::CONFIGURATION_STATE;
        } else if (type_name == "BEHAVIOUR") {
            type = Message::BEHAVIOUR;
        } else if (type_name == "OVERVIEW") {
            type = Message::OVERVIEW;
        } else {
            throw new std::runtime_error("NUbugger::getMessageTypeFromString: Invalid message string");
        }
        return type;

    }

    std::string NUbugger::getStringFromMessageType(Message::Type type) {
        switch (type) {
            case Message::PING:                 return "PING";
            case Message::SENSOR_DATA:          return "SENSOR_DATA";
            case Message::IMAGE:                return "IMAGE";
            case Message::CLASSIFIED_IMAGE:     return "CLASSIFIED_IMAGE";
            case Message::VISION_OBJECT:        return "VISION_OBJECT";
            case Message::LOCALISATION:         return "LOCALISATION";
            case Message::DATA_POINT:           return "DATA_POINT";
            case Message::DRAW_OBJECTS:         return "DRAW_OBJECTS";
            case Message::REACTION_STATISTICS:  return "REACTION_STATISTICS";
            case Message::LOOKUP_TABLE:         return "LOOKUP_TABLE";
            case Message::LOOKUP_TABLE_DIFF:    return "LOOKUP_TABLE_DIFF";
            case Message::SUBSUMPTION:          return "SUBSUMPTION";
            case Message::COMMAND:              return "COMMAND";
            case Message::REACTION_HANDLES:     return "REACTION_HANDLES";
            case Message::GAME_STATE:           return "GAME_STATE";
            case Message::CONFIGURATION_STATE:  return "CONFIGURATION_STATE";
            case Message::BEHAVIOUR:            return "BEHAVIOUR";
            case Message::OVERVIEW:             return "OVERVIEW";

            default: throw new std::runtime_error("NUbugger::getStringFromMessageType: Invalid message type");
        }
    }

} // support
} // modules
