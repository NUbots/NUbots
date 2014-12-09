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

#include <zmq.hpp>

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
    using messages::support::SaveConfiguration;
    using messages::support::nubugger::proto::Message;

    using messages::vision::LookUpTable;
    using messages::vision::SaveLookUpTable;
    using messages::vision::Colour;

    using utility::time::getUtcTimestamp;

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB)
        , sub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_SUB) {

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&NUbugger::run), this), std::bind(std::mem_fn(&NUbugger::kill), this)));

        on<Trigger<Configuration<NUbugger>>>([this] (const Configuration<NUbugger>& config) {

            // TODO if network disables then we should unbind

            // TODO if the file disables we should close the file

            // TODO if the file location is different, close the file and open a new one

            // If we are using the network
            if(config["output"]["network"]["enabled"].as<bool>()) {

                uint newPubPort = config["output"]["network"]["pub_port"].as<uint>();
                uint newSubPort = config["output"]["network"]["sub_port"].as<uint>();

                if (newPubPort != pubPort) {
                    if (networkEnabled) {
                        pub.unbind(("tcp://*:" + std::to_string(pubPort)).c_str());
                    }
                    pubPort = newPubPort;
                    pub.bind(("tcp://*:" + std::to_string(pubPort)).c_str());
                }

                if (newSubPort != subPort) {
                    if (networkEnabled) {
                        sub.unbind(("tcp://*:" + std::to_string(subPort)).c_str());
                    }
                    subPort = newSubPort;
                    sub.bind(("tcp://*:" + std::to_string(subPort)).c_str());
                }

                // Set our high water mark
                int hwm = config["output"]["network"]["high_water_mark"].as<int>();
                pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
                sub.setsockopt(ZMQ_SUBSCRIBE, 0, 0);

                networkEnabled = true;
            }
            // If we were enabled and now we are not
            else if(networkEnabled && !config["output"]["network"]["enabled"].as<bool>()) {
                // Unbind the network when we disable
                pub.unbind(("tcp://*:" + std::to_string(pubPort)).c_str());
                sub.unbind(("tcp://*:" + std::to_string(subPort)).c_str());

                networkEnabled = false;
            }

            // If we are using files and haven't set one up yet
            if(!fileEnabled && config["output"]["file"]["enabled"].as<bool>()) {

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
            else if(fileEnabled && !config["output"]["file"]["enabled"].as<bool>()) {

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
                for (auto& handle : handles[name]) {
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
                        log("Enabled:", name);
                    } else {
                        log("Disabled:", name);
                    }
                }
            }
        });

        on<Trigger<Every<1, std::chrono::seconds>>>([this] (const time_t&) {
            Message message;
            message.set_type(Message::PING);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());
            send(message);
        });

        provideDataPoints();
        provideDrawObjects();
        provideBehaviour();
        provideGameController();
        provideLocalisation();
        provideReactionStatistics();
        provideSensors();
        provideVision();

        // When we shutdown, close our publisher and our file if we have one
        on<Trigger<Shutdown>>([this](const Shutdown&) {
            pub.close();

            // Close the file if it exists
            fileEnabled = false;
            outputFile.close();
            // TODO DO THIS
        });
    }

    void NUbugger::run() {
        // TODO: fix this - still blocks on last recv even if listening = false
        while (listening) {
            zmq::message_t message;
            sub.recv(&message);

            // If our message size is 0, then it is probably our termination message
            if (message.size() > 0) {

                // Parse our message
                Message proto;
                proto.ParseFromArray(message.data(), message.size());
                recvMessage(proto);
            }
        }
    }

    void NUbugger::recvMessage(const Message& message) {
        log("Received message of type:", message.type());
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
            default:
                return;
        }
    }

    void NUbugger::recvCommand(const Message& message) {
        std::string command = message.command().command();
        log("Received command:", command);
        if (command == "download_lut") {
            std::shared_ptr<LookUpTable> lut;
            try {
                lut = powerplant.get<LookUpTable>();
            }
            catch (NUClear::metaprogramming::NoDataException err) {
                log("There is no LUT loaded");
                return;
            }

            Message message;

            message.set_type(Message::LOOKUP_TABLE);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* api_lookup_table = message.mutable_lookup_table();

            api_lookup_table->set_table(lut->getData());

            send(message);
        } else if (command == "get_configuration_state") {
            sendConfigurationState();
        }
    }

    void NUbugger::recvLookupTable(const Message& message) {
        auto lookuptable = message.lookup_table();
        const std::string& lutData = lookuptable.table();

        log("Loading LUT");
        std::vector<messages::vision::Colour> data;
        data.reserve(lutData.size());
        for (auto& s : lutData) {
            data.push_back(messages::vision::Colour(s));
        }
        auto lut = std::make_unique<LookUpTable>(lookuptable.bits_y(), lookuptable.bits_cb(), lookuptable.bits_cr(), std::move(data));
        emit<Scope::DIRECT>(std::move(lut));

        if (lookuptable.save()) {
            log("Saving LUT to file");
            emit<Scope::DIRECT>(std::make_unique<SaveLookUpTable>());
        }
    }

    void NUbugger::recvReactionHandles(const Message& message) {

        auto currentConfig = powerplant.get<Configuration<NUbugger>>();

        auto config = std::make_unique<SaveConfiguration>();
        config->config = currentConfig->config;

        for (const auto& command : message.reaction_handles().handles()) {

            std::string name = command.name();
            bool enabled = command.enabled();

            config->path = CONFIGURATION_PATH;
            config->config["reaction_handles"][name] = enabled;
        }

        emit(std::move(config));
    }

    void NUbugger::kill() {
        listening = false;
    }

    /**
     * This method needs to be used over pub.send as all calls to
     * pub.send need to be synchronized with a concurrency primitive
     * (such as a mutex)
     */
    void NUbugger::send(zmq::message_t& packet) {
        std::lock_guard<std::mutex> lock(networkMutex);
        pub.send(packet);
    }

    void NUbugger::send(Message message) {

        if(networkEnabled) {
            // Make a ZMQ packet and send it
            size_t messageSize = message.ByteSize();
            zmq::message_t packet(messageSize + 2);
            char* dataPtr = static_cast<char*>(packet.data());
            message.SerializeToArray(dataPtr + 2, messageSize);
            dataPtr[0] = uint8_t(message.type());
            dataPtr[1] = uint8_t(message.filter_id());
            send(packet);
        }
        if(fileEnabled && outputFile) {
            // Append the number of bytes to the file (so we can re-read it)
            outputFile << message.ByteSize();
            // Append the protocol buffer to the file
            message.SerializeToOstream(&outputFile);
        }

    }

} // support
} // modules
