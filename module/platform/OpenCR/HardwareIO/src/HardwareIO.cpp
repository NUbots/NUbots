/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "HardwareIO.hpp"

#include <fmt/format.h>

#include "Convert.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::OpenCR {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::StatusReturn;
    using utility::input::ServoID;
    using utility::support::Expression;

    using message::output::Buzzer;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {


        packet_watchdog =
            on<Watchdog<PacketWatchdog, 20, std::chrono::milliseconds>, Sync<PacketWatchdog>>()
                .then([this] {
                    // This is a hacky fix because the watchdog is not disabled quickly enough at the beginning. This
                    // may be related to the out of order packets with Sync within NUClear. This should be fixed in a
                    // later version of NUClear.
                    if (model_watchdog.enabled()) {
                        log<NUClear::WARN>(
                            "Packet watchdog cannot be enabled while model watchdog is enabled. You may see this "
                            "warning at the start of the program. This is expected as the watchdog reaction may still "
                            "be disabling.");
                        packet_watchdog.disable();
                        return;
                    }

                    // Check what the hangup was

                    int num_packets_dropped        = 0;                 // keep track of how many we dropped
                    NUgus::ID first_dropped_packet = NUgus::ID::NO_ID;  // keep track in case we have a chain

                    // The result of the assignment is 0 (NUgus::ID::NO_ID) if we aren't waiting on
                    // any packets, otherwise is the nonzero ID of the timed out device
                    for (NUgus::ID dropout_id; (dropout_id = queue_item_waiting()) != NUgus::ID::NO_ID;) {

                        // Delete the packet we're waiting on
                        packet_queue[dropout_id].erase(packet_queue[dropout_id].begin());

                        // notify with ID and servo name
                        log<NUClear::WARN>(fmt::format("Dropped packet from ID {} ({})",
                                                       int(dropout_id),
                                                       nugus.device_name(dropout_id)));

                        // if this is the first packet, set our flag
                        if (num_packets_dropped == 0) {
                            first_dropped_packet = dropout_id;
                        }

                        // increment our counter
                        num_packets_dropped++;
                    }

                    // if this is the first packet then send a warning
                    if (num_packets_dropped > 1) {
                        log<NUClear::WARN>(
                            fmt::format("NOTE: A dropped response packet by a dynamixel device in a SYNC READ/WRITE "
                                        "chain will cause all later packets (of higher ID) to be dropped. Consider "
                                        "checking cables for ID {} ({})",
                                        int(first_dropped_packet),
                                        nugus.device_name(first_dropped_packet)));
                    }

                    // Send a request for all servo packets, only if there were packets dropped
                    // In case the system stops for some other reason, we don't want the watchdog
                    // to make it automatically restart
                    if (num_packets_dropped > 0) {
                        log<NUClear::WARN>("Requesting servo packets to restart system");
                        send_servo_request();
                    }
                })
                .disable();

        model_watchdog =
            on<Watchdog<ModelWatchdog, 500, std::chrono::milliseconds>, Sync<ModelWatchdog>>()
                .then([this] {
                    log<NUClear::WARN>(fmt::format("OpenCR model information not received, restarting system"));
                    // Clear all packet queues just in case
                    queue_clear_all();
                    // Restart the system and exit the watchdog
                    startup();
                    return;
                })
                .enable();

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            opencr      = utility::io::uart(config["opencr"]["device"], config["opencr"]["baud"]);
            byte_wait   = config["opencr"]["byte_wait"];
            packet_wait = config["opencr"]["packet_wait"];

            // Initialise packet_queue map
            // OpenCR
            packet_queue[NUgus::ID::OPENCR] = std::vector<PacketTypes>();

            // Within this codebase, we consider servos to be 0 indexed
            // However, when we receive a packet, they are 1 indexed
            // The packet queue and servo_ids function handles this
            for (const auto& id : nugus.servo_ids()) {
                packet_queue[NUgus::ID(id)] = std::vector<PacketTypes>();
            }

            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                nugus.servo_offset[i]     = config["servos"][i]["offset"].as<Expression>();
                nugus.servo_direction[i]  = config["servos"][i]["direction"].as<Expression>();
                servo_states[i].simulated = config["servos"][i]["simulated"].as<bool>();
            }

            // populate alarm config levels
            cfg.alarms.temperature.level            = config["alarms"]["temperature"]["level"].as<float>();
            cfg.alarms.temperature.buzzer_frequency = config["alarms"]["temperature"]["buzzer_frequency"].as<float>();

            // populate battery config levels
            battery_state.charged_voltage = config["battery"]["charged_voltage"].as<float>();
            battery_state.nominal_voltage = config["battery"]["nominal_voltage"].as<float>();
            battery_state.flat_voltage    = config["battery"]["flat_voltage"].as<float>();
        });

        on<Startup>().then("HardwareIO Startup", [this] {
            // The first thing to do is get the model information
            // The model watchdog is started, which has a longer time than the packet watchdog
            // The packet watchdog is disabled until we start the main loop
            model_watchdog.enable();
            packet_watchdog.disable();

            // The startup function sets up the subcontroller state
            startup();
        });

        on<Shutdown>().then("HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (opencr.connected()) {
                opencr.close();
            }
        });

        // When we receive data back from the OpenCR it will arrive here
        // Run a state machine to handle reception of packet header and data
        // If a packet is successfully emitted then we emit a StatusReturn message
        on<IO>(opencr.native_handle(), IO::READ).then([this] {
            // Process the response packet and emit a StatusReturn if applicable
            handle_response();
        });

        on<Trigger<StatusReturn>, Sync<HardwareIO>>().then([this](const StatusReturn& packet) {
            const NUgus::ID packet_id = NUgus::ID(packet.id);
            /* Error handling */

            // Check we can process this packet
            if (packet_queue.find(packet_id) == packet_queue.end()) {
                log<NUClear::WARN>(fmt::format("received packet for unexpected ID {}.", packet.id));
                return;
            }

            // Check we're expecting the packet
            if (packet_queue[packet_id].empty()) {
                log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", int(packet_id)));
                return;
            }

            /* All good now */

            // Pop the front of the packet queue
            auto& info = packet_queue[packet_id].front();
            packet_queue[packet_id].erase(packet_queue[packet_id].begin());

            /// @brief handle incoming packets, and send next request if all packets were handled
            // -> received model information packet
            //    -> Trigger first servo request
            // -> Received all servo packets
            //    -> Request OpenCR packet
            // -> Received OpenCR packet
            //    -> Request servo packets
            switch (info) {
                // Handles OpenCR model and version information
                case PacketTypes::MODEL_INFORMATION:
                    emit<Scope::WATCHDOG>(ServiceWatchdog<ModelWatchdog>());
                    // call packet handler
                    process_model_information(packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<NUClear::TRACE>("Initial data received, kickstarting system");

                        // Stop the model watchdog since we have it now
                        // Start the packet watchdog since the main loop is now starting
                        model_watchdog.disable();
                        model_watchdog.unbind();
                        log<NUClear::WARN>("Packet watchdog enabled");
                        packet_watchdog.enable();

                        // At the start, we want to query the motors so we can store their state internally
                        // This will start the loop of reading and writing to the servos and opencr
                        for (const auto& id : nugus.servo_ids()) {
                            packet_queue[NUgus::ID(id)].push_back(PacketTypes::SERVO_DATA);
                        }
                        opencr.write(dynamixel::v2::SyncReadCommand<20>(uint16_t(AddressBook::SERVO_READ),
                                                                        sizeof(DynamixelServoReadData),
                                                                        nugus.servo_ids()));
                    }

                    break;

                // Handles OpenCR sensor data
                case PacketTypes::OPENCR_DATA:
                    emit<Scope::WATCHDOG>(ServiceWatchdog<PacketWatchdog>());
                    // call packet handler
                    process_opencr_data(packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<NUClear::TRACE>("OpenCR data received, requesting servo data");
                        send_servo_request();
                    }

                    break;

                // Handles servo data
                case PacketTypes::SERVO_DATA:
                    emit<Scope::WATCHDOG>(ServiceWatchdog<PacketWatchdog>());
                    // call packet handler
                    process_servo_data(packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<NUClear::TRACE>("All servos received, requesting OpenCR data");
                        send_opencr_request();
                    }

                    break;

                default: log<NUClear::WARN>("Unknown packet data received"); break;
            }
        });

        // REACTIONS FOR RECEIVING HARDWARE REQUESTS FROM THE SYSTEM

        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            // Loop through each of our commands and update servo state information accordingly
            for (const auto& command : commands.targets) {
                // Desired time to reach the goal position (in milliseconds)
                NUClear::clock::duration duration = command.time - NUClear::clock::now();
                float time_span = float(duration.count()) / float(NUClear::clock::period::den) * 1000.0f;
                // Ensure the time span is positive
                time_span = std::max(time_span, 0.0f);

                // Update our internal state
                if (servo_states[command.id].torque != command.torque
                    || servo_states[command.id].position_p_gain != command.gain
                    || servo_states[command.id].position_i_gain != command.gain * 0
                    || servo_states[command.id].position_d_gain != command.gain * 0
                    || servo_states[command.id].goal_position != command.position
                    || servo_states[command.id].profile_velocity != time_span) {

                    servo_states[command.id].dirty = true;

                    servo_states[command.id].torque = command.torque;

                    servo_states[command.id].position_p_gain = command.gain;
                    servo_states[command.id].position_i_gain = command.gain * 0;
                    servo_states[command.id].position_d_gain = command.gain * 0;

                    servo_states[command.id].goal_position = command.position;
                    // Drive Mode is Time-Based, so we need to set the profile velocity to the time (in milliseconds) we
                    // want to take to reach the goal position
                    servo_states[command.id].profile_velocity = time_span;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(command_list));
        });

        // If we get a head_led command then write it
        on<Trigger<RawSensors::HeadLED>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            opencr_state.head_led = led.RGB;
            opencr_state.dirty    = true;
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>>().then([this](const RawSensors::EyeLED& /*led*/) {
            // Update our internal state
            // OpenCR can only use 1 RGB LED
        });

        // If we get an LED panel command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            opencr_state.led_panel.led2 = led.led2;
            opencr_state.led_panel.led3 = led.led3;
            opencr_state.led_panel.led4 = led.led4;
            opencr_state.dirty          = true;
        });

        on<Trigger<Buzzer>>().then([this](const Buzzer& buzzer_msg) {
            log<NUClear::DEBUG>("Received Buzzer message");
            // Fill the necessary field within the opencr_state struct
            opencr_state.buzzer = buzzer_msg.frequency;
            opencr_state.dirty  = true;
        });
    }

}  // namespace module::platform::OpenCR
