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

#include "message/actuation/ServoOffsets.hpp"
#include "message/actuation/ServoTarget.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::OpenCR {

    using extension::Configuration;
    using message::actuation::ServoOffsets;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::StatusReturn;
    using utility::input::ServoID;
    using utility::support::Expression;

    using message::output::Buzzer;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {

        on<Configuration>("OpenCR.yaml").then([this](const Configuration& config) {
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

            // populate alarm config levels
            cfg.alarms.temperature.level            = config["alarms"]["temperature"]["level"].as<float>();
            cfg.alarms.temperature.buzzer_frequency = config["alarms"]["temperature"]["buzzer_frequency"].as<float>();

            // populate battery config levels
            battery_state.charged_voltage = config["battery"]["charged_voltage"].as<float>();
            battery_state.nominal_voltage = config["battery"]["nominal_voltage"].as<float>();
            battery_state.flat_voltage    = config["battery"]["flat_voltage"].as<float>();

            // Gyro offset
            cfg.gyro_offset = config["gyro_offset"].as<Expression>();
        });

        on<Trigger<ServoOffsets>>().then([this](const ServoOffsets& offsets) {
            // Update the servo offsets if the config changes
            for (size_t i = 0; i < offsets.offsets.size(); ++i) {
                nugus.servo_offset[i]     = offsets.offsets[i].offset;
                nugus.servo_direction[i]  = offsets.offsets[i].direction;
                servo_states[i].simulated = offsets.offsets[i].simulated;
            }
        });

        on<Startup, With<ServoOffsets>>().then("HardwareIO Startup", [this](const ServoOffsets& offsets) {
            // The first thing to do is get the model information
            // The model watchdog is started, which has a longer time than the packet watchdog
            // The packet watchdog is disabled until we start the main loop
            model_watchdog = create_model_watchdog();

            // The startup function sets up the subcontroller state
            startup();

            // Set the servo offsets
            for (size_t i = 0; i < offsets.offsets.size(); ++i) {
                nugus.servo_offset[i]     = offsets.offsets[i].offset;
                nugus.servo_direction[i]  = offsets.offsets[i].direction;
                servo_states[i].simulated = offsets.offsets[i].simulated;
            }
        });

        // When we receive data back from the OpenCR it will arrive here
        // Run a state machine to handle reception of packet header and data
        // If a packet is successfully emitted then we emit a StatusReturn message
        on<IO, Pool<HardwareIO>>(opencr.native_handle(), IO::READ).then([this] {
            // Process the response packet and emit a StatusReturn if applicable
            handle_response();
        });

        on<Trigger<StatusReturn>, Pool<HardwareIO>>().then([this](const StatusReturn& packet) {
            const NUgus::ID packet_id = NUgus::ID(packet.id);
            /* Error handling */

            // Check we can process this packet
            if (packet_queue.find(packet_id) == packet_queue.end()) {
                log<WARN>(fmt::format("received packet for unexpected ID {}.", packet.id));
                return;
            }

            // Check we're expecting the packet
            if (packet_queue[packet_id].empty()) {
                log<WARN>(fmt::format("Unexpected packet data received for ID {}.", int(packet_id)));
                return;
            }

            // Prepare for unstuffing, start copying fields from the original packet
            StatusReturn unstuffed_packet;

            unstuffed_packet.magic       = packet.magic;
            unstuffed_packet.id          = packet.id;
            unstuffed_packet.length      = packet.length;
            unstuffed_packet.instruction = packet.instruction;
            unstuffed_packet.error       = packet.error;

            // We handle the unstuffing here so that we ensure that handle_response() only cares about getting full
            // packets It is also easier to debug this stuff here if it ever goes wrong again
            std::vector<uint8_t> unstuffed_data;
            UnstuffingState unstuffing_state = UnstuffingState::DATA;

            for (const auto& byte : packet.data) {
                switch (unstuffing_state) {
                    case DATA:
                        unstuffing_state = byte == 0xFF ? UnstuffingState::UNSTUFF_1 : UnstuffingState::DATA;
                        unstuffed_data.push_back(byte);
                        break;

                    case UNSTUFF_1:
                        unstuffing_state = byte == 0xFF ? UnstuffingState::UNSTUFF_2 : UnstuffingState::DATA;
                        unstuffed_data.push_back(byte);
                        break;

                    case UNSTUFF_2:
                        // If we see 0xFD, then we have a stuffed byte but we also have to account for long trailing
                        // 0xFF's e.g. ff ff ff fd fd will not be unstuffed if we dont do another check on the 3rd ff
                        unstuffing_state = byte == 0xFD
                                               ? UnstuffingState::UNSTUFF_3
                                               : (byte == 0xFF ? UnstuffingState::UNSTUFF_2 : UnstuffingState::DATA);
                        unstuffed_data.push_back(byte);
                        break;

                    case UNSTUFF_3:
                        if (byte != 0xFD) {
                            // Go back to data and ignore the last 0xFD
                            unstuffed_data.push_back(byte);
                        }
                        else {
                            unstuffed_packet.length--;
                        }
                        unstuffing_state = UnstuffingState::DATA;
                        break;
                }
            }

            unstuffed_packet.data      = unstuffed_data;
            unstuffed_packet.checksum  = packet.checksum;
            unstuffed_packet.timestamp = packet.timestamp;

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
                    process_model_information(unstuffed_packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<TRACE>("Initial data received, kickstarting system");

                        // Stop the model watchdog since we have it now
                        // Start the packet watchdog since the main loop is now starting
                        model_watchdog.disable();
                        model_watchdog.unbind();

                        log<INFO>("Packet watchdog enabled");

                        packet_watchdog = create_packet_watchdog();

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
                    process_opencr_data(unstuffed_packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<TRACE>("OpenCR data received, requesting servo data");
                        send_servo_request();
                    }

                    break;

                // Handles servo data
                case PacketTypes::SERVO_DATA:
                    emit<Scope::WATCHDOG>(ServiceWatchdog<PacketWatchdog>());
                    // call packet handler
                    process_servo_data(unstuffed_packet);

                    // check if we received the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<TRACE>("All servos received, requesting OpenCR data");
                        send_opencr_request();
                    }

                    break;

                default: log<WARN>("Unknown packet data received"); break;
            }
        });

        // REACTIONS FOR RECEIVING HARDWARE REQUESTS FROM THE SYSTEM

        on<Trigger<ServoTargets>, Pool<HardwareIO>>().then([this](const ServoTargets& commands) {
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

        on<Trigger<ServoTarget>, Pool<HardwareIO>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::INLINE>(std::move(command_list));
        });

        // If we get a head_led command then write it
        on<Trigger<RawSensors::HeadLED>, Pool<HardwareIO>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            opencr_state.head_led = led.RGB;
            opencr_state.dirty    = true;
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>, Pool<HardwareIO>>().then([this](const RawSensors::EyeLED& /*led*/) {
            // Update our internal state
            // OpenCR can only use 1 RGB LED
        });

        // If we get an LED panel command then write it
        on<Trigger<RawSensors::LEDPanel>, Pool<HardwareIO>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            opencr_state.led_panel.led2 = led.led2;
            opencr_state.led_panel.led3 = led.led3;
            opencr_state.led_panel.led4 = led.led4;
            opencr_state.dirty          = true;
        });

        on<Trigger<Buzzer>, Pool<HardwareIO>>().then([this](const Buzzer& buzzer_msg) {
            log<DEBUG>("Received Buzzer message");
            // Fill the necessary field within the opencr_state struct
            opencr_state.buzzer = buzzer_msg.frequency;
            opencr_state.dirty  = true;
        });
    }

}  // namespace module::platform::OpenCR
