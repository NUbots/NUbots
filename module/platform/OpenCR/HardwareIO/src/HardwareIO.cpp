#include "HardwareIO.hpp"

#include <fmt/format.h>

#include "Convert.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/support/yaml_expression.hpp"

#define DEBUG_ENABLE_BUTTON_SPOOF 1

namespace module::platform::OpenCR {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::StatusReturn;
    using utility::support::Expression;

    using message::platform::ButtonMiddleDown;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Make sure OpenCR is operating at the correct baud rate (based on config params)
            if (opencr.connected()) {
                opencr.close();
            }

            opencr.open(config["opencr"]["device"], config["opencr"]["baud"]);
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
        });

        on<Startup>().then("HardwareIO Startup", [this] {
            startup();

#if DEBUG_ENABLE_BUTTON_SPOOF
            // trigger scriptrunner
            log<NUClear::INFO>("Simulating Middle Button Down in 3 seconds");
            emit<Scope::DELAY>(std::make_unique<ButtonMiddleDown>(), std::chrono::seconds(3));
#endif  // DEBUG_ENABLE_BUTTON_SPOOF
        });

        on<Shutdown>().then("HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (opencr.connected()) {
                opencr.close();
            }
        });

        on<Watchdog<HardwareIO, 2, std::chrono::seconds>, Sync<HardwareIO>>().then([this] {
            // First, check if this is the model info packet, because if it is, the system
            // startup failed, and we need to re-trigger it.
            if (opencr_waiting() && packet_queue[NUgus::ID::OPENCR].front() == PacketTypes::MODEL_INFORMATION) {
                log<NUClear::WARN>(fmt::format("OpenCR model information not recieved, restarting system"));
                // Clear all packet queues just in case
                queue_clear_all();
                // Restart the system and exit the watchdog
                startup();
                return;
            }

            // Check what the hangup was
            bool packet_dropped = false;
            // The result of the assignment is 0 (NUgus::ID::NO_ID) if we aren't waiting on
            // any packets, otherwise is the nonzero ID of the timed out device
            for (NUgus::ID dropout_id; (dropout_id = queue_item_waiting()) != NUgus::ID::NO_ID;) {
                // delete the packet we're waiting on
                packet_queue[dropout_id].erase(packet_queue[dropout_id].begin());
                log<NUClear::WARN>(fmt::format("Dropped packet from ID {}", dropout_id));
                // set flag
                packet_dropped = true;
            }


            // Send a request for all servo packets, only if there were packets dropped
            // In case the system stops for some other reason, we don't want the watchdog
            // to make it automatically restart
            if (packet_dropped) {
                log<NUClear::WARN>("Requesting servo packets to restart system");
                send_servo_request();
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
                log<NUClear::WARN>(fmt::format("Recieved packet for unexpected ID {}.", packet.id));
                return;
            }

            // Check we're expecting the packet
            if (packet_queue[packet_id].empty()) {
                log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", packet_id));
                return;
            }

            /* All good now */

            // Service the watchdog because we recieved a valid packet
            emit<Scope::WATCHDOG>(ServiceWatchdog<HardwareIO>());

            // Pop the front of the packet queue
            auto& info = packet_queue[packet_id].front();
            packet_queue[packet_id].erase(packet_queue[packet_id].begin());

            // Check for packet errors
            if (packet.error != StatusReturn::CommandError::NO_ERROR) {
                log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with error flag", packet_id));
            }
            if (packet.alert) {
                log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with hardware alert", packet_id));
            }

            /// @brief handle incoming packets, and send next request if all packets were handled
            // -> Recieved model information packet
            //    -> Trigger first servo request
            // -> Received all servo packets
            //    -> Request OpenCR packet
            // -> Received OpenCR packet
            //    -> Request servo packets
            switch (info) {
                // Handles OpenCR model and version information
                case PacketTypes::MODEL_INFORMATION:
                    // call packet handler
                    process_model_information(packet);

                    // check if we recieved the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<NUClear::TRACE>("Initial data received, kickstarting system");

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
                    // call packet handler
                    process_opencr_data(packet);

                    // check if we recieved the final packet we are expecting
                    if (queue_item_waiting() == NUgus::ID::NO_ID) {
                        log<NUClear::TRACE>("OpenCR data received, requesting servo data");
                        send_servo_request();
                    }

                    break;

                // Handles servo data
                case PacketTypes::SERVO_DATA:
                    // call packet handler
                    process_servo_data(packet);

                    // check if we recieved the final packet we are expecting
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
    }

}  // namespace module::platform::OpenCR
