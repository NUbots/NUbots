#include "HardwareIO.hpp"

#include <fmt/format.h>

#include "Convert.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::openCR {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::ServoLED;
    using message::platform::StatusReturn;
    using utility::support::Expression;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {

        on<Configuration>("HardwareIO_OpenCR.yaml").then([this](const Configuration& config) {
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
            packet_queue[uint8_t(NUgus::ID::OPENCR)] = std::vector<PacketTypes>();
            // Servos - NOT zero indexed. First servo is ID 1.
            // Required because response packets have Dynamixel ID
            for (auto& id : nugus.servo_ids()) {
                packet_queue[id] = std::vector<PacketTypes>();
            }


            // FSRs
            // packet_queue[uint8_t(NUgus::ID::R_FSR)] = std::vector<PacketTypes>();
            // packet_queue[uint8_t(NUgus::ID::L_FSR)] = std::vector<PacketTypes>();


            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                nugus.servo_offset[i]    = config["servos"][i]["offset"].as<Expression>();
                nugus.servo_direction[i] = config["servos"][i]["direction"].as<Expression>();
                servoStates[i].simulated = config["servos"][i]["simulated"].as<bool>();
            }
        });

        on<Startup>().then("HardwareIO Startup", [this] {
            startup();
            log<NUClear::DEBUG>("HardwareIO started");
        });

        on<Shutdown>().then("HardwareIO Shutdown", [this] {
            // log<NUClear::TRACE>("Shutdown");
            // Close our connection to the OpenCR
            if (opencr.connected()) {
                opencr.close();
            }
        });

        on<Watchdog<HardwareIO, 2, std::chrono::seconds>, Sync<HardwareIO>>().then([this] {
            // First, check if this is the model info packet, because if it is, the system
            // startup failed, and we need to re-trigger it.
            if (opencr_waiting()
                && packet_queue[uint8_t(NUgus::ID::OPENCR)].front() == PacketTypes::MODEL_INFORMATION) {
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
            while (uint8_t dropout_id = queue_item_waiting()) {
                // delete the packet we're waiting on
                packet_queue[dropout_id].erase(packet_queue[dropout_id].begin());
                log<NUClear::WARN>(fmt::format("Dropped packet from ID {}", dropout_id));
                // set flag
                packet_dropped = true;
            }

            // Send a request for all servo packets, only if there were packets dropped
            // In case the system stops for some other reason, we don't want the watchdog
            // to make it automaticlaly restart
            if (packet_dropped) {
                log<NUClear::WARN>("Requesting servo packets to restart system");
                send_servo_request();
            }
        });

        // When we receive data back from the OpenCR it will arrive here
        // Run a state machine to handle reception of packet header and data
        // If a packet is successfully emitted then we emit a StatusReturn message
        on<IO>(opencr.native_handle(), IO::READ).then([this] {
            // log<NUClear::WARN>("Received data");
            // Process the response packet and emit a StatusReturn if applicable
            handle_response();
        });

        on<Trigger<StatusReturn>, Sync<HardwareIO>>().then([this](const StatusReturn& packet) {
            /* Error handling */

            // Check we can process this packet
            if (packet_queue.find(packet.id) == packet_queue.end()) {
                log<NUClear::WARN>(fmt::format("Recieved packet for unexpected ID {}.", packet.id));
                return;
            }

            // Check we're expecting the packet
            if (packet_queue[packet.id].size() == 0) {
                log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", packet.id));
                return;
            }

            /* All good now */

            // Service the watchdog because we recieved a valid packet
            emit<Scope::WATCHDOG>(ServiceWatchdog<HardwareIO>());

            // Pop the front of the packet queue
            auto& info = packet_queue[packet.id].front();
            packet_queue[packet.id].erase(packet_queue[packet.id].begin());

            /*
            log<NUClear::WARN>(fmt::format(
                "Packet ID {}, Contents {}, Data size {}, Remaining in this
                queue {} ",
                packet.id,
                (uint8_t) info,
                packet.data.size(),
                packet_queue[packet.id].size()));
            //*/

            // Check for packet errors
            /// @todo Do we want to handle packets differently if they have errors?
            if (packet.error != StatusReturn::CommandError::NO_ERROR) {
                log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with error flag", packet.id));
            }
            if (packet.alert) {
                log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with hardware alert"));
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
                    if (!queue_item_waiting()) {
                        log<NUClear::INFO>("Initial data received, kickstarting system");
                        send_servo_request();
                    }
                    break;

                // Handles OpenCR sensor data
                case PacketTypes::OPENCR_DATA:
                    // call packet handler
                    process_opencr_data(packet);
                    // check if we recieved the final packet we are expecting
                    if (!queue_item_waiting()) {
                        log<NUClear::INFO>("OpenCR data received, requesting servo data");
                        send_servo_request();
                    }
                    break;

                // Handles servo data
                case PacketTypes::SERVO_DATA:
                    // call packet handler
                    process_servo_data(packet);
                    // check if we recieved the final packet we are expecting
                    if (!queue_item_waiting()) {
                        log<NUClear::INFO>("All servos received, requesting OpenCR data");
                        send_opencr_request();
                    }
                    break;

                // Handles FSR data
                // case PacketTypes::FSR_DATA: processFSRData(packet); break;

                // What is this??
                default: log<NUClear::WARN>("Unknown packet data received"); break;
            }
        });

        // REACTIONS FOR RECEIVING HARDWARE REQUESTS FROM THE SYSTEM

        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            // log<NUClear::TRACE>("ServoTargets START");
            // Loop through each of our commands and update servo state information accordingly
            for (const auto& command : commands.targets) {
                float diff =
                    utility::math::angle::difference(command.position, servoStates[command.id].presentPosition);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }
                else {
                    speed = 0;
                }

                // Update our internal state
                if (servoStates[command.id].velocityPGain != command.gain
                    || servoStates[command.id].velocityIGain != command.gain * 0
                    || servoStates[command.id].velocityDGain != command.gain * 0
                    || servoStates[command.id].goalVelocity != speed
                    || servoStates[command.id].goalPosition != command.position) {


                    log<NUClear::ERROR>(fmt::format("ServoTarget ID {} to {}", command.id, command.position));

                    servoStates[command.id].dirty = true;

                    servoStates[command.id].velocityPGain = command.gain;
                    servoStates[command.id].velocityIGain = command.gain * 0;
                    servoStates[command.id].velocityDGain = command.gain * 0;

                    servoStates[command.id].goalVelocity = speed;
                    servoStates[command.id].goalPosition = command.position;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });

        // If we get a HeadLED command then write it
        on<Trigger<RawSensors::HeadLED>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            opencrState.headLED = led.RGB;
            opencrState.dirty   = true;
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>>().then([this](const RawSensors::EyeLED& /*led*/) {
            // Update our internal state
            // OpenCR can only use 1 RGB LED
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            opencrState.ledPanel.led2 = led.led2;
            opencrState.ledPanel.led3 = led.led3;
            opencrState.ledPanel.led4 = led.led4;
            opencrState.dirty         = true;
        });

        // If we get a ServoLED command then trigger it
        // Debugging so set priority HIGH
        on<Trigger<ServoLED>, Sync<HardwareIO>, Priority::HIGH>().then([this](const ServoLED& target) {
            // check it's a valid ID
            if (target.id < 1 || target.id > 20) {
                log<NUClear::WARN>(fmt::format("Bad ID {} for ServoLED command", target.id));
                return;
            }
            // create the write command and send it
            // the target state is casted from a bool to a byte (0x00 or 0x01)
            opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(target.id),
                                                              uint16_t(DynamixelServo::Address::LED),
                                                              uint8_t(target.state)));
            // logging
            log<NUClear::DEBUG>(fmt::format("Turned servo {} LED {}", target.id, target.state ? "on" : "off"));
        });

        /// @brief trigger the LEDs of all servos
        /// @details this is basically for debugging only
        on<Every<10, std::chrono::seconds>>().then([this] {
            // control vals
            int msBetweenServos = 100;
            int msBeforeOff     = 1000;
            // create triggers for every servo
            for (auto& id : nugus.servo_ids()) {
                // create messages
                auto onTarget  = std::make_unique<ServoLED>();
                auto offTarget = std::make_unique<ServoLED>();
                // fill messages
                onTarget->id     = uint32_t(id);
                offTarget->id    = uint32_t(id);
                onTarget->state  = true;
                offTarget->state = false;
                // emit on with delay
                int onDelay = (id - 1) * msBetweenServos;
                emit<Scope::DELAY>(std::move(onTarget), std::chrono::milliseconds(onDelay));
                // emit off with additional delay
                int offDelay = (((20 - 1) + (id - 1)) * msBetweenServos) + msBeforeOff;
                emit<Scope::DELAY>(std::move(offTarget), std::chrono::milliseconds(offDelay));
            }
        });
    }

}  // namespace module::platform::openCR
