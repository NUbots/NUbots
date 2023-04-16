#include "HardwareIO.hpp"

#include <fmt/format.h>

#include "Convert.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::openCR {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
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
            // Broadcast ID - defined to capture any StatusReturn's
            packet_queue[uint8_t(NUgus::ID::BROADCAST)] = std::vector<PacketTypes>();


            // FSRs
            // packet_queue[uint8_t(NUgus::ID::R_FSR)] = std::vector<PacketTypes>();
            // packet_queue[uint8_t(NUgus::ID::L_FSR)] = std::vector<PacketTypes>();


            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                nugus.servo_offset[i]    = config["servos"][i]["offset"].as<Expression>();
                nugus.servo_direction[i] = config["servos"][i]["direction"].as<Expression>();
                servoStates[i].simulated = config["servos"][i]["simulated"].as<bool>();
            }
        });

        on<Startup>().then("HardwareIO Startup", [this] { startup(); });

        on<Shutdown>().then("HardwareIO Shutdown", [this] {
            // log<NUClear::TRACE>("Shutdown");
            // Close our connection to the OpenCR
            if (opencr.connected()) {
                opencr.close();
            }
        });

        on<Watchdog<HardwareIO, 1, std::chrono::seconds>, Sync<HardwareIO>>().then([this] {
            // We haven't received any messages lately
            log<NUClear::WARN>("No packets recently, request servo packets");
            // Send a request for all servo packets
            send_servo_request();
        });

        // When we receive data back from the OpenCR it will arrive here
        // Run a state machine to handle reception of packet header and data
        // If a packet is successfully emitted then we emit a StatusReturn message
        on<IO>(opencr.native_handle(), IO::READ).then([this] {
            // Process the response packet and emit a StatusReturn if applicable
            handle_response();
            // Service the watchdog
            emit<Scope::WATCHDOG>(ServiceWatchdog<HardwareIO>());
        });


        // Single is used to process servos one at a time and increment the counter
        on<Trigger<StatusReturn>, Single, Sync<HardwareIO>>().then([this](const StatusReturn& packet) {
            // Check we can process this packet
            if (packet_queue.find(packet.id) == packet_queue.end()) {
                log<NUClear::WARN>(fmt::format("Recieved packet for unexpected ID {}.", packet.id));
            }
            // Check we're expecting the packet
            else if (packet_queue[packet.id].size() == 0) {
                log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", packet.id));
            }

            // All good, now figure out what the contents of the message are
            else {
                // Pop the front of the packet queue
                auto info = packet_queue[packet.id].front();
                packet_queue[packet.id].erase(packet_queue[packet.id].begin());

                // log<NUClear::DEBUG>(fmt::format(
                //     "Packet ID {}, Contents {}, Data size {}, Remaining in this
                //     queue {} ",
                //     packet.id,
                //     (uint8_t) info,
                //     packet.data.size(),
                //     packet_queue[packet.id].size()));

                // Check for errors
                if (packet.error != StatusReturn::CommandError::NO_ERROR) {
                    log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with error flag", packet.id));
                }
                if (packet.alert) {
                    log<NUClear::WARN>(fmt::format("Recieved packet for ID {} with hardware alert"));
                }

                switch (info) {
                    // Handles OpenCR model and version information
                    case PacketTypes::MODEL_INFORMATION: process_model_information(packet); break;

                    // Handles OpenCR sensor data
                    case PacketTypes::OPENCR_DATA: process_opencr_data(packet); break;

                    // Handles servo data
                    case PacketTypes::SERVO_DATA: process_servo_data(packet); break;

                    // Handles FSR data
                    // case PacketTypes::FSR_DATA: processFSRData(packet); break;

                    // What is this??
                    default: log<NUClear::WARN>("Unknown packet data received"); break;
                }
            }

            // -> Received all servo packets
            // -> Request OpenCR packet
            // -> Received OpenCR packet
            // -> Request servo packets

            // Loop through the array - if any are false, then not all servos received
            bool all_servos_received = true;
            for (const bool s : servo_response) {
                if (!s) {
                    all_servos_received = false;
                    break;
                }
            }

            // If we have all the servos, now we need to send an OpenCR read command
            if (all_servos_received) {
                send_opencr_request();

                // Reset the servo response array
                for (bool& s : servo_response) {
                    s = false;
                }
            }
            // If we have all the OpenCR data, now we need to send servo read commands
            else if (opencr_response) {
                send_servo_request();

                // Reset the OpenCR response flag
                opencr_response = false;
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


                    // log<NUClear::DEBUG>(fmt::format("ServoTarget ID {} to {}", command.id, command.position));

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
    }

}  // namespace module::platform::openCR
