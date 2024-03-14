#include "HardwareIO.hpp"

#include <cmath>
#include <fmt/format.h>
#include <sstream>

#include "util/SIProcessor.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"

namespace module::platform::NUsense {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), nusense() {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HardwareIO.yaml
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.nusense.port = config["nusense"]["port"].as<std::string>();
            cfg.nusense.baud = config["nusense"]["baud"].as<unsigned int>();

            nusense = utility::io::uart(cfg.nusense.port, cfg.nusense.baud);

            log<NUClear::INFO>("PORT OPENED");
        });

        // on<Startup>().then("NUSense HardwareIO Startup", [this] {

        //     // Send fake packets
        //     auto fake_servo_targets = std::make_unique<ServoTargets>();

        //     for (int i = 0; i < 20; ++i) {
        //         auto fake_servo_target = std::make_unique<ServoTarget>();

        //         fake_servo_target->id = i + 1;
        //         fake_servo_target->position = 100.0;
        //         fake_servo_target->gain = 100.0;
        //         fake_servo_target->torque = 100.0;

        //         fake_servo_targets->targets.push_back(*fake_servo_target);
        //     }

        //     emit(fake_servo_targets);
        // });

        on<Shutdown>().then("NUSense HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (nusense.connected()) {
                nusense.close();
            }
        });

        // When we receive data back from NUsense, we handle it here
        on<IO>(nusense.native_handle(), IO::READ).then([this] {
            // TODO Fill this properly below, receive protobuf bytes and parse them to our NUsense message
            // Read from NUsense
            // log<NUClear::INFO>("In IO reaction");
            uint32_t num_bytes = nusense.read(nusense_usb_bytes.data(), 512);

            nusense_receiver.receive(num_bytes, nusense_usb_bytes.data());

            // If packet successfully decoded
            if (nusense_receiver.handle()) {
                const auto& nusense_msg = nusense_receiver.get_nusense_message();

                log<NUClear::INFO>(
                    fmt::format("\nIMU Data\n"
                                "\tAccel(xyz): {} - {} - {}\n"
                                "\t Gyro(xyz): {} - {} - {}\n ",
                                nusense_msg.imu.accel.x,
                                nusense_msg.imu.accel.y,
                                nusense_msg.imu.accel.z,
                                nusense_msg.imu.gyro.x,
                                nusense_msg.imu.gyro.y,
                                nusense_msg.imu.gyro.z));

                log<NUClear::INFO>("Logging servo states...");

                for (const auto& [key, val] : nusense_msg.servo_map) {
                    log<NUClear::INFO>(fmt::format("      key: {}", key));

                    log<NUClear::INFO>(fmt::format("       id: {}", val.id));
                    log<NUClear::INFO>(fmt::format("   hw_err: {}", val.hardware_error));
                    log<NUClear::INFO>(fmt::format("torque_en: {}", val.torque_enabled));
                    log<NUClear::INFO>(fmt::format("     ppwm: {}", val.present_pwm));
                    log<NUClear::INFO>(fmt::format("    pcurr: {}", val.present_current));
                    log<NUClear::INFO>(fmt::format("    pvelo: {}", val.present_velocity));
                    log<NUClear::INFO>(fmt::format("     ppos: {}", val.present_position));
                    log<NUClear::INFO>(fmt::format("     gpwm: {}", val.goal_pwm));
                    log<NUClear::INFO>(fmt::format("    gcurr: {}", val.goal_current));
                    log<NUClear::INFO>(fmt::format("    gvelo: {}", val.goal_velocity));
                    log<NUClear::INFO>(fmt::format("     gpos: {}", val.goal_position));
                    log<NUClear::INFO>(fmt::format("  voltage: {}", val.voltage));
                    log<NUClear::INFO>(fmt::format("     temp: {}", val.temperature));
                }
            }
        });


        // on <Trigger<ServoTargets>>().then([this](const ServoTargets& commands){
        //     // Write the command as one vector. ServoTargets messages are usually greater than 512 bytes but less
        //     than 1024
        //     // This means that the USB2.0 protocol will split this up and will be received on the nusense side as
        //     chunks of 512
        //     // as 512 bytes is the maximum bulk size that 2.0 allows. This also implies that the read callback in the
        //     nusense side
        //     // will be triggered twice.
        //     std::array<char, 3> header = {(char)0xE2, (char)0x98, (char)0xA2};

        //     std::vector<uint8_t> payload  = NUClear::util::serialise::Serialise<ServoTargets>::serialise(commands);

        //     int payload_length = payload.size();
        //     uint8_t high_byte = (payload_length >> 8) & 0xFF;
        //     uint8_t low_byte = payload_length & 0xFF;
        //     std::array<uint8_t, 2> byte_lengths = {high_byte, low_byte};

        //     std::vector<char> full_msg;
        //     full_msg.insert(full_msg.end(), header.begin(), header.end());
        //     full_msg.insert(full_msg.end(), byte_lengths.begin(), byte_lengths.end());
        //     full_msg.insert(full_msg.end(), payload.begin(), payload.end());

        //     nusense.write(full_msg.data(), full_msg.size());

        //     // Logging
        //     log<NUClear::INFO>("Servo targets received");

        //     uint16_t total_length = static_cast<uint16_t>(high_byte << 8) | static_cast<uint16_t>(low_byte);

        //     log<NUClear::INFO>(total_length);
        //     log<NUClear::INFO>(fmt::format("header length: {}  length length: {}  payload length: {}", header.size(),
        //     byte_lengths.size(), payload.size()));

        // });

        // on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
        //     auto command_list = std::make_unique<ServoTargets>();
        //     command_list->targets.push_back(command);

        //     // Emit it so it's captured by the reaction above
        //     emit<Scope::DIRECT>(std::move(command_list));

        //     log<NUClear::INFO>("Servo target received");
        // });
    }

}  // namespace module::platform::NUsense
