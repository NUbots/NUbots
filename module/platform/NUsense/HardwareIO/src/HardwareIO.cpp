#include "extension/Configuration.hpp"
#include "HardwareIO.hpp"
#include "message/actuation/ServoTarget.hpp"
#include "NUsense/SIProcessor.hpp"

#include <sstream>
#include <fmt/format.h>
#include <cmath>

namespace module::platform::NUsense {

    using extension::Configuration;
    // using message::platform::RawSensors;// This will most likely change to a more specific message for NUsense
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;


    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), nusense() {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HardwareIO.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.nusense.port        = config["nusense"]["port"].as<std::string>();
            cfg.nusense.baud        = config["nusense"]["baud"].as<int>();
        });

        on<Startup>().then("NUSense HardwareIO Startup", [this] {
            // Make sure the port is closed before we open it
            if (nusense.connected()){
                nusense.close();
            }

            // If /dev/ttyACM0 is acting up, follow the link below
            // https://stackoverflow.com/questions/40951728/avrdude-ser-open-cant-open-device-dev-ttyacm0-device-or-resource-busy
            nusense.open(cfg.nusense.port, cfg.nusense.baud);
            log<NUClear::INFO>("PORT OPENED");

            // Send fake packets
            auto fake_servo_targets = std::make_unique<ServoTargets>();

            for (int i = 0; i < 20; ++i) {
                auto fake_servo_target = std::make_unique<ServoTarget>();

                fake_servo_target->id = i + 1;
                fake_servo_target->position = 100.0;
                fake_servo_target->gain = 100.0;
                fake_servo_target->torque = 100.0;

                fake_servo_targets->targets.push_back(*fake_servo_target);
            }

            emit(fake_servo_targets);
        });

        on<Shutdown>().then("NUSense HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (nusense.connected()) {
                nusense.close();
            }
        });

        // When we receive data back from NUsense it will arrive here
        on<IO>(nusense.native_handle(), IO::READ).then([this] {
            // TODO Fill this properly below, receive protobuf bytes and parse them to our NUsense message
            // Read from NUsense
            log<NUClear::INFO>("In IO reaction");
            std::vector<char> buf;
            uint8_t num_bytes;
            num_bytes = nusense.read(buf.data(), 18);

            std::string str(buf.begin(), buf.end());
            log<NUClear::INFO>(str);
        });


        on <Trigger<ServoTargets>>().then([this](const ServoTargets& commands){
            log<NUClear::INFO>("Servo targets received");

            std::vector<char> serialised_msg = msg_to_nbs(commands);

            // Must split the packets up by 512 bytes before sending
            // If not split, we will lose data because the STM32 buffer is circular by nature
            float max_split_size = 512.0;
            float n_split = serialised_msg.size() / max_split_size;
            int splits = (int)std::ceil(n_split);

            // Check packet before writing
            log<NUClear::INFO>(fmt::format("length: {}", serialised_msg.size()));

            int start_index = 0;
            for (int i = 0; i < splits; ++i) {
                int chunk_size = static_cast<int>(std::min(max_split_size, static_cast<float>(serialised_msg.size() - start_index)));
                std::vector<char> ith_split(serialised_msg.begin() + start_index, serialised_msg.begin() + start_index + chunk_size);

                // Send the ith split to nusense then update our indices
                nusense.write(ith_split.data(), ith_split.size());
                start_index += chunk_size;

                log<NUClear::INFO>(fmt::format("ith split size {}", ith_split.size()));
            }

            uint8_t high_byte = static_cast<uint8_t>(serialised_msg[3]);
            uint8_t low_byte = static_cast<uint8_t>(serialised_msg[4]);

            log<NUClear::INFO>(high_byte);
            log<NUClear::INFO>(low_byte);

            uint16_t total_length = static_cast<uint16_t>(high_byte << 8) | static_cast<uint16_t>(low_byte);

            log<NUClear::INFO>(total_length);
            log<NUClear::INFO>((int16_t)total_length);

        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(command_list));

            log<NUClear::INFO>("Servo target received");
        });
    }

}  // namespace module::platform::NUsense
