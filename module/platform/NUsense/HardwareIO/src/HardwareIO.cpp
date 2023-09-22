#include "extension/Configuration.hpp"
#include "HardwareIO.hpp"
#include "message/actuation/ServoTarget.hpp"
#include "NUsense/SIProcessor.hpp"


namespace module::platform::NUsense {

    using extension::Configuration;
    using message::platform::RawSensors;// This will most likely change to a more specific message for NUsense
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
        });

        on<Shutdown>().then("NUSense HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (nusense.connected()) {
                nusense.close();
            }
        });

        // TODO add an on<IO> to handle messages coming from nusense
        // When we receive data back from NUsense it will arrive here
        on<IO>(nusense.native_handle(), IO::READ).then([this] {
            // TODO Fill this properly below, receive protobuf bytes and parse them to our NUsense message

        });

        // TODO create a message tailored for NUsense and emit it

        // When a message for NUsense is detected, serialise it and send it to the port
        // on<Trigger<RawSensors>>().then([this](const RawSensors& msg){
        //     std::vector<char> serialised_msg = msg_to_nbs(msg);
        //     nusense.write(serialised_msg.data(), serialised_msg.size());
        // });

        on <Trigger<ServoTargets>>().then([this](const ServoTargets& commands){
            log<NUClear::INFO>("Servo targets received");
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(commandList);
        });
    }

}  // namespace module::platform::NUsense
