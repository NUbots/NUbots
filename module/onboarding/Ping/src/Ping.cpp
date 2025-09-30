#include "Ping.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {
    using extension::Configuration;

    Ping::Ping(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        // Responsible for responding to pongs with an incremented ping

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Pass
        });

        on<Trigger<Pong>>().then([this](const Pong& pong_msg) {
            // Constructs an increment ping from the pong, then emits it
            auto ping_msg       = std::make_unique<Ping>();
            ping_msg->value     = pong_msg.value + pong_msg.iteration + 1;
            ping_msg->iteration = pong_msg.iteration + 1;
            log<INFO>("Total is: " + std::to_string(ping_msg->value));
            log<INFO>("Ping");
            emit(ping_msg);
        });
    }

}  // namespace module::onboarding
