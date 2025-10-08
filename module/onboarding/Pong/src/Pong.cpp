
#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/VerifiedPing.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        // Responsible for starting the ping reaction and responding to pings with pongs

        using message::onboarding::Ping;
        using message::onboarding::Pong;
        using message::onboarding::VerifiedPing;

        on<Configuration>("Pong.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Pong.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Initiates the ping pong chain on startup
            auto pong_msg       = std::make_unique<Pong>();
            pong_msg->value     = 0;
            pong_msg->iteration = 0;
            emit(pong_msg);
        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            // Constructs then emits a pong message
            if (ping_msg.iteration >= 10) {
                log<INFO>("Finished iterating, value is: " + std::to_string(ping_msg.value));
                auto judged_msg = std::make_unique<VerifiedPing>();
                emit(judged_msg);
                return;
            }

            auto pong_msg       = std::make_unique<Pong>();
            pong_msg->value     = ping_msg.value;
            pong_msg->iteration = ping_msg.iteration;
            log<INFO>("Pong");
            emit(pong_msg);
        });
    }
}  // namespace module::onboarding
