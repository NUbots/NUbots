#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        // Load configuration
        on<Configuration>("Pong.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Emit the first Pong at startup
        on<Startup>().then([this] {
            auto pong_msg   = std::make_unique<Pong>();
            pong_msg->count = counter;
            emit(std::move(pong_msg));
        });

        // Listen for Ping messages and respond
        on<Trigger<Ping>>().then([this](const Ping& /*ping_msg*/) {
            counter++;
            log<INFO>("Pong", counter);
            auto pong_msg   = std::make_unique<Pong>();
            pong_msg->count = counter;
            emit(std::move(pong_msg));
        });
    }

}  // namespace module::onboarding
