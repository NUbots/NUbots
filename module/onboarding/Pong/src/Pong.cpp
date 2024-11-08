#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Pong.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Pong.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Start the ping pong chain
            auto pong_msg = std::make_unique<Pong>();
            emit(pong_msg);
        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            // TODO: Log a INFO level message with the text "Pong"
            log<NUClear::INFO>("Pong");
            // TODO: Emit a Pong message
            auto pong_msg = std::make_unique<Pong>();
            emit(pong_msg);
        });
    }

}  // namespace module::onboarding
