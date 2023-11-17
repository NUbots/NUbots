#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ping.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Vibe
            auto pong =  std::make_unique<Pong>();
            emit(pong);
        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            log<NUClear::INFO>("Pong");
            auto pong =  std::make_unique<Pong>();
            emit(pong);
        });
    }

}  // namespace module::onboarding
