#include "Ping.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Ping::Ping(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ping.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {});

        on<Trigger<Pong>>().then([this](const Pong& pong_msg) {
            auto ping_msg   = std::make_unique<Ping>();
            ping_msg->count = pong_msg.count + 1;
            ping_msg->val   = (ping_msg->count + pong_msg.val);
            log<NUClear::INFO>("Ping:" + std::to_string(ping_msg->count));
            emit(ping_msg);
        });
    }

}  // namespace module::onboarding
