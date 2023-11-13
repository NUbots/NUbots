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

    on<Startup>().then([this]{
        // Start the reaction
        auto ping_msg = std::make_unique<Ping>();
        ping_msg -> val = 1;
        ping_msg -> iter_count = 1;

        emit(ping_msg);
    });

    on<Trigger<Pong>>().then([this](const Pong& pong_msg){
        auto ping_msg = std::make_unique<Ping>();
        ping_msg -> val = pong_msg.val + pong_msg.iter_count + 1;
        ping_msg -> iter_count = pong_msg.iter_count + 1;

        log<NUClear::INFO>(ping_msg->val);
        log<NUClear::INFO>(ping_msg->iter_count);

        emit(ping_msg);
    });
}

}  // namespace module::onboarding
