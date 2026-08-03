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

            // Start the counters and track it
            pong_msg->count = 1;
            pong_msg->iter_count = 1;

            log<INFO>("Pong count: ", pong_msg->count, " Pong iter_count: ", pong_msg->iter_count);
            emit(pong_msg);
    });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            auto pong_msg = std::make_unique<Pong>();
            // log<INFO>("Pong");

            // Increment the counters and track it
            pong_msg->count = ping_msg.count + 1;
            pong_msg->iter_count = pong_msg->count + ping_msg.iter_count;

            log<INFO>("Pong count: ", pong_msg->count, " Pong iter_count: ", pong_msg->iter_count);

            emit(pong_msg);
        });



}

}  // namespace module::onboarding
