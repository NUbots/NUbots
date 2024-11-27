#include "Ping.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/Task.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Ping::Ping(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;
        using message::onboarding::Task;

        on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ping.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            auto ping_msg = std::make_unique<Ping>();
            // Assign initial k value
            ping_msg->k = 0;
            // Set starting sum
            ping_msg->tempSum = 0;

            // Emite ping to start the reaction
            emit(ping_msg);
        });

        on<Trigger<Pong>>().then([this](const Pong& pong_msg) {
            auto ping_msg = std::make_unique<Ping>();
            // Check judgeMe flag
            if(pong_msg.judgeMe == 1)
            {
                // Do nothing
            }
            else
            {
                // Add k to sum
                ping_msg->tempSum = pong_msg.tempSum + pong_msg.k;
                // bring k forward 1
                ping_msg->k = pong_msg.k + 1;
                log<NUClear::INFO>("Ping");
                emit(ping_msg);
            };

        });
    }

}  // namespace module::onboarding
