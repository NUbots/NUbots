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

        on<Startup>().then([this] {
            // Vibe
        });

        on<Trigger<Pong>>().then([this](const Pong& pong_msg) {
            auto ping = std::make_unique<Ping>();
            if (cfg.num == 10) {
                log<NUClear::INFO>("Stop bounce");
                log<NUClear::INFO>(cfg.total);
            }
            else {
                cfg.num += 1;
                log<NUClear::INFO>(cfg.num);
                cfg.total *= cfg.num;
                log<NUClear::INFO>("Size2", cfg.total);
                emit(ping);
            }
        });
    }

}  // namespace module::onboarding
