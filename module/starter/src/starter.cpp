#include "starter.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding.hpp"

namespace module {

    using extension::Configuration;
    using message::onboarding::new_k;

    starter::starter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("starter.yaml").then([this](const Configuration& config) {
            // Use configuration here from file starter.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            int k           = config["k"].as<int>();
            int n           = config["n"].as<int>();
            NUClear::log<INFO>("k =", k, "n =", n);
        });
        on<Startup>().then([this] {
            for (int i = 1; i <= 10; ++i) {
                NUClear::log<INFO>("Sending new_k with k =", i);
                auto message_k = std::make_unique<message::onboarding::new_k>();
                message_k->k   = i;
                emit(message_k);
            }
        });
    }

}  // namespace module
