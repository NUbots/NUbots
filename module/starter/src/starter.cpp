#include "starter.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding.hpp"

namespace module {

    using extension::Configuration;
    using message::onboarding::new_n;

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
                NUClear::log<INFO>("Sending new_n with n =", i);
                auto message_n = std::make_unique<message::onboarding::new_n>();
                message_n->n   = i;
                emit(message_n);
            }
        });
    }

}  // namespace module
