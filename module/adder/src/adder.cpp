#include "adder.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding.hpp"

namespace module {

    using message::onboarding::check;
    using message::onboarding::new_k;

    using extension::Configuration;

    adder::adder(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("adder.yaml").then([this](const Configuration& config) {
            // Use configuration here from file adder.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<new_k>>().then([this](const new_k& k) {
            log<INFO>("what the helly");
            cfg.running_total += k.k;
            log<INFO>("Received new_k with k =", k.k, "running total is now", cfg.running_total);
            auto message_check   = std::make_unique<check>();
            message_check->value = cfg.running_total;
            emit(message_check);
            // nvm
        });
    }

}  // namespace module
