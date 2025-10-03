#include "checker.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding.hpp"

namespace module {

    using extension::Configuration;
    using message::actuation::HeadSequence;
    using message::onboarding::check;

    checker::checker(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("checker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file checker.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });
        on<Trigger<check>>().then([this](const check& c) {
            log<INFO>("Received check with value =", c.value);
            if (c.value == 55) {
                log<INFO>("The value is correct!");
            }
            else {
                log<ERROR>("The value is incorrect!");
            }
        });
    }

}  // namespace module
