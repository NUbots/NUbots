#include "Reaction.hpp"
#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/onboarding/Catalyst.hpp"
#include "message/onboarding/Reaction.hpp"
#include "message/onboarding/Explosion.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Reaction::Reaction(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Catalyst;
        using message::onboarding::Reaction;
        using message::onboarding::Explosion;

        on<Configuration>("Reaction.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Reaction.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<Catalyst>>().then([this](const Catalyst& cata_msg) {
            auto react_msg = std::make_unique<Reaction>();
            react_msg->count = cata_msg.count;
            react_msg->sum = cata_msg.sum;
            if(react_msg->count < 10) {
                react_msg->count++;
                react_msg->sum = cata_msg.sum + react_msg->count;
                log<NUClear::INFO>(fmt::format("Sum at count {} is {}", react_msg->count, react_msg->sum));
            }
            emit(react_msg);
        });
    }

}  // namespace module::onboarding
