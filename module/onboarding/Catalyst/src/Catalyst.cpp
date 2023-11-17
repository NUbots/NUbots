#include "Catalyst.hpp"
#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/onboarding/Catalyst.hpp"
#include "message/onboarding/Reaction.hpp"
#include "message/onboarding/Explosion.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Catalyst::Catalyst(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Catalyst;
        using message::onboarding::Reaction;
        using message::onboarding::Explosion;

        on<Configuration>("Catalyst.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Catalyst.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            auto cata_msg = std::make_unique<Catalyst>();
            cata_msg->sum = 1;
            cata_msg->count = 1;
            log<NUClear::INFO>(fmt::format("Sum at count {} is {}", cata_msg->count, cata_msg->sum));
            emit(cata_msg);
        });

        on<Trigger<Reaction>>().then([this](const Reaction& react_msg) {
            auto cata_msg = std::make_unique<Catalyst>();
            cata_msg->count = react_msg.count;
            cata_msg->sum = react_msg.sum;
            if(cata_msg->count < 10) {
                cata_msg->count++;
                cata_msg->sum = react_msg.sum + cata_msg->count;
                log<NUClear::INFO>(fmt::format("Sum at count {} is {}", cata_msg->count, cata_msg->sum));
            }
            emit(cata_msg);
        });
    }

}  // namespace module::onboarding
