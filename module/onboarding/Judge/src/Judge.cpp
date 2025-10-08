#include "Judge.hpp"

#include "extension/Configuration.hpp"

// #include "message/onboarding/..."
#include "message/onboarding/VerifiedPing.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::VerifiedPing;

        on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ping.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // pass
        });


        on<Trigger<VerifiedPing>>().then([this](const VerifiedPing& verified_ping) {
            /*
            auto nod = std::make_unique<the nod message>();
            emit(nod);
            */
            log<INFO>("Done");
        });
    }

}  // namespace module::onboarding
