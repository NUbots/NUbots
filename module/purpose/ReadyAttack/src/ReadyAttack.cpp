#include "ReadyAttack.hpp"

#include "extension/Configuration.hpp"

namespace module::purpose {

    using extension::Configuration;

    ReadyAttack::ReadyAttack(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ReadyAttack.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReadyAttack.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<ReadyAttack>>().then([this] {
            // We are the attacker, but something is happening to prevent us going directly to the ball
            // Currently this would only be a penalty situation where we are not allowed to attack

            // FREE KICK Must stay 0.75m back from the ball, unless they are on their own goal line between the
            // goalposts
            //
        });
    }

}  // namespace module::purpose
