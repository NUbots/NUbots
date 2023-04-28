#include "KeyboardWalk.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::purpose {

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("KeyboardWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KeyboardWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Start the Director graph for the KeyboardWalk.
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without this emit, modules that need a Stability message may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 1);
            // Stand Still on startup, TODO: Fix this preventing robot from being able to walk.
            // emit<Task>(std::make_unique<StandStill>(), 2);
        });
    }

}  // namespace module::purpose
