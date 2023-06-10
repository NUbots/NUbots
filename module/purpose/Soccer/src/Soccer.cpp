#include "Soccer.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/GameEvents.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Striker.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using message::behaviour::state::Stability;
    using message::input::GameEvents;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::purpose::Defender;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::Striker;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    Soccer::Soccer(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Soccer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Soccer.yaml
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.force_playing = config["force_playing"].as<bool>();

            // Get the soccer position, if not valid option then default to striker
            cfg.position = Position(config["position"].as<std::string>());
        });

        // Start the Director graph for the soccer scenario!
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without this emit, modules that need a Stability message may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            // This emit starts the tree to play soccer
            emit<Task>(std::make_unique<FindPurpose>());
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 1);
        });

        on<Provide<FindPurpose>>().then([this] {
            // Make task based on configured purpose/soccer position
            switch (cfg.position) {
                case Position::STRIKER: emit<Task>(std::make_unique<Striker>(cfg.force_playing)); break;
                case Position::GOALIE: emit<Task>(std::make_unique<Goalie>(cfg.force_playing)); break;
                case Position::DEFENDER: emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
                // emit<Task>(std::make_unique<StandStill>());
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // If the robot is unpenalised, stop standing still and find its purpose
            if (self_unpenalisation.context == GameEvents::Context::SELF) {
                // emit<Task>(std::unique_ptr<StandStill>(nullptr));
                emit<Task>(std::make_unique<FindPurpose>());
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            // Middle button forces playing
            log<NUClear::INFO>("Middle button pressed!");
            if (!cfg.force_playing) {
                log<NUClear::INFO>("Force playing started.");
                cfg.force_playing = true;
                emit<Task>(std::make_unique<FindPurpose>());
                // emit<Task>(std::unique_ptr<StandStill>(nullptr));
            }
        });
    }

}  // namespace module::purpose
