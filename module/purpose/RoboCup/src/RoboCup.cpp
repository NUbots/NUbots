#include "RoboCup.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/GameEvents.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Striker.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using message::behaviour::state::Stability;
    using message::input::GameEvents;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::purpose::Defender;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::PenaltyShootoutGoalie;
    using message::purpose::PenaltyShootoutStriker;
    using message::purpose::PlayDefender;
    using message::purpose::PlayGoalie;
    using message::purpose::PlayStriker;
    using message::purpose::Striker;

    RoboCup::RoboCup(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("RoboCup.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RoboCup.yaml
            this->log_level            = config["log_level"].as<NUClear::LogLevel>();
            cfg.force_playing          = config["force_playing"].as<bool>();
            cfg.force_penalty_shootout = config["force_penalty_shootout"].as<bool>();

            // Get the soccer position, if not valid option then default to striker
            cfg.position = Position(config["position"].as<std::string>());
        });

        // Start the Director graph for the RoboCup soccer scenario!
        on<Startup>().then([this] {
            emit<Task>(std::make_unique<FindPurpose>());
            // At the start of the program, we should be standing
            emit(std::make_unique<Stability>(Stability::STANDING));
        });

        on<Provide<FindPurpose>>().then([this] {
            // If force play is active, assume it is the playing state of the game
            if (cfg.force_playing) {
                switch (cfg.position) {
                    case Position::STRIKER: emit<Task>(std::make_unique<PlayStriker>()); break;
                    case Position::GOALIE: emit<Task>(std::make_unique<PlayGoalie>()); break;
                    case Position::DEFENDER: emit<Task>(std::make_unique<PlayDefender>()); break;
                    default: log<NUClear::ERROR>("Invalid robot position");
                }
                return;
            }

            // If force penalty shootout is active, assume we are in a penalty shootout
            if (cfg.force_penalty_shootout) {
                switch (cfg.position) {
                    case Position::STRIKER: emit<Task>(std::make_unique<PenaltyShootoutStriker>()); break;
                    case Position::GOALIE: emit<Task>(std::make_unique<PenaltyShootoutGoalie>()); break;
                    case Position::DEFENDER: log<NUClear::ERROR>("Defender is not valid in penalty shootout"); break;
                    default: log<NUClear::ERROR>("Invalid robot position");
                }
                return;
            }

            // Play with GameController state info
            switch (cfg.position) {
                case Position::STRIKER: emit<Task>(std::make_unique<Striker>()); break;
                case Position::GOALIE: emit<Task>(std::make_unique<Goalie>()); break;
                case Position::DEFENDER: emit<Task>(std::make_unique<Defender>()); break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
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
                // emit(std::make_unique<Nod>(true));
                cfg.force_playing = true;
                emit<Task>(std::make_unique<FindPurpose>());
            }
        });
    }

}  // namespace module::purpose
