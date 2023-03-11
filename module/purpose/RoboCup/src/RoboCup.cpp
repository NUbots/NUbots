#include "RoboCup.hpp"

#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Striker.hpp"

namespace module::purpose {

    using extension::Configuration;
    using message::input::GameState;
    using message::purpose::Defender;
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
            cfg.force_play             = config["force_play"].as<bool>();
            cfg.force_penalty_shootout = config["force_penalty_shootout"].as<bool>();

            // Get the soccer position, if not valid option then default to striker
            cfg.position = Position(config["position"].as<std::string>());
        });

        on<Trigger<GameState>>().then([this](const GameState& game_state) {
            // If the robot is penalised, it should do nothing
            if (game_state.data.self.penalty_reason != GameState::Data::PenaltyReason::UNPENALISED) {
                // emit<Task>(std::make_unique<StandStill>());
                return;
            }

            // If force play is active, assume it is the playing state of the game
            if (cfg.force_play) {
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
    }

}  // namespace module::purpose
