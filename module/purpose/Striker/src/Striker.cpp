#include "Striker.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/planning/LookAtFeature.hpp"
#include "message/purpose/Striker.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase = message::input::GameState::Data::Phase;
    using message::input::GameState;
    using message::planning::KickTo;
    using message::planning::LookAtBall;
    using message::purpose::PenaltyShootoutStriker;
    using message::purpose::PlayStriker;
    using StrikerTask = message::purpose::Striker;
    using message::strategy::FallRecovery;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using utility::support::Expression;
    using GameMode = message::input::GameState::Data::Mode;

    Striker::Striker(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Striker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Striker.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.rRFf        = config["rRFf"].as<Expression>();
        });

        on<Provide<StrikerTask>, Optional<With<GameState>>>().then(
            [this](const std::shared_ptr<const GameState>& game_state) {
                log<NUClear::WARN>("striker task");
                if (game_state) {
                    auto mode = game_state->data.mode.value;
                    switch (mode) {
                        case GameMode::PENALTY_SHOOTOUT: emit<Task>(std::make_unique<PenaltyShootoutStriker>()); break;
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<PlayStriker>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        on<Provide<PlayStriker>, Optional<With<Phase>>>().then([this](const std::shared_ptr<const Phase>& phase) {
            log<NUClear::WARN>("Play striker");
            play();
            if (phase) {
                switch (phase->value) {
                    // Stand still in initial and set state
                    case Phase::INITIAL:   // Beginning of game and half time
                    case Phase::SET:       // After ready
                    case Phase::FINISHED:  // Game has finished
                    case Phase::TIMEOUT:   // A pause in playing - not in simulation
                        emit<Task>(std::make_unique<StandStill>());
                        break;
                    // After initial, robots position on their half of the field
                    case Phase::READY: emit<Task>(std::make_unique<Ready>(cfg.rRFf)); break;
                    // After set, main game where we should play soccer
                    case Phase::PLAYING: play(); break;
                    default: log<NUClear::WARN>("Unknown normal gamemode phase.");
                }
            }
        });

        on<Provide<PenaltyShootoutStriker>, With<Phase>>().then([this](const Phase& phase) {
            switch (phase.value) {
                // Stand still in initial and set state
                case Phase::INITIAL:   // Beginning of penalty shootout
                case Phase::SET:       // After initial
                case Phase::FINISHED:  // Penalty shootout has finished
                case Phase::TIMEOUT:   // A pause in playing - not in simulation
                    emit<Task>(std::make_unique<StandStill>());
                    break;
                // After set, where we should play a penalty shootout
                case Phase::PLAYING: play(); break;
                default: log<NUClear::WARN>("Unknown penalty shootout gamemode phase.");
            }
        });
    }

    void Striker::play() {
        emit<Task>(std::make_unique<StandStill>());
        emit<Task>(std::make_unique<LookAtBall>(), 1);
        emit<Task>(std::make_unique<WalkToBall>(), 2);
        emit<Task>(std::make_unique<KickTo>(Eigen::Vector3f::Zero()), 3);
        emit<Task>(std::make_unique<FallRecovery>(), 4);
    }

}  // namespace module::purpose
