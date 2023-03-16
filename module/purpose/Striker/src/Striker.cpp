#include "Striker.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/Striker.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase    = message::input::GameState::Data::Phase;
    using GameMode = message::input::GameState::Data::Mode;
    using message::input::GameState;
    using message::planning::KickTo;
    using message::purpose::PenaltyShootoutStriker;
    using message::purpose::PlayStriker;
    using message::strategy::FallRecovery;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using StrikerTask = message::purpose::Striker;
    using utility::support::Expression;

    Striker::Striker(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Striker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Striker.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.rRFf        = config["rRFf"].as<Expression>();
        });

        on<Provide<StrikerTask>, Optional<With<GameState>>>().then(
            [this](const StrikerTask& striker_task, const std::shared_ptr<const GameState>& game_state) {
                // Do not use GameController information if force playing or force penalty shootout
                if (striker_task.force_playing || striker_task.force_penalty_shootout) {
                    play();
                    return;
                }

                // Check if tthere is GameState information, and if so act based on the current mode
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

        // Normal READY state
        on<Provide<PlayStriker>, With<Phase, std::equals, Phase::READY>>().then(
            [this] { emit<Task>(std::make_unique<Ready>(cfg.rRFf)); });

        // Normal PLAYING state
        on<Provide<PlayStriker>, With<Phase, std::equals, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<PlayStriker>, With<Phase, std::equals, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<PlayStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty shootout PLAYING state
        on<Provide<PenaltyShootoutStriker>, With<Phase, std::equals, Phase::PLAYING>>().then([this] { play(); });

        // Penalty shootout UNKNOWN state
        on<Provide<PenaltyShootoutStriker>, With<Phase, std::equals, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown penalty shootout game phase."); });

        // Default for INITIAL, READY, SET, FINISHED, TIMEOUT
        on<Provide<PenaltyShootoutStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });
    }

    void Striker::play() {
        // Walk to the ball and kick!
        emit<Task>(std::make_unique<StandStill>());     // if nothing else is happening, stand still
        emit<Task>(std::make_unique<FindBall>(), 1);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 2);  // try to track the ball
        emit<Task>(std::make_unique<WalkToBall>(), 3);  // try to walk to the ball
        emit<Task>(std::make_unique<KickTo>(Eigen::Vector3f::Zero()), 4);  // kick the ball if possible
        emit<Task>(std::make_unique<FallRecovery>(), 5);                   // recover from falling if applicable
    }

}  // namespace module::purpose
