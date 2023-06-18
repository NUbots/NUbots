#include "Defender.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/Defender.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase    = message::input::GameState::Data::Phase;
    using GameMode = message::input::GameState::Data::Mode;
    using message::input::GameState;
    using message::planning::KickTo;
    using message::purpose::NormalDefender;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkInsideBoundedBox;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;

    using DefenderTask = message::purpose::Defender;
    using utility::support::Expression;

    Defender::Defender(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defender.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defender.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position = config["ready_position"].as<Expression>();
        });

        on<Provide<DefenderTask>, Optional<Trigger<GameState>>>().then(
            [this](const DefenderTask& defender_task, const std::shared_ptr<const GameState>& game_state) {
                log<NUClear::DEBUG>("DEFENDER");
                // Do not use GameController information if force playing or force penalty shootout
                if (defender_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->data.mode.value) {
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalDefender>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        // Normal READY state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // If we are stable, walk to the ready field position
            log<NUClear::DEBUG>("READY");
            emit<Task>(std::make_unique<WalkToFieldPosition>(
                Eigen::Vector3f(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                cfg.ready_position.z()));
        });

        // Normal PLAYING state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<NormalDefender>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalDefender>>().then([this] {
            log<NUClear::DEBUG>("INITIAL");
            emit<Task>(std::make_unique<StandStill>());
        });
    }

    void Defender::play() {
        // Walk to the ball and kick!
        // Second argument is priority - higher number means higher priority
        emit<Task>(std::make_unique<FindBall>(), 1);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 2);  // try to track the ball
        emit<Task>(std::make_unique<WalkToBall>(), 3);  // try to walk to the ball
        emit<Task>(std::make_unique<AlignBallToGoal>(), 4);       // Aligning ball to goal to aim kick
        emit<Task>(std::make_unique<KickToGoal>(), 5);            // kick the ball if possible
        emit<Task>(std::make_unique<WalkInsideBoundedBox>(), 6);  // Patrol bounded box region
    }

}  // namespace module::purpose
