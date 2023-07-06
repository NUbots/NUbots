#include "Striker.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/Striker.hpp"
#include "message/skill/Look.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/AlignRobotToBall.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/Localise.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Phase    = message::input::GameState::Data::Phase;
    using GameMode = message::input::GameState::Data::Mode;
    using message::input::GameState;
    using message::planning::KickTo;
    using message::purpose::CornerKickStriker;
    using message::purpose::DirectFreeKickStriker;
    using message::purpose::GoalKickStriker;
    using message::purpose::InDirectFreeKickStriker;
    using message::purpose::NormalStriker;
    using message::purpose::PenaltyKickStriker;
    using message::purpose::PenaltyShootoutStriker;
    using message::purpose::ThrowInStriker;
    using message::skill::Look;
    using message::strategy::AlignBallToGoal;
    using message::strategy::AlignRobotToBall;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::Localise;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;


    using StrikerTask = message::purpose::Striker;
    using utility::support::Expression;

    Striker::Striker(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Striker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Striker.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.ready_position = config["ready_position"].as<Expression>();
        });

        on<Provide<StrikerTask>, Optional<Trigger<GameState>>>().then(
            [this](const StrikerTask& striker_task, const std::shared_ptr<const GameState>& game_state) {
                // Do not use GameController information if force playing or force penalty shootout
                if (striker_task.force_playing) {
                    play();
                    return;
                }

                // Check if there is GameState information, and if so act based on the current mode
                if (game_state) {
                    switch (game_state->data.mode.value) {
                        case GameMode::PENALTY_SHOOTOUT: emit<Task>(std::make_unique<PenaltyShootoutStriker>()); break;
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: emit<Task>(std::make_unique<NormalStriker>()); break;
                        case GameMode::DIRECT_FREEKICK: emit<Task>(std::make_unique<DirectFreeKickStriker>()); break;
                        case GameMode::INDIRECT_FREEKICK:
                            emit<Task>(std::make_unique<InDirectFreeKickStriker>());
                            break;
                        case GameMode::PENALTYKICK: emit<Task>(std::make_unique<PenaltyKickStriker>()); break;
                        case GameMode::CORNER_KICK: emit<Task>(std::make_unique<CornerKickStriker>()); break;
                        case GameMode::GOAL_KICK: emit<Task>(std::make_unique<GoalKickStriker>()); break;
                        case GameMode::THROW_IN: emit<Task>(std::make_unique<ThrowInStriker>()); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
            });

        // Normal READY state
        on<Provide<NormalStriker>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // If we are stable and localised, walk to the ready field position
            emit<Task>(std::make_unique<WalkToFieldPosition>(
                           Eigen::Vector3f(cfg.ready_position.x(), cfg.ready_position.y(), 0),
                           cfg.ready_position.z()),
                       1);
            emit<Task>(std::make_unique<Localise>(), 2);                      // localise if necessary
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX()), 3);  // Look straight ahead
        });

        // Normal PLAYING state
        on<Provide<NormalStriker>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Normal UNKNOWN state
        on<Provide<NormalStriker>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown normal game phase."); });

        // Default for INITIAL, SET, FINISHED, TIMEOUT
        on<Provide<NormalStriker>>().then([this] {
            emit<Task>(std::make_unique<StandStill>(), 1);
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX()), 2);  // Look straight ahead
        });

        // Penalty shootout PLAYING state
        on<Provide<PenaltyShootoutStriker>, When<Phase, std::equal_to, Phase::PLAYING>>().then([this] { play(); });

        // Penalty shootout UNKNOWN state
        on<Provide<PenaltyShootoutStriker>, When<Phase, std::equal_to, Phase::UNKNOWN_PHASE>>().then(
            [this] { log<NUClear::WARN>("Unknown penalty shootout game phase."); });

        // Default for INITIAL, READY, SET, FINISHED, TIMEOUT
        on<Provide<PenaltyShootoutStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Direct free kick
        on<Provide<DirectFreeKickStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Indirect free kick
        on<Provide<InDirectFreeKickStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Penalty kick
        on<Provide<PenaltyKickStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Corner kick
        on<Provide<CornerKickStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Goal kick
        on<Provide<GoalKickStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // Throw in
        on<Provide<ThrowInStriker>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });
    }

    void Striker::play() {
        // Walk to the ball and kick!
        // Second argument is priority - higher number means higher priority
        emit<Task>(std::make_unique<FindBall>(), 1);    // if the look/walk to ball tasks are not running, find the ball
        emit<Task>(std::make_unique<LookAtBall>(), 2);  // try to track the ball
        emit<Task>(std::make_unique<WalkToBall>(), 3);  // try to walk to the ball
        emit<Task>(std::make_unique<AlignRobotToBall>(), 4);  // Align robot to ball
        emit<Task>(std::make_unique<AlignBallToGoal>(), 5);   // try to walk to the ball
        // emit<Task>(std::make_unique<KickToGoal>(), 6);        // kick the ball if possible
    }

}  // namespace module::purpose
