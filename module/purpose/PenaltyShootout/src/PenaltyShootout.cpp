#include "PenaltyShootout.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using extension::Configuration;
    using utility::skill::load_script;

    using Phase = message::input::GameState::Phase;

    using message::actuation::BodySequence;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::planning::LookAround;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::WalkToBall;
    using message::support::FieldDescription;

    struct StartPenalty {};
    struct PenaltyGoalie {};
    struct Play {};

    PenaltyShootout::PenaltyShootout(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PenaltyShootout.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PenaltyShootout.yaml
            this->log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_action_distance = config["ball_action_distance"].as<double>();
            cfg.startup_delay        = config["startup_delay"].as<int>();
            cfg.ball_search_timeout  = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
        });

        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Stand idle
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            // Idle look forward if the head isn't doing anything else
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true), 0);
            // Startup delay to prevent issues with low servo gains at the start
            emit<Scope::DELAY>(std::make_unique<StartPenalty>(), std::chrono::seconds(cfg.startup_delay));
        });

        on<Trigger<StartPenalty>>().then([this] {
            // This emit starts the tree to play the penalty shootout
            emit<Task>(std::make_unique<Play>(), 1);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);
        });

        on<Provide<Play>,
           Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>,
           With<GameState>,
           Optional<With<Ball>>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const GameState& game_state, const std::shared_ptr<const Ball>& ball) {
                // Find the ball first
                if (ball == nullptr || (NUClear::clock::now() - ball->time_of_measurement) > cfg.ball_search_timeout) {
                    emit<Task>(std::make_unique<LookAround>());
                }

                // If our kick off, walk the ball into the goal!
                if (game_state.our_kick_off) {
                    emit<Task>(std::make_unique<WalkToBall>());
                    emit<Task>(std::make_unique<LookAtBall>());
                }
                // Else be a goalie!
                else {
                    emit<Task>(std::make_unique<PenaltyGoalie>());
                    emit<Task>(std::make_unique<LookAtBall>(), 1);  // Track the ball
                }
            });

        on<Provide<PenaltyGoalie>, With<Ball>, With<Sensors>>().then([this](const Ball& ball, const Sensors& sensors) {
            // Get position of the ball in robot space
            Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;

            // Check if the ball is close enough to act yet
            if (rBRr.norm() > cfg.ball_action_distance) {
                return;
            }

            // Walk the ball away
            emit<Task>(std::make_unique<WalkToBall>());
        });
    }

}  // namespace module::purpose
