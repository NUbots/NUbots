#include "PenaltyShootout.hpp"

#include "extension/Configuration.hpp"

namespace module::purpose {

    using extension::Configuration;

    struct StartPenalty {};
    struct Play {};

    PenaltyShootout::PenaltyShootout(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("PenaltyShootout.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PenaltyShootout.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.is_goalie   = config["is_goalie"].as<bool>();
        });

        on<Startup, With<FieldDescription>>().then([this](const FieldDescription& field_description) {
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

            // Emit localisation reset in specific custom positions for the penalty
            if (cfg.is_goalie) {
                double x = field_description.field_length / 2.0;
                emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>(true, Eigen::Vector3d(x, 0.0, 0.0)));
            }
            else {
                double x = field_description.field_length / 2.0 - field_description.dimensions.penalty_mark_distance;
                emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>(true, Eigen::Vector3d(x, 0.0, 0.0)));
            }
        });

        on<Trigger<StartPenalty>>().then([this] {
            // This emit starts the tree to play the penalty shootout
            emit<Task>(std::make_unique<Play>(), 1);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);
        });

        on<Provide<Play>, Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this] {
            if (cfg.is_goalie) {
                emit<Task>(std::make_unique<WalkToFieldPosition>());
            }
            else {
                emit<Task>(std::make_unique<FieldPlayer>());
            }
        });

        on<Trigger<Penalisation>, With<GlobalConfig>, With<GameState>>().then(
            [this](const Penalisation& self_penalisation,
                   const GlobalConfig& global_config,
                   const GameState& game_state) {
                // If the robot is penalised, it must stand still
                if (!cfg.force_playing && self_penalisation.context == GameEvents::Context::SELF) {
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   message::purpose::SoccerPosition::UNKNOWN,
                                                   false,
                                                   false,
                                                   game_state.team.team_colour));
                    emit(std::make_unique<Stability>(Stability::UNKNOWN));
                    emit(std::make_unique<ResetFieldLocalisation>());
                    emit<Task>(std::unique_ptr<Play>(nullptr));
                }
            });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // If the robot is unpenalised, stop standing still and find its purpose
            if (!cfg.force_playing && !idle && self_unpenalisation.context == GameEvents::Context::SELF) {
                emit<Task>(std::make_unique<Play>(), 1);
                emit<Task>(std::make_unique<FallRecovery>(), 2);
            }
        });

        // Left button pauses the soccer scenario
        on<Trigger<ButtonLeftDown>>().then([this] {
            emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>());
            emit<Scope::INLINE>(std::make_unique<EnableIdle>());
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            idle = true;
        });

        on<Trigger<ButtonLeftUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });

        on<Trigger<EnableIdle>>().then([this] {
            // Stop all tasks and stand still
            emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            emit<Task>(std::unique_ptr<FallRecovery>(nullptr));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            log<INFO>("Idle mode enabled");
        });

        // Middle button resumes the soccer scenario
        on<Trigger<ButtonMiddleDown>>().then([this] {
            emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>());
            // Restart the Director graph for the soccer scenario after a delay
            emit<Scope::DELAY>(std::make_unique<DisableIdle>(), std::chrono::seconds(cfg.disable_idle_delay));
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            idle = false;
        });

        on<Trigger<ButtonMiddleUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });

        on<Trigger<DisableIdle>, With<GameState>>().then([this](const GameState& game_state) {
            // If the robot is not idle nor penalised, restart the Director graph for the soccer scenario!
            if (!idle && game_state.self.penalty_reason == GameState::PenaltyReason::UNPENALISED) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                emit<Task>(std::make_unique<FallRecovery>(), 2);
                log<INFO>("Idle mode disabled");
            }
        });
    }

}  // namespace module::purpose
