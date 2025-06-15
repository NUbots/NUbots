#include "KickWalk.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/WalkState.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

#include "utility/nusight/NUhelpers.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::WalkState;
    using message::skill::Kick;
    using message::skill::Walk;
    using utility::nusight::graph;

    KickWalk::KickWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {
        log<INFO>("KickWalk module constructor called");
        on<Configuration>("KickWalk.yaml").then([this](const Configuration& config) {
            log<INFO>("KickWalk configuration loaded");
            // Use configuration here from file KickWalk.yaml
            this->log_level     = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_velocity_x = config["kick_velocity_x"].as<double>();
            cfg.kick_velocity_y = config["kick_velocity_y"].as<double>();
        });

        on<Provide<Kick>, With<WalkState>>().then(
            [this](const Kick& kick, const RunReason& run_reason, const WalkState& walk_state) {
                log<INFO>("KickWalk module received a kick request for leg: ", kick.leg);

                if (run_reason == RunReason::NEW_TASK) {
                    log<INFO>("KickWalk starting - checking foot phase for leg: ", kick.leg);

                    // Determine the optimal phase for the kick
                    WalkState::Phase optimal_phase;
                    if (kick.leg == LimbID::LEFT_LEG) {
                        optimal_phase = WalkState::Phase::RIGHT;  // Right foot is planted
                    }
                    else {
                        optimal_phase = WalkState::Phase::LEFT;  // Left foot is planted
                    }

                    // Guard condition: only proceed if in optimal phase
                    if (walk_state.phase != optimal_phase) {
                        log<DEBUG>("Wrong phase for kick - current: ",
                                   walk_state.phase,
                                   " needed: ",
                                   optimal_phase,
                                   " - not emitting Walk task");
                        return;  // Don't emit anything - PlanKick will retry
                    }

                    log<INFO>("Optimal phase detected - executing kick");

                    Eigen::Vector3d walk_velocity;
                    if (kick.leg == LimbID::RIGHT_LEG) {
                        walk_velocity = Eigen::Vector3d(cfg.kick_velocity_x, -cfg.kick_velocity_y, 0.0);
                    }
                    else {
                        walk_velocity = Eigen::Vector3d(cfg.kick_velocity_x, cfg.kick_velocity_y, 0.0);
                    }

                    emit<Task>(std::make_unique<Walk>(walk_velocity, true));
                }


                // If the walk says the kick is done, emit a Done task
                if (run_reason == RunReason::SUBTASK_DONE) {
                    log<INFO>("KickWalk step completed, kick done for leg: ", kick.leg);
                    emit<Task>(std::make_unique<Done>());
                    return;
                }
            });
    }

}  // namespace module::skill
