#include "KickWalk.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::skill::Kick;
    using message::skill::Walk;

    KickWalk::KickWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {
        on<Configuration>("KickWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickWalk.yaml
            this->log_level              = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_approach_velocity_x = config["kick_approach_velocity_x"].as<double>();
            cfg.kick_approach_velocity_y = config["kick_approach_velocity_y"].as<double>();
        });

        on<Provide<Kick>, Uses<Walk>>().then(
            [this](const Kick& kick, const RunReason& run_reason, const Uses<Walk>& walk) {
                if (run_reason == RunReason::NEW_TASK) {
                    if (NUClear::clock::now() - last_kick_end < std::chrono::seconds(2)) {
                        log<DEBUG>("KickWalk is waiting for cooldown after last kick.");
                        NUClear::clock::time_point wait_until = last_kick_end + std::chrono::seconds(2);
                        emit<Task>(std::make_unique<Wait>(wait_until));
                        return;
                    }

                    log<DEBUG>("KickWalk module received a kick request for leg: ", kick.leg);

                    // Slow approach
                    Eigen::Vector3d walk_velocity =
                        Eigen::Vector3d(cfg.kick_approach_velocity_x, cfg.kick_approach_velocity_y, 0.0);
                    emit<Task>(std::make_unique<Walk>(walk_velocity, true, kick.leg));
                }
                // The wait triggered the provider
                else if (run_reason == RunReason::SUBTASK_DONE && !walk.done) {
                    // Slow approach
                    log<DEBUG>("KickWalk module received a kick request for leg: ", kick.leg);
                    Eigen::Vector3d walk_velocity =
                        Eigen::Vector3d(cfg.kick_approach_velocity_x, cfg.kick_approach_velocity_y, 0.0);
                    emit<Task>(std::make_unique<Walk>(walk_velocity, true, kick.leg));
                }

                // If the walk says the kick is done, emit a Done task
                if (run_reason == RunReason::SUBTASK_DONE) {
                    log<DEBUG>("KickWalk step completed, kick done for leg: ", kick.leg);
                    last_kick_end = NUClear::clock::now();
                    emit<Task>(std::make_unique<Done>());
                    return;
                }
            });
    }

}  // namespace module::skill
