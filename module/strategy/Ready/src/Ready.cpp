#include "Ready.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/Ready.hpp"
// #include "message/strategy/StandStill.hpp"
#include "message/behaviour/state/Stability.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using ReadyTask = message::strategy::Ready;
    // using message::strategy::StandStill;
    using message::behaviour::state::Stability;

    Ready::Ready(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Ready.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ready.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.walk_to_ready_time = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["walk_to_ready_time"].as<double>()));
            cfg.walk_to_ready_speed_x  = config["walk_to_ready_speed_x"].as<double>();
            cfg.walk_to_ready_speed_y  = config["walk_to_ready_speed_y"].as<double>();
            cfg.walk_to_ready_rotation = config["walk_to_ready_rotation"].as<double>();
        });

        on<Provide<ReadyTask>, With<Stability>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const RunInfo& info, const Stability& stability) {
                if (info.run_reason == RunInfo::RunReason::NEW_TASK) {
                    // Set the timer and emit a walk Task
                    start_ready_time = NUClear::clock::now();
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(cfg.walk_to_ready_speed_x,
                                                                      cfg.walk_to_ready_speed_y,
                                                                      cfg.walk_to_ready_rotation)));
                }
                // If the time has elapsed to walk to ready, then emit the stand still task
                // Don't emit another stand still task if we already did so
                else if (NUClear::clock::now() - start_ready_time > cfg.walk_to_ready_time
                         && stability != Stability::STANDING) {
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                }
                else {  // Otherwise, emit the idle task to keep walking or standing still
                    emit<Task>(std::make_unique<Idle>());
                }
            });
    }

}  // namespace module::strategy
