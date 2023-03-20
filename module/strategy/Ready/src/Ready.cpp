#include "Ready.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using ReadyTask = message::strategy::Ready;
    using message::strategy::StandStill;

    Ready::Ready(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Ready.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ready.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.walk_to_ready_time = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["walk_to_ready_time"].as<double>()));
            cfg.walk_to_ready_speed_x  = config["walk_to_ready_speed_x"].as<float>();
            cfg.walk_to_ready_speed_y  = config["walk_to_ready_speed_y"].as<float>();
            cfg.walk_to_ready_rotation = config["walk_to_ready_rotation"].as<float>();
        });

        on<Provide<ReadyTask>, Uses<StandStill>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const RunInfo& info, const Uses<StandStill>& stand_still) {
                // If we have just started running ready, then record the current time and emit the walk task
                if (info.run_reason == RunInfo::NEW_TASK) {
                    start_ready_time = NUClear::clock::now();
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3f(cfg.walk_to_ready_speed_x,
                                                                      cfg.walk_to_ready_speed_y,
                                                                      cfg.walk_to_ready_rotation)));
                }
                // If the time has elapsed to walk to ready, then emit the stand still task
                // Don't emit another stand still task if we already did so
                else if (NUClear::clock::now() - start_ready_time > cfg.walk_to_ready_time
                         && stand_still.run_state == GroupInfo::RunState::NO_TASK) {
                    emit<Task>(std::make_unique<StandStill>());
                }
                else {  // Otherwise, emit the idle task to keep walking
                    emit<Task>(std::make_unique<Idle>());
                }
            });
    }

}  // namespace module::strategy
