#include "StandStill.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using StandStillTask = message::strategy::StandStill;

    StandStill::StandStill(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StandStill.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StandStill.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<StandStillTask>>().then([this](const RunInfo& info) {
            // If we haven't emitted yet, then emit a walk task
            if (info.run_reason == RunInfo::RunReason::NEW_TASK) {
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
            }
            else {
                emit<Task>(std::make_unique<Idle>());
            }
        });
    }

}  // namespace module::strategy
