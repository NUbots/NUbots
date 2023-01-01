#include "PlanWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::planning {

    using extension::Configuration;

    PlanWalkPath::PlanWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanWalkPath.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanWalkPath.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Walking to a particular point
        on<Provide<WalkTo>>().then([this](const WalkTo& walk_to) {

        });

        // Walking around a fixed point so you are aligned in a different direction
        on<Provide<WalkAround>>().then([this](const WalkAround& walk_around) {

        });

        // Note: walk to ready is not here, in favour of just emitting a Walk task directly from the strategy for ready
        // state - then the config values can be in that module, which is where ppl are more likely to look for them
    }

}  // namespace module::planning
