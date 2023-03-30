#include "FindObject.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/LookAround.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/FindFeature.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::planning::LookAround;
    using message::planning::TurnOnSpot;
    using message::strategy::FindBall;

    FindObject::FindObject(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FindObject.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FindObject.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<FindBall>>().then([this] {
            emit<Task>(std::make_unique<LookAround>());
            emit<Task>(std::make_unique<TurnOnSpot>());
        });
    }

}  // namespace module::strategy
