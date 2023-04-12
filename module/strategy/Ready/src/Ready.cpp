#include "Ready.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
// #include "message/strategy/StandStill.hpp"
#include "message/behaviour/state/Stability.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using ReadyTask = message::strategy::Ready;
    // using message::strategy::StandStill;
    using message::behaviour::state::Stability;
    using WalkToFieldPosition = message::strategy::WalkToFieldPosition;

    Ready::Ready(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Ready.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ready.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<ReadyTask>, With<Stability>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const RunInfo& info, const Stability& stability) {
                // Create walk to field position message
                auto walk_to_ready(std::make_unique<WalkToFieldPosition>());
                walk_to_ready->rPFf  = Eigen::Vector3f(0, 0, 0);
                walk_to_ready->theta = 0;

                log<NUClear::DEBUG>("Walk to ready field position");

                // If we are stable, walk to the ready field position
                emit<Task>(walk_to_ready);
            });
    }

}  // namespace module::strategy
