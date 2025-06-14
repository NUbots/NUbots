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
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<Kick>>().then([this](const RunReason& run_reason) {
            // If the walk says the kick is done, return a Done task
            if (run_reason == RunReason::SUBTASK_DONE) {
                emit<Task>(std::make_unique<Done>());
                return;
            }
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(0.1, 0.0, 0.0), true));
        });
    }

}  // namespace module::skill
