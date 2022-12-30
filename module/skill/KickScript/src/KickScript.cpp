#include "KickScript.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"
#include "extension/behaviour/Script.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/skill/Kick.hpp"

#include "utility/input/LimbID.hpp"

namespace module::skill {

    using extension::Configuration;
    using extension::behaviour::Script;
    using extension::behaviour::ScriptRequest;
    using message::actuation::LimbsSequence;
    using message::skill::Kick;
    using utility::input::LimbID;

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("kickscript.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickScript.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<Kick>, Needs<LimbsSequence>>().then([this](const Kick& kick, const RunInfo& info) {
            // If the script has finished executing, then this is Done
            if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                emit<Task>(std::make_unique<Done>());
                return;
            }
            // If it isn't a new task, don't do anything
            if (info.run_reason != RunInfo::RunReason::NEW_TASK) {
                emit<Task>(std::make_unique<Idle>());
                return;
            }

            if (kick.leg == LimbID::RIGHT_LEG) {
                emit<Script>(std::make_unique<LimbsSequence>(),
                             std::vector<ScriptRequest>{{"Stand.yaml"}, {"KickRight.yaml"}, {"Stand.yaml"}});
            }
            else {  // LEFT_LEG
                emit<Script>(std::make_unique<LimbsSequence>(),
                             std::vector<ScriptRequest>{{"Stand.yaml"}, {"KickLeft.yaml"}, {"Stand.yaml"}});
            }
        });
    }

}  // namespace module::skill
