#include "KickScript.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::motion {

    using extension::Configuration;

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("KickScript.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickScript.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<Kick>, Needs<LimbsSequence>>().then([this](const Kick& kick, const RunInfo& info) {
            // If the script has finished executing, then this is Done
            if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                emit<Task>(std::make_unique<Done>());
                return;
            }

            LimbID leg = kick_command->leg;

            // Execute the penalty kick if the type is PENALTY
            if (kick->type == KickCommandType::PENALTY) {
                emit<Script>(std::make_unique<ExecuteScriptByName>("KickPenalty.yaml"));
            }
            else {
                if (leg == LimbID::RIGHT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        subsumption_id,
                        std::vector<std::string>({"Stand.yaml", "KickRight.yaml", "Stand.yaml"})));
                }
                else {  // LEFT_LEG
                    emit(std::make_unique<ExecuteScriptByName>(
                        subsumption_id,
                        std::vector<std::string>({"Stand.yaml", "KickLeft.yaml", "Stand.yaml"})));
                }
            }
        });
    }

}  // namespace module::motion
