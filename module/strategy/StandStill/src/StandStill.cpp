#include "StandStill.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/skill/Script.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::actuation::LimbsSequence;
    using message::behaviour::state::Stability;
    using message::skill::Walk;
    using utility::skill::load_script;
    using StandStillTask = message::strategy::StandStill;

    StandStill::StandStill(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StandStill.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StandStill.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<StandStillTask>, Uses<LimbsSequence>, Trigger<Stability>>().then(
            [this](const Stability& stability, const Uses<LimbsSequence>& limbs) {
                // If we are stable, then we can provide the StandStill command
                if (stability != Stability::STANDING) {
                    standing = false;
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3f::Zero()));
                }
                // If we aren't currently standing and we aren't currently running the stand, then request a stand
                else if (!standing && limbs.run_state == GroupInfo::RunState::NO_TASK) {
                    standing = true;
                    emit<Task>(load_script<LimbsSequence>("Stand.yaml"));
                }
            });
        on<Start<StandStillTask>>().then([this] {
            // We don't know that we are standing, so reset to false
            standing = false;
        });
    }

}  // namespace module::strategy
