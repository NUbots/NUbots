#include "Dive.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/skill/Dive.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using utility::skill::load_script;
    using DiveTask = message::skill::Dive;

    Dive::Dive(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Dive.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Dive.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<DiveTask>>().then([this](const DiveTask& dive) {
            if (dive.side == BodySide::LEFT) {
                emit<Task>(load_script<BodySequence>("DiveLeft.yaml"));
            }
            else {
                emit<Task>(load_script<BodySequence>("DiveRight.yaml"));
            }
        });
    }

}  // namespace module::skill
