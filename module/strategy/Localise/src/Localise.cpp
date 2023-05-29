#include "Localise.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/Localise.hpp"

#include "utility/skill/Script.hpp"

namespace module::strategy {

    using extension::Configuration;
    using LocaliseTask = message::strategy::Localise;
    using message::actuation::LimbsSequence;
    using message::localisation::Field;
    using utility::skill::load_script;

    Localise::Localise(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Localise.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Localise.yaml
            this->log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.confidence_threshold = config["confidence_threshold"].as<float>();
        });

        on<Provide<LocaliseTask>, Trigger<Field>>().then([this](const Field& field) {
            // If we are not confident (lower confidence value is better) in our position on the field, stand still.
            if (field.confidence > cfg.confidence_threshold) {
                log<NUClear::DEBUG>("Localisation confidence is not high enough, stand still.");
                emit<Task>(load_script<LimbsSequence>("Stand.yaml"));
            }
        });
    }

}  // namespace module::strategy
