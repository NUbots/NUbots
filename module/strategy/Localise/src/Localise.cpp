#include "Localise.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/Look.hpp"
#include "message/strategy/Localise.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/skill/Script.hpp"

namespace module::strategy {

    using extension::Configuration;
    using LocaliseTask = message::strategy::Localise;
    using message::actuation::LimbsSequence;
    using message::localisation::AddNoiseToParticles;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::planning::LookAround;
    using message::skill::Look;
    using message::strategy::StandStill;
    using utility::skill::load_script;

    Localise::Localise(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Localise.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Localise.yaml
            this->log_level           = config["log_level"].as<NUClear::LogLevel>();
            cfg.uncertainty_threshold = config["uncertainty_threshold"].as<double>();
            cfg.max_lost_time         = config["max_lost_time"].as<double>();
        });

        on<Provide<LocaliseTask>, Trigger<Field>>().then([this](const Field& field) {
            // If we are uncertain on our position on the field, stand still and look around
            if (field.covariance.trace() > cfg.uncertainty_threshold) {
                log<NUClear::DEBUG>("Localisation uncertainty is not high enough, stand still.");

                // Stop walking
                emit<Task>(std::make_unique<StandStill>());

                // Look around
                emit<Task>(std::make_unique<LookAround>());

                // Add extra noise to the particles
                emit(std::make_unique<AddNoiseToParticles>());

                // Increment the time since we have
                if (!just_lost) {
                    time_point_lost = NUClear::clock::now();
                    just_lost       = true;
                }
                double time_since_lost =
                    std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - time_point_lost)
                        .count()
                    * 1e-6;
                log<NUClear::DEBUG>("Time since lost: {}", time_since_lost);
                if (time_since_lost > cfg.max_lost_time) {
                    log<NUClear::INFO>("Lost for too long, reset localisation.");
                    time_point_lost = NUClear::clock::now();
                    emit(std::make_unique<ResetFieldLocalisation>());
                }
            }
            else {
                just_lost = false;
            }
        });
    }

}  // namespace module::strategy
