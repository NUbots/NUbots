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
            cfg.look_around           = config["look_around"].as<bool>();
        });

        on<Provide<LocaliseTask>, Trigger<Field>>().then([this](const Field& field) {
            // Check if we are uncertain in our position and orientation on the field
            if (field.covariance.trace() > cfg.uncertainty_threshold) {
                log<NUClear::DEBUG>("Localisation uncertainty is too high, stand still.");

                // Stop walking
                emit<Task>(std::make_unique<StandStill>());

                // Look around
                if (cfg.look_around) {
                    emit<Task>(std::make_unique<LookAround>());
                }

                // If we have just become lost, store the time point
                if (!just_lost) {
                    time_point_lost = NUClear::clock::now();
                    just_lost       = true;
                }

                // Compute how long we have been lost for
                double time_since_lost =
                    std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - time_point_lost)
                        .count()
                    * 1e-6;
                log<NUClear::DEBUG>("Time since lost: {}", time_since_lost);

                // If we have been lost for too long, reset localisation
                if (time_since_lost > cfg.max_lost_time) {
                    log<NUClear::DEBUG>("Lost for too long, reset localisation.");
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
