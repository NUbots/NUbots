#include "Localise.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/Look.hpp"
#include "message/strategy/Localise.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/skill/Script.hpp"

namespace module::strategy {

    using extension::Configuration;

    using LocaliseTask = message::strategy::Localise;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::planning::LookAround;
    using message::strategy::StandStill;

    Localise::Localise(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Localise.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Localise.yaml
            this->log_level                  = config["log_level"].as<NUClear::LogLevel>();
            cfg.start_uncertainty_threshold  = config["start_uncertainty_threshold"].as<double>();
            cfg.resume_uncertainty_threshold = config["resume_uncertainty_threshold"].as<double>();
            cfg.max_lost_time                = config["max_lost_time"].as<double>();
            cfg.look_around_enabled          = config["look_around_enabled"].as<bool>();
        });

        on<Provide<LocaliseTask>, Trigger<Field>>().then([this](const Field& field) {
            // Compute how uncertain in our position and orientation on the field
            double uncertainty = field.covariance.trace();

            // If we are too uncertain and not already "lost", stand still and look around
            if ((!lost && uncertainty > cfg.start_uncertainty_threshold)) {
                // We have just become lost, store the time point
                lost            = true;
                time_point_lost = NUClear::clock::now();
            }

            // If we are already lost, check if we are still lost
            else if (lost && uncertainty > cfg.resume_uncertainty_threshold) {

                log<NUClear::DEBUG>("Localisation uncertainty is too high, stand still.");
                // Stop walking
                emit<Task>(std::make_unique<StandStill>());

                // Look around
                if (cfg.look_around_enabled) {
                    emit<Task>(std::make_unique<LookAround>());
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
                // We are not or no longer lost
                log<NUClear::DEBUG>("Localisation uncertainty is low enough.");
                lost = false;
            }
        });
    }

}  // namespace module::strategy
