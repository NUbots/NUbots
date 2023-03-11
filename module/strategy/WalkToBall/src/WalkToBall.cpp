#include "WalkToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToBall.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::localisation::FilteredBall;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using WalkToBallTask = message::strategy::WalkToBall;

    WalkToBall::WalkToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToBall.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
        });

        on<Provide<WalkToBallTask>, Optional<With<FilteredBall>>>().then(
            [this](const std::shared_ptr<const FilteredBall>& ball) {
                // If we have a ball, walk to it
                if (ball && (NUClear::clock::now() - ball->time_of_measurement < cfg.ball_search_timeout)) {
                    emit(std::make_unique<WalkTo>(ball->rBTt));
                }
                else {
                    // Otherwise, search for it
                    emit(std::make_unique<TurnOnSpot>(true));
                }
            });
    }

}  // namespace module::strategy
