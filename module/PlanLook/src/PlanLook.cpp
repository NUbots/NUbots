#include "PlanLook.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module {

    using extension::Configuration;

    PlanLook::PlanLook(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanLook.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanLook.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["search_timeout"].as<double>()));
            cfg.goal_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["search_timeout"].as<double>()));
            cfg.field_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["search_timeout"].as<double>()));
            cfg.fixation_time = config["fixation_time"].as<float>();

            // Create vector of search positions
            for (const auto& search_position : config["search_positions"].config) {
                cfg.search_positions.push_back(search_position.as<Expression>());
            }
        });

        on<Provide<FindFeatures>, Optional<With<FilteredBall>>, Optional<With<Goals>>>().then(
            [this](const FindFeatures& find_features,
                   const std::shared_ptr<const FilteredBall>& ball,
                   const std::shared_ptr<const Goals>& goals) {
                // if we have seen no ball or goals, search
                // if we haven't seen the ball or goals in a while, search
                // if it's been too long since a field search, search
                // if we're not looking for anything in particular, search
                // otherwise we've seen balls and goals and field, just keep looking at the ball

                // Do we need to look for the ball?
                bool search_ball =
                    find_features.ball
                    && (ball == nullptr
                        || (ball && NUClear::clock::now() - ball->time_of_measurement > cfg.ball_search_timeout));

                // Do we need to look for the goals?
                bool search_goals = find_features.goals
                                    && (goals == nullptr
                                        || (goals && !goals.goals.empty()
                                            && NUClear::clock::now() - goals->timestamp > cfg.goal_search_timeout));

                // Do we need to look at the field?
                // todo: wait long enough for one full search of the field
                bool search_field =
                    find_features.field && (NUClear::clock::now() - last_field_search > cfg.field_search_timeout);

                // If we're not looking for anything in particular, just search
                bool search_anything = !find_features.ball && !find_features.goals;

                // Either we need to search for the ball, goals or we're not looking for anything in particular so we
                // just search around in general
                if (search_ball || search_goals || search_field || search_anything) {
                    log<NUClear::TRACE>("Searching. Ball",
                                        search_ball,
                                        ": goals",
                                        search_goals,
                                        ": field",
                                        search_field,
                                        ": any",
                                        search_anything);
                    // If the search has completed one run of the pattern, then a field search has just been done
                    if (search_field && search_idx == cfg.search_positions.size()) {
                        last_field_search = NUClear::clock::now();
                    }
                    emit<Task>(std::make_unique<LookSearch>());
                }
                else {  // keep watching the ball
                    emit<Task>(std::make_unique<Look>(ball->rBCc, false))
                }
            });

        on<Provide<LookSearch>>().then([this] {
            // How long the look has lingered - will move to the next position if long enough
            float time_since_last_search_moved =
                std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - search_last_moved)
                    .count();

            // Robot will move through the search positions, and linger for fixation_time. Once
            // fixation_time time has passed, send a new head command for the next position in the list
            // of cfg.search_positions
            if (time_since_last_search_moved > cfg.search_fixation_time) {
                // Send command for look position
                Eigen::Vector3d rPCc = sphericalToCartesian(
                    Eigen::Vector3f(1, cfg.search_positions[search_idx][0], cfg.search_positions[search_idx][1]));
                emit<Task>(std::make_unique<LookAt>(rPCc, true));

                // Move to next search position in list
                search_last_moved = NUClear::clock::now();
                search_idx++;
            }

            // Reset the search position index if at end of list
            search_idx = search_idx == cfg.search_positions.size() ? 0 : search_index;
        });

        // When starting a LookSearch, start from the first look position
        on<Start<LookSearch>>().then([this] { search_idx = 0; });
    }

}  // namespace module
