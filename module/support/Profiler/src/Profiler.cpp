#include "Profiler.hpp"

#include "extension/Configuration.hpp"

#include "message/nuclear/LogMessage.hpp"
#include "message/nuclear/ReactionStatistics.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

    using message::support::nuclear::ReactionProfiles;

    Profiler::Profiler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Profiler.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<ReactionStatistics>>().then("Profiler", [this](const ReactionStatistics& stats) {
            // Compute the time this reaction took
            double time =
                1000.0
                * (double((stats.finished - stats.started).count()) / double(NUClear::clock::duration::period::den));

            // Check if we have a profile for this reaction
            if (reaction_profiles.find(stats.reaction_id) == reaction_profiles.end()) {
                // Add new profile
                reaction_profiles[stats.reaction_id]             = ReactionProfile();
                reaction_profiles[stats.reaction_id].reaction_id = stats.reaction_id;
                reaction_profiles[stats.reaction_id].name        = stats.identifiers.name;
                reaction_profiles[stats.reaction_id].reactor     = stats.identifiers.reactor;
            }

            // Update the profile for this reaction
            reaction_profiles[stats.reaction_id].total_time += time;
            reaction_profiles[stats.reaction_id].count++;
            reaction_profiles[stats.reaction_id].max_time =
                std::max(reaction_profiles[stats.reaction_id].max_time, time);
            reaction_profiles[stats.reaction_id].min_time =
                std::min(reaction_profiles[stats.reaction_id].min_time, time);
            reaction_profiles[stats.reaction_id].avg_time =
                reaction_profiles[stats.reaction_id].total_time / reaction_profiles[stats.reaction_id].count;

            // Compute the total time (sum of all reactions total time)
            double total_time_all = 0;
            for (auto& profile : reaction_profiles) {
                total_time_all += profile.second.total_time;
            }

            // Update all the profiles percentages based on new total time
            for (auto& profile : reaction_profiles) {
                profile.second.percentage = 100.0 * profile.second.total_time / total_time_all;
            }

            // Emit the profiles
            auto profiles = std::make_unique<ReactionProfiles>();
            for (auto& profile : reaction_profiles) {
                profiles->reaction_profiles.push_back(profile.second);
            }
            emit(profiles);
        });
    }
}  // namespace module::support
