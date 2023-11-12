#include "Profiler.hpp"

#include "extension/Configuration.hpp"

#include "message/support/nuclear/ReactionProfile.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

    using ReactionProfileMsg = message::support::nuclear::ReactionProfile;

    Profiler::Profiler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Profiler.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Profiler.yaml
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            // Check if we have a profile for this reaction
            if (reaction_profiles.find(stats.identifiers.name) == reaction_profiles.end()) {
                // Create a and a new profile
                reaction_profiles[stats.identifiers.name]      = ReactionProfile();
                reaction_profiles[stats.identifiers.name].name = stats.identifiers.name;
            }

            // Update the profile
            reaction_profiles[stats.identifiers.name].count++;
            reaction_profiles[stats.identifiers.name].total_time += time;
            reaction_profiles[stats.identifiers.name].avg_time =
                reaction_profiles[stats.identifiers.name].total_time / reaction_profiles[stats.identifiers.name].count;
            reaction_profiles[stats.identifiers.name].max_time =
                std::max(reaction_profiles[stats.identifiers.name].max_time, time);
            reaction_profiles[stats.identifiers.name].min_time =
                std::min(reaction_profiles[stats.identifiers.name].min_time, time);
            reaction_profiles[stats.identifiers.name].avg_time =
                reaction_profiles[stats.identifiers.name].total_time / reaction_profiles[stats.identifiers.name].count;


            // Emit the profile
            auto profile        = std::make_unique<ReactionProfileMsg>();
            profile->name       = stats.identifiers.name;
            profile->total_time = reaction_profiles[stats.identifiers.name].total_time;
            profile->count      = reaction_profiles[stats.identifiers.name].count;
            profile->max_time   = reaction_profiles[stats.identifiers.name].max_time;
            profile->min_time   = reaction_profiles[stats.identifiers.name].min_time;
            profile->avg_time   = reaction_profiles[stats.identifiers.name].avg_time;
            emit(profile);
        });
    }
}  // namespace module::support
