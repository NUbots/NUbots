#include "ReactionTimer.hpp"

#include "extension/Configuration.hpp"

#include "message/support/nuclear/ReactionProfile.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

    using ReactionProfileMsg = message::support::nuclear::ReactionProfile;

    ReactionTimer::ReactionTimer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ReactionTimer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReactionTimer.yaml
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            log(stats.identifiers.name,
                1000.0
                    * (double((stats.finished - stats.started).count())
                       / double(NUClear::clock::duration::period::den)),
                "ms");

            // Check if we have a profile for this reaction
            if (reaction_profiles.find(stats.identifier.name) == reaction_profiles.end()) {
                // Create a new profile
                reaction_profiles[stats.identifier.name]            = ReactionProfile();
                reaction_profiles[stats.identifier.name].identifier = stats.identifier.name;
            }

            // Update the profile
            reaction_profiles[stats.identifier.name].count++;
            reaction_profiles[stats.identifier.name].total_time += (stats.finished - stats.started).count();
            reaction_profiles[stats.identifier.name].avg_time =
                reaction_profiles[stats.identifier.name].total_time / reaction_profiles[stats.identifier.name].count;
            reaction_profiles[stats.identifier.name].max_time =
                std::max(reaction_profiles[stats.identifier.name].max_time, (stats.finished - stats.started).count());
            reaction_profiles[stats.identifier.name].min_time =
                std::min(reaction_profiles[stats.identifier.name].min_time, (stats.finished - stats.started).count());
            reaction_profiles[stats.identifier.name].avg_time =
                reaction_profiles[stats.identifier.name].total_time / reaction_profiles[stats.identifier.name].count;


            // Emit the profile
            auto profile        = std::make_unique<ReactionProfileMsg>();
            profile->identifier = stats.identifier;
            profile->total_time = reaction_profiles[stats.identifier.name].total_time;
            profile->count      = reaction_profiles[stats.identifier.name].count;
            profile->max_time   = reaction_profiles[stats.identifier.name].max_time;
            profile->min_time   = reaction_profiles[stats.identifier.name].min_time;
            profile->avg_time   = reaction_profiles[stats.identifier.name].avg_time;
            emit(profile);
        });
    }
}  // namespace module::support
