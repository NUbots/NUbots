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

        on<Startup>().then([this] {
            // Record the start time
            start_time = NUClear::clock::now();
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            double time =
                1000.0
                * (double((stats.finished - stats.started).count()) / double(NUClear::clock::duration::period::den));

            // Check if we have a profile for this reaction
            if (reaction_profiles.find(stats.reaction_id) == reaction_profiles.end()) {
                // Add new profile
                reaction_profiles[stats.reaction_id]             = ReactionProfile();
                reaction_profiles[stats.reaction_id].reaction_id = stats.reaction_id;
            }

            // Update the profile
            reaction_profiles[stats.reaction_id].total_time += time;
            reaction_profiles[stats.reaction_id].count++;
            reaction_profiles[stats.reaction_id].max_time =
                std::max(reaction_profiles[stats.reaction_id].max_time, time);
            reaction_profiles[stats.reaction_id].min_time =
                std::min(reaction_profiles[stats.reaction_id].min_time, time);
            reaction_profiles[stats.reaction_id].avg_time =
                reaction_profiles[stats.reaction_id].total_time / reaction_profiles[stats.reaction_id].count;

            // Compute the total time of all the reactions since the start of the profiler
            double total_time_all = 0;
            for (auto& profile : reaction_profiles) {
                total_time_all += profile.second.total_time;
            }

            reaction_profiles[stats.reaction_id].percentage =
                100.0 * reaction_profiles[stats.reaction_id].total_time / total_time_all;

            // Emit the profile
            auto profile = std::make_unique<ReactionProfileMsg>();
            if (stats.identifiers.name != "") {
                profile->name = stats.identifiers.name;
            }
            else {
                profile->name = stats.identifiers.dsl;
            }
            profile->reactor     = stats.identifiers.reactor;
            profile->reaction_id = stats.reaction_id;
            profile->total_time  = reaction_profiles[stats.reaction_id].total_time;
            profile->count       = reaction_profiles[stats.reaction_id].count;
            profile->max_time    = reaction_profiles[stats.reaction_id].max_time;
            profile->min_time    = reaction_profiles[stats.reaction_id].min_time;
            profile->avg_time    = reaction_profiles[stats.reaction_id].avg_time;
            profile->percentage  = reaction_profiles[stats.reaction_id].percentage;
            emit(profile);
        });
    }
}  // namespace module::support
