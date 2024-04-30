/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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

            // Add new profile if it doesn't exist
            if (reaction_profiles.find(stats.reaction_id) == reaction_profiles.end()) {
                reaction_profiles[stats.reaction_id]             = ReactionProfile();
                reaction_profiles[stats.reaction_id].reaction_id = stats.reaction_id;
                reaction_profiles[stats.reaction_id].name        = stats.identifiers.name;
                reaction_profiles[stats.reaction_id].reactor     = stats.identifiers.reactor;
            }

            // Update the profile stats for this reaction
            reaction_profiles[stats.reaction_id].total_time += time;
            reaction_profiles[stats.reaction_id].count++;
            reaction_profiles[stats.reaction_id].max_time =
                std::max(reaction_profiles[stats.reaction_id].max_time, time);
            reaction_profiles[stats.reaction_id].min_time =
                std::min(reaction_profiles[stats.reaction_id].min_time, time);
            reaction_profiles[stats.reaction_id].avg_time =
                reaction_profiles[stats.reaction_id].total_time / reaction_profiles[stats.reaction_id].count;

            // Update the total time and count for all reactions
            total_time_all += time;
            total_count++;

            // Update all the profiles percentages based on new total time
            for (auto& profile : reaction_profiles) {
                profile.second.percentage = 100.0 * profile.second.total_time / total_time_all;
            }

            // Emit the updated profile
            auto profiles = std::make_unique<ReactionProfiles>();
            for (auto& profile : reaction_profiles) {
                profiles->reaction_profiles.push_back(profile.second);
            }
            profiles->total_time  = total_time_all;
            profiles->total_count = total_count;
            emit(profiles);
        });
    }
}  // namespace module::support
