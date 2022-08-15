#include "ReactionStats.hpp"

#include "extension/Configuration.hpp"

#include "message/support/nuclear/ReactionStatistics.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;
    using ReactionStatisticsProto = message::support::nuclear::ReactionStatistics;

    ReactionStats::ReactionStats(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ReactionStats.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReactionStats.yaml
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            auto reactionData               = std::make_unique<ReactionStatisticsProto>();
            reactionData->name              = stats.identifier[0];
            reactionData->trigger_name      = stats.identifier[1];
            reactionData->function_name     = stats.identifier[2];
            reactionData->reaction_id       = stats.reaction_id;
            reactionData->task_id           = stats.task_id;
            reactionData->cause_reaction_id = stats.cause_reaction_id;
            reactionData->cause_task_id     = stats.cause_task_id;
            reactionData->emitted           = stats.emitted.time_since_epoch().count();
            reactionData->started           = stats.started.time_since_epoch().count();
            reactionData->finished          = stats.finished.time_since_epoch().count();
            // TODO fix this
            /*reactionData.emitted           = getUtcTimestamp<std::chrono::microseconds>(stats.emitted);
            reactionData.started           = getUtcTimestamp<std::chrono::microseconds>(stats.started);
            reactionData.finished          = getUtcTimestamp<std::chrono::microseconds>(stats.finished);*/

            emit(reactionData);


            // log(stats.identifier[0],
            //     1000.0
            //         * (double((stats.finished - stats.started).count())
            //            / double(NUClear::clock::duration::period::den)),
            //     "ms");
        });
    }
}  // namespace module::support
