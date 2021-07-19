#include "ReactionStats.hpp"

#include "extension/Configuration.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

    ReactionStats::ReactionStats(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ReactionStats.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReactionStats.yaml
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            log(stats.identifier[0],
                1000.0
                    * (double((stats.finished - stats.started).count())
                       / double(NUClear::clock::duration::period::den)),
                "ms");
        });
    }
}  // namespace module::support
