#include "ReactionTimer.h"

#include "extension/Configuration.h"

namespace module {
namespace support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

    ReactionTimer::ReactionTimer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ReactionTimer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReactionTimer.yaml
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            log(stats.identifier[0],
                1000.0
                    * (double((stats.finished - stats.started).count())
                       / double(NUClear::clock::duration::period::den)),
                "ms");
        });
    }
}  // namespace support
}  // namespace module
