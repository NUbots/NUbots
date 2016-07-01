#include "LegLoadsLogger.h"

#include "extension/Configuration.h"

namespace module {
namespace support {

    using extension::Configuration;

    LegLoadsLogger::LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("LegLoadsLogger.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file LegLoadsLogger.yaml
        });
    }
}
}
