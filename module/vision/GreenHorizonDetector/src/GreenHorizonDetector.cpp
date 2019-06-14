#include "GreenHorizonDetector.h"

#include "extension/Configuration.h"

namespace module {
namespace vision {

    using extension::Configuration;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GreenHorizonDetector.yaml
        });
    }
}
}
