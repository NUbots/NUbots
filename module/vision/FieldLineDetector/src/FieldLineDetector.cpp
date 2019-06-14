#include "FieldLineDetector.h"

#include "extension/Configuration.h"

namespace module {
namespace vision {

    using extension::Configuration;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file FieldLineDetector.yaml
        });
    }
}
}
