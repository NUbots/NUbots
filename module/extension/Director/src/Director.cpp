#include "Director.h"

#include "extension/Configuration.h"

namespace module {
namespace extension {

    using extension::Configuration;

    Director::Director(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Director.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Director.yaml
        });
    }
}
}
