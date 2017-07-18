#include "BonnWalk.h"

#include "extension/Configuration.h"

namespace module {
namespace motion {

    using extension::Configuration;

    BonnWalk::BonnWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("BonnWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file BonnWalk.yaml
        });
    }
}  // namespace motion
}  // namespace module
