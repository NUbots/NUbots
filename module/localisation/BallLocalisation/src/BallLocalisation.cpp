#include "BallLocalisation.h"

#include "message/support/Configuration.h"

namespace module {
namespace localisation {

    using message::support::Configuration;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("BallLocalisation.yaml").then([this] (const Configuration&) {
            // Use configuration here from file BallLocalisation.yaml
        });
    }
}
}
