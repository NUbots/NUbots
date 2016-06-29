#include "BallLocalisation.h"

#include "message/support/Configuration.h"
#include "message/localisation/FieldObject.h"

namespace module {
namespace localisation {

    using message::support::Configuration;
    using message::localisation::Ball;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("BallLocalisation.yaml").then([this] (const Configuration&) {
        	emit(std::make_unique<std::vector<Ball>>());
            // Use configuration here from file BallLocalisation.yaml
        });
    }
}
}
