#include "VisionBallLocalisation.hpp"

#include <chrono>

#include "extension/Configuration.hpp"

#include "message/localisation/VisionBall.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::localisation::VisionBall;

    using utility::nusight::graph;
    using utility::support::Expression;

    VisionBallLocalisation::VisionBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Configuration>("VisionBallLocalisation.yaml")
            .then("VisionBallLocalisation Config",
                  [this](const Configuration& config) { log_level = config["log_level"].as<NUClear::LogLevel>(); });

        /* To run whenever a ball has been detected */
        on<Trigger<message::vision::Balls>>().then([this](const message::vision::Balls& balls) {
            // If there are no balls, do nothing
            if (!balls.balls.empty()) {
                // Find the ball
            }
        });
    }
}  // namespace module::localisation
