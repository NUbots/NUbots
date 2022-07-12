#include "DeadReckonLocalisation.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/SimpleBall.hpp"
#include "message/support/nusight/DataPoint.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using SimpleBall  = message::localisation::SimpleBall;
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;
    using message::input::Sensors;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;
    using utility::support::Expression;

    DeadReckonLocalisation::DeadReckonLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        using message::localisation::SimpleBall;
        using utility::math::coordinates::reciprocalSphericalToCartesian;

        on<Configuration>("DeadReckonLocalisation.yaml")
            .then("DeadReckonLocalisation Config", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();

                // Set our initial estimate for theta
                theta                = config["initial_theta"].as<float>();
                filtered_theta       = config["initial_theta"].as<float>();
                cfg.smoothing_factor = config["smoothing_factor"].as<float>();
            });

        /* Runs on sensors updates */
        on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            /******************************** Goal Based Measurement ********************************/
            // If we have a single set of goals and it is currently within correct 180 degree region of our estimated
            // theta, run an opponent goal based update, assuming we can't see 2 sets of goal posts at a time, if we can
            // ignore

            float goal_theta_estimate = 0;

            // Dead reckon integral of gyro reading and kinematics based dtheta

            /******************************** Kinematics Based Measurement ********************************/

            float kinematics_theta_estimate = 0;

            /******************************** Gyro Based Measurement ********************************/
            log<NUClear::DEBUG>("Gyro (x,y,z): ", sensors.gyroscope.transpose());
            emit(graph("Gyro (x,y,z): ", sensors.gyroscope.x(), sensors.gyroscope.y(), sensors.gyroscope.z()));
            float deltaT              = 0.011111111;
            float gyro_dtheta         = deltaT * sensors.gyroscope.z();
            float gyro_theta_estimate = theta + gyro_dtheta;

            // cfg.smoothing_factor * goal_angles + (1 - cfg.smoothing_factor) * current_angles
            theta += gyro_dtheta;
            filtered_theta = cfg.smoothing_factor * theta + (1 - cfg.smoothing_factor) * filtered_theta;
            emit(graph("deltaT: ", deltaT));
            emit(graph("Theta: ", theta));
            emit(graph("filtered_theta: ", filtered_theta));
        });
    }

}  // namespace module::localisation
