#include "VisionBallLocalisation.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/SimpleBall.hpp"
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

    VisionBallLocalisation::VisionBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        using message::localisation::SimpleBall;
        using utility::math::coordinates::reciprocalSphericalToCartesian;

        on<Configuration>("VisionBallLocalisation.yaml")
            .then("VisionBallLocalisation Config", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();
                // Emit on config to ensure other modules relying on VisionBall run (CHECK IF ACTUALLY NEEDED)
                emit(std::make_unique<VisionBall>());
            });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                // Get the first vision ball measurement and treat as the closest measurement to our estimate
                Eigen::Vector3f rBCc =
                    reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                Eigen::Vector3f rBTt     = Htc * rBCc;
                auto raw_lowest_distance = std::sqrt(std::pow(rBTt.x(), 2) + std::pow(rBTt.y(), 2));

                // Loop through all our ball measurements and find the closest measurement to our current estimate
                for (const auto& ball : balls.balls) {
                    rBCc           = reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                    auto temp_rBTt = Htc * rBCc;
                    if (std::sqrt(std::pow(temp_rBTt.x(), 2) + std::pow(temp_rBTt.y(), 2)) < raw_lowest_distance) {
                        raw_lowest_distance = std::sqrt(std::pow(temp_rBTt.x(), 2) + std::pow(temp_rBTt.y(), 2));
                        rBTt                = temp_rBTt;
                    }
                }
                // TODO: Apply filter to rBTt

                // Generate message and emit
                auto time_of_measurment = NUClear::clock::now();
                float distance_to_ball  = std::sqrt(std::pow(rBTt.x(), 2) + std::pow(rBTt.y(), 2));
                float angle_to_ball     = std::abs(std::atan2(rBTt.y(), rBTt.x()));
                auto ball               = std::make_unique<SimpleBall>();
                ball->rBTt              = rBTt;
                ball->distance          = distance_to_ball;
                ball->angle             = angle_to_ball;
                emit(ball);

                // Logging
                log<NUClear::DEBUG>("rBTt: ", rBTt.transpose());
                log<NUClear::DEBUG>("Distance: ", distance_to_ball);
                log<NUClear::DEBUG>("Angle: ", angle_to_ball);
            }
        });
    }
}  // namespace module::localisation
