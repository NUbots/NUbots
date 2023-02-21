#include "BallFilter.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/support/nusight/DataPoint.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using FilteredBall = message::localisation::FilteredBall;
    using VisionBalls  = message::vision::Balls;
    using VisionBall   = message::vision::Ball;

    using message::input::Sensors;
    using message::support::nusight::DataPoint;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;
    using utility::support::Expression;

    BallFilter::BallFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::localisation::FilteredBall;
        using utility::math::coordinates::reciprocalSphericalToCartesian;

        on<Configuration>("BallFilter.yaml").then([this](const Configuration& config) {
            log_level            = config["log_level"].as<NUClear::LogLevel>();
            cfg.smoothing_factor = config["smoothing_factor"].as<float>();
        });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {

                // Get the first vision ball measurement and treat it as the closest measurement to our estimate
                Eigen::Vector3f rBCc =
                    reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());
                float lowest_distance = (rBCc - filtered_rBCc).norm();

                // Loop through all our ball measurements and find the closest measurement to our current estimate
                for (const auto& ball : balls.balls) {
                    Eigen::Vector3f temp_rBCc =
                        reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                    if ((temp_rBCc - filtered_rBCc).norm() < lowest_distance) {
                        lowest_distance = (temp_rBCc - filtered_rBCc).norm();
                        rBCc            = temp_rBCc;
                    }
                }

                // Apply exponential filter to rBCc
                filtered_rBCc = cfg.smoothing_factor * rBCc + (1 - cfg.smoothing_factor) * filtered_rBCc;

                // Generate message and emit
                auto ball = std::make_unique<FilteredBall>();
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                ball->rBTt                = Htc * filtered_rBCc;
                ball->rBCc                = filtered_rBCc;
                ball->time_of_measurement = NUClear::clock::now();

                if (log_level <= NUClear::DEBUG) {
                    emit(graph("rBCc: ", filtered_rBCc.x(), filtered_rBCc.y(), filtered_rBCc.z()));
                    emit(graph("rBTt: ", ball->rBTt.x(), ball->rBTt.y(), ball->rBTt.z()));
                    log<NUClear::DEBUG>("rBCc: ", filtered_rBCc.transpose());
                    log<NUClear::DEBUG>("rBTt: ", ball->rBTt.transpose());
                }

                emit(ball);
            }
        });
    }

}  // namespace module::localisation
