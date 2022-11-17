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

                // Get the transform from camera {c} to torso space {t}
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());

                // Get the first vision ball measurement and treat it as the closest measurement to our estimate
                Eigen::Vector3f srBCc = balls.balls[0].measurements[0].srBCc.cast<float>();
                Eigen::Vector3f rBCc  = reciprocalSphericalToCartesian(srBCc);
                Eigen::Vector3f rBTt  = Htc * rBCc;
                float lowest_distance = get_distance(rBTt - filtered_rBTt);

                // Loop through all our ball measurements and find the closest measurement to our current estimate
                for (const auto& ball : balls.balls) {
                    rBCc = reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                    Eigen::Vector3f temp_rBTt = Htc * rBCc;
                    if (get_distance(temp_rBTt - filtered_rBTt) < lowest_distance) {
                        lowest_distance = get_distance(temp_rBTt - filtered_rBTt);
                        rBTt            = temp_rBTt;
                        srBCc           = ball.measurements[0].srBCc.cast<float>();
                    }
                }

                // Apply exponential filter to rBTt
                filtered_rBTt = cfg.smoothing_factor * rBTt + (1.0 - cfg.smoothing_factor) * filtered_rBTt;

                if (log_level <= NUClear::DEBUG) {
                    emit(graph("rBTt: ", rBTt.x(), rBTt.y(), rBTt.z()));
                    emit(graph("filtered_rBTt: ", filtered_rBTt.x(), filtered_rBTt.y(), filtered_rBTt.z()));
                    log<NUClear::DEBUG>("rBTt: ", rBTt.transpose());
                    log<NUClear::DEBUG>("filtered_rBTt: ", filtered_rBTt.transpose());
                }

                // Generate message and emit
                auto ball                 = std::make_unique<FilteredBall>();
                ball->rBTt                = filtered_rBTt;
                ball->srBCc               = srBCc;
                ball->time_of_measurement = NUClear::clock::now();
                ball->distance            = get_distance(filtered_rBTt);
                float absolute_yaw_angle  = std::abs(std::atan2(filtered_rBTt.y(), filtered_rBTt.x()));
                ball->absolute_yaw_angle  = absolute_yaw_angle;
                emit(ball);
            }
        });
    }
    float BallFilter::get_distance(Eigen::Matrix<float, 3, 1> v) {
        return std::sqrt(std::pow(v.x(), 2) + std::pow(v.y(), 2));
    }

}  // namespace module::localisation
