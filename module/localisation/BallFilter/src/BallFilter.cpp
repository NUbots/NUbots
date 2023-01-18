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
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Define the state transition model
            Eigen::Matrix3f A = Eigen::Matrix3f::Zero();

            // Define the control model
            Eigen::MatrixXf B;

            // Define the measurement model
            Eigen::Matrix3f C = Eigen::Matrix3f::Identity();

            // Define the process noise covariance
            float process_noise = config["filter"]["process_noise"].as<Expression>();
            Eigen::Matrix3f Q   = process_noise * Eigen::Matrix3f::Identity();

            // Define the measurement noise covariance
            float measurement_noise = config["filter"]["measurement_noise"].as<Expression>();
            Eigen::Matrix3f R       = measurement_noise * Eigen::Matrix3f::Identity();

            // Update the models in the filter
            filter.update_model(A, B, C, Q, R);

            // Initialise the filter with the initial state and covariance
            Eigen::Vector3f x        = config["filter"]["initial_state"]["state"].as<Expression>();
            float initial_covariance = config["filter"]["initial_state"]["covariance"].as<Expression>();
            Eigen::Matrix3f P        = initial_covariance * Eigen::Matrix3f::Identity();
            filter.init(x, P);

            // Set the filter last update time
            last_update_time = NUClear::clock::now();
        });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                // Get the first vision ball measurement and treat it as the closest measurement to our estimate
                Eigen::Vector3f rBCc =
                    reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());
                float lowest_distance = (rBCc - filter.get_state()).norm();

                // Loop through all our ball measurements and find the closest measurement to our current estimate
                for (const auto& ball : balls.balls) {
                    Eigen::Vector3f temp_rBCc =
                        reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                    if ((temp_rBCc - filter.get_state()).norm() < lowest_distance) {
                        lowest_distance = (temp_rBCc - filter.get_state()).norm();
                        rBCc            = temp_rBCc;
                    }
                }

                // Compute the time since the last measurement in seconds
                float dt         = (NUClear::clock::now() - last_update_time).count() * 1e-9;
                last_update_time = NUClear::clock::now();

                // Run the Kalman filter
                auto u = Eigen::Matrix<float, 0, 1>::Zero();  // No control input
                auto y = rBCc;
                filter.run(u, y, dt);


                // Generate message and emit
                auto ball = std::make_unique<FilteredBall>();
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                ball->rBTt                = Htc * filter.get_state();
                ball->rBCc                = filter.get_state();
                ball->time_of_measurement = NUClear::clock::now();

                if (log_level <= NUClear::DEBUG) {
                    emit(graph("rBCc: ", filter.get_state().x(), filter.get_state().y(), filter.get_state().z()));
                    emit(graph("rBTt: ", ball->rBTt.x(), ball->rBTt.y(), ball->rBTt.z()));
                    log<NUClear::DEBUG>("rBCc: ", filter.get_state().transpose());
                    log<NUClear::DEBUG>("rBTt: ", ball->rBTt.transpose());
                }

                emit(ball);
            }
        });
    }

}  // namespace module::localisation
