#include "BallFilter.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/support/FieldDescription.hpp"
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
    using message::support::FieldDescription;

    using message::eye::DataPoint;
    using message::input::Sensors;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;
    using utility::support::Expression;

    BallFilter::BallFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::localisation::FilteredBall;
        using utility::math::coordinates::reciprocalSphericalToCartesian;

        on<Configuration>("BallFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
            // Set our measurement noise
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2f(config["ukf"]["noise"]["measurement"]["ball_position"].as<Expression>()).asDiagonal();

            // Set our process noises
            cfg.ukf.noise.process.position = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();

            // Set our motion model's process noise
            BallModel<float>::StateVec process_noise;
            process_noise.rBWw      = cfg.ukf.noise.process.position;
            process_noise.vBw       = cfg.ukf.noise.process.velocity;
            ukf.model.process_noise = process_noise;

            // Set our initial mean
            cfg.ukf.initial.mean.position = config["ukf"]["initial"]["mean"]["position"].as<Expression>();
            cfg.ukf.initial.mean.velocity = config["ukf"]["initial"]["mean"]["velocity"].as<Expression>();

            // Set out initial covariance
            cfg.ukf.initial.covariance.position = config["ukf"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.ukf.initial.covariance.velocity = config["ukf"]["initial"]["covariance"]["velocity"].as<Expression>();

            // Set our initial state with the config means and covariances, flagging the filter to reset it
            cfg.initial_mean.rBWw = cfg.ukf.initial.mean.position;
            cfg.initial_mean.vBw  = cfg.ukf.initial.mean.velocity;

            cfg.initial_covariance.rBWw = cfg.ukf.initial.covariance.position;
            cfg.initial_covariance.vBw  = cfg.ukf.initial.covariance.velocity;
            ukf.set_state(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal());

            last_time_update = NUClear::clock::now();
        });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<Sensors>, With<FieldDescription>>().then(
            [this](const VisionBalls& balls, const Sensors& sensors, const FieldDescription& fd) {
                if (!balls.balls.empty()) {
                    Eigen::Isometry3f Hcw = Eigen::Isometry3f(balls.Hcw.cast<float>());
                    Eigen::Isometry3f Hwc = Hcw.inverse();
                    auto state            = BallModel<float>::StateVec(ukf.get_state());

                    // Get the first ball measurement and treat it as the closest measurement to our current estimate
                    Eigen::Vector3f rBWw =
                        Hwc * reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());
                    float lowest_distance = (rBWw.head<2>() - state.rBWw).squaredNorm();

                    // Loop through all our ball measurements and find the closest measurement to our current estimate
                    for (const auto& ball : balls.balls) {
                        Eigen::Vector3f temp_rBWw =
                            Hwc * reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                        float temp_distance = (temp_rBWw.head<2>() - state.rBWw).squaredNorm();
                        if (temp_distance < lowest_distance) {
                            lowest_distance = temp_distance;
                            rBWw            = temp_rBWw;
                        }
                    }

                    // Compute the time since the last update (in seconds)
                    const auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now()
                                                                                             - last_time_update)
                                        .count();
                    last_time_update = NUClear::clock::now();

                    // Time update
                    ukf.time(dt);

                    // Measurement update
                    ukf.measure(Eigen::Vector2f(rBWw.head<2>()),
                                cfg.ukf.noise.measurement.position,
                                MeasurementType::BALL_POSITION());

                    // Get the new state, here we are assuming ball is on the ground
                    state = BallModel<float>::StateVec(ukf.get_state());
                    rBWw << state.rBWw.x(), state.rBWw.y(), fd.ball_radius;

                    // Generate and emit message
                    auto ball = std::make_unique<FilteredBall>();
                    Eigen::Isometry3f Htw(sensors.Htw.cast<float>());
                    Eigen::Isometry3f Hrw(sensors.Hrw.cast<float>());
                    Eigen::Isometry3f Htc     = Eigen::Isometry3f(sensors.Htw.cast<float>()) * Hwc;
                    ball->rBWw                = rBWw;
                    ball->rBTt                = Htw * rBWw;
                    ball->rBRr                = Hrw * rBWw;
                    ball->rBCt                = Htc.rotation() * (Hcw * rBWw);
                    ball->time_of_measurement = last_time_update;

                    if (log_level <= NUClear::DEBUG) {
                        log<NUClear::DEBUG>("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z());
                        log<NUClear::DEBUG>("rBTt: ", ball->rBTt.x(), ball->rBTt.y(), ball->rBTt.z());
                        log<NUClear::DEBUG>("rBRr: ", ball->rBRr.x(), ball->rBRr.y(), ball->rBRr.z());
                        log<NUClear::DEBUG>("rBCt: ", ball->rBCt.x(), ball->rBCt.y(), ball->rBCt.z());
                        emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                        emit(graph("rBTt: ", ball->rBTt.x(), ball->rBTt.y(), ball->rBTt.z()));
                        emit(graph("rBRr: ", ball->rBRr.x(), ball->rBRr.y(), ball->rBRr.z()));
                        emit(graph("rBCt: ", ball->rBCt.x(), ball->rBCt.y(), ball->rBCt.z()));
                    }

                    emit(ball);
                }
            });
    }

}  // namespace module::localisation
