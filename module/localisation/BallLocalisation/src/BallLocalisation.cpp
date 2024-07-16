/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "BallLocalisation.hpp"

#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using Ball        = message::localisation::Ball;
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;
    using message::support::FieldDescription;

    using message::eye::DataPoint;

    using utility::nusight::graph;
    using utility::support::Expression;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        using message::localisation::Ball;

        on<Configuration>("BallLocalisation.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
            // Set our measurement noise
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["ball_position"].as<Expression>()).asDiagonal();

            // Set our process noises
            cfg.ukf.noise.process.position = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();

            // Set our motion model's process noise
            BallModel<double>::StateVec process_noise;
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

            // Set our acceptance radius
            cfg.acceptance_radius = config["acceptance_radius"].as<double>();

            // Set our rejection count
            cfg.max_rejections = config["max_rejections"].as<int>();

            last_time_update = NUClear::clock::now();
        });


        on<Trigger<VisionBalls>, With<FieldDescription>>().then(
            [this](const VisionBalls& balls, const FieldDescription& fd) {
                if (!balls.balls.empty()) {
                    Eigen::Isometry3d Hwc = Eigen::Isometry3d(balls.Hcw.cast<double>()).inverse();

                    // Process all ball measurements
                    for (const auto& ball : balls.balls) {
                        Eigen::Vector3d rBWw     = Hwc * ball.measurements[0].rBCc.cast<double>();
                        Eigen::Vector2d ball_pos = rBWw.head<2>();

                        // Add the measurement to recent_measurements
                        recent_measurements.emplace_back(ball_pos, NUClear::clock::now());
                    }

                    // Remove old measurements
                    auto now = NUClear::clock::now();
                    recent_measurements.erase(
                        std::remove_if(
                            recent_measurements.begin(),
                            recent_measurements.end(),
                            [&](const auto& m) {
                                return std::chrono::duration_cast<std::chrono::duration<double>>(now - m.second).count()
                                       > cfg.cluster_time_window;
                            }),
                        recent_measurements.end());

                    if (!ball_estimate_initialized) {
                        cluster_and_initialize();
                    }

                    if (ball_estimate_initialized) {
                        update_ball_estimate(balls, fd);
                    }
                }
            });
    }

    void BallLocalisation::cluster_and_initialize() {
        if (recent_measurements.size() < cfg.required_measurements) {
            return;
        }

        // Simple clustering algorithm
        std::vector<std::vector<Eigen::Vector2d>> clusters;
        for (const auto& measurement : recent_measurements) {
            bool added_to_cluster = false;
            for (auto& cluster : clusters) {
                if ((cluster[0] - measurement.first).norm() < cfg.cluster_distance_threshold) {
                    cluster.push_back(measurement.first);
                    added_to_cluster = true;
                    break;
                }
            }
            if (!added_to_cluster) {
                clusters.push_back({measurement.first});
            }
        }

        // Check if any cluster has enough measurements
        for (const auto& cluster : clusters) {
            if (cluster.size() >= cfg.required_measurements) {
                // Initialize the ball estimate with the average position of the cluster
                Eigen::Vector2d avg_pos = Eigen::Vector2d::Zero();
                for (const auto& pos : cluster) {
                    avg_pos += pos;
                }
                avg_pos /= cluster.size();

                // Set the initial state and covariance
                BallModel<double>::StateVec initial_state;
                initial_state.rBWw = avg_pos;
                initial_state.vBw  = Eigen::Vector2d::Zero();  // Initialize velocity to zero

                ukf.set_state(initial_state.getStateVec(), cfg.initial_covariance.asDiagonal());

                ball_estimate_initialized = true;
                log<NUClear::INFO>("Ball estimate initialized at position: ", avg_pos.x(), ", ", avg_pos.y());
                break;
            }
        }
    }

    void BallLocalisation::update_ball_estimate(const VisionBalls& balls, const FieldDescription& fd) {
        auto state            = BallModel<double>::StateVec(ukf.get_state());
        Eigen::Isometry3d Hwc = Eigen::Isometry3d(balls.Hcw.cast<double>()).inverse();

        // Data association: find the ball closest to our current estimate
        Eigen::Vector3d rBWw   = Eigen::Vector3d::Zero();
        double lowest_distance = std::numeric_limits<double>::max();
        for (const auto& ball : balls.balls) {
            Eigen::Vector3d current_rBWw = Hwc * ball.measurements[0].rBCc.cast<double>();
            double current_distance      = (current_rBWw.head<2>() - state.rBWw).squaredNorm();
            if (current_distance < lowest_distance) {
                lowest_distance = current_distance;
                rBWw            = current_rBWw;
            }
        }

        // Data association: ensure the ball is within the acceptance radius
        bool accept_ball = true;
        if (lowest_distance > cfg.acceptance_radius && !first_ball_seen) {
            accept_ball = false;
            rejection_count++;
        }
        else {
            first_ball_seen = true;
            rejection_count = 0;
        }
        log<NUClear::DEBUG>("Rejection count: ", rejection_count);
        log<NUClear::DEBUG>("Accept ball: ", accept_ball);

        // Data association: if we have rejected too many balls, accept the closest one
        if (rejection_count > cfg.max_rejections) {
            accept_ball     = true;
            rejection_count = 0;
        }

        if (accept_ball) {
            // Compute the time since the last update (in seconds)
            const auto dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_time_update)
                    .count();
            last_time_update = NUClear::clock::now();

            // Time update
            ukf.time(dt);

            // Measurement update
            ukf.measure(Eigen::Vector2d(rBWw.head<2>()),
                        cfg.ukf.noise.measurement.position,
                        MeasurementType::BALL_POSITION());

            // Get the new state, here we are assuming ball is on the ground
            state = BallModel<double>::StateVec(ukf.get_state());

            // Generate and emit message
            auto ball                 = std::make_unique<Ball>();
            ball->rBWw                = Eigen::Vector3d(state.rBWw.x(), state.rBWw.y(), fd.ball_radius);
            ball->vBw                 = Eigen::Vector3d(state.vBw.x(), state.vBw.y(), 0);
            ball->time_of_measurement = last_time_update;
            ball->Hcw                 = balls.Hcw;
            if (log_level <= NUClear::DEBUG) {
                log<NUClear::DEBUG>("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z());
                log<NUClear::DEBUG>("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z());
                emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                emit(graph("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z()));
            }

            emit(ball);
        }
    }

}  // namespace module::localisation
