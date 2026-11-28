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
#include <algorithm>
#include <chrono>
#include <limits>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Cooperative.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/GroundTruthMode.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;

    using Ball        = message::localisation::Ball;
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;

    using message::eye::DataPoint;
    using message::localisation::CooperativeBall;
    using message::localisation::Field;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::nusight::graph;
    using utility::support::Expression;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        using message::localisation::Ball;
        using message::localisation::GroundTruthMode;

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

            cfg.max_distance_from_field = config["max_distance_from_field"].as<double>();

            cfg.use_cooperative_ball     = config["use_cooperative_ball"].as<bool>();
            cfg.cooperative_ball_timeout = config["cooperative_ball_timeout"].as<double>();
            cfg.cooperative_ball_max_age = config["cooperative_ball_max_age"].as<double>();

            last_time_update = NUClear::clock::now();
        });

        // While ground truth perception is on, Ball comes from the simulator instead, so stop our own reactions
        on<Trigger<GroundTruthMode>>().then([this](const GroundTruthMode& mode) {
            vision_ball_reaction.enable(!mode.enabled);
            cooperative_ball_reaction.enable(!mode.enabled);
        });

        // Every reaction that touches the filter is synced, so the cooperative fallback can't race a vision update
        using BallSync = Sync<BallLocalisation>;

        /* To run whenever a ball has been detected */
        vision_ball_reaction = on<Trigger<VisionBalls>, With<FieldDescription>, With<Field>, Single, BallSync>().then(
            [this](const VisionBalls& balls, const FieldDescription& fd, const Field& field) {
                // If there are no balls or there are no valid balls, return
                if (std::ranges::none_of(balls.balls, [](const auto& ball) { return !ball.is_invalid; })) {
                    return;
                }

                Eigen::Isometry3d Hwc = Eigen::Isometry3d(balls.Hcw.cast<double>()).inverse();
                last_Hcw              = balls.Hcw;
                auto state            = BallModel<double>::StateVec(ukf.get_state());

                // Data association: find the ball closest to our current estimate
                Eigen::Vector3d rBWw   = Eigen::Vector3d::Zero();
                double lowest_distance = std::numeric_limits<double>::max();
                for (const auto& ball : balls.balls) {
                    if (ball.is_invalid) {
                        continue;  // Skip ball if it is marked invalid
                    }

                    Eigen::Vector3d current_rBWw = Hwc * ball.measurements[0].rBCc.cast<double>();
                    Eigen::Vector3d rBFf         = field.Hfw * Eigen::Vector3d(current_rBWw.x(), current_rBWw.y(), 0);

                    if (rBFf.x() < (-fd.dimensions.field_length / 2) - cfg.max_distance_from_field
                        || rBFf.x() > (fd.dimensions.field_length / 2) + cfg.max_distance_from_field
                        || rBFf.y() < (-fd.dimensions.field_width / 2) - cfg.max_distance_from_field
                        || rBFf.y() > (fd.dimensions.field_width / 2) + cfg.max_distance_from_field) {
                        continue;
                    }
                    double current_distance = (current_rBWw.head<2>() - state.rBWw).squaredNorm();
                    if (current_distance < lowest_distance) {
                        lowest_distance = current_distance;
                        rBWw            = current_rBWw;
                    }
                }

                // Data association: ensure the ball is within the acceptance radius
                bool accept_ball = true;
                if (lowest_distance > cfg.acceptance_radius && !first_ball_seen) {
                    rejection_count++;
                }
                else {
                    first_ball_seen = true;
                    rejection_count = 0;
                }
                log<DEBUG>("Rejection count: ", rejection_count);
                log<DEBUG>("Accept ball: ", accept_ball);

                // Data association: if we have rejected too many balls, accept the closest one
                if (rejection_count > cfg.max_rejections) {
                    accept_ball     = true;
                    rejection_count = 0;
                }

                // Don't continue if we don't accept the ball
                if (!accept_ball) {
                    return;
                }

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
                last_own_ball_time = last_time_update;

                // Generate and emit message. Get the new state, here we are assuming ball is on the ground
                auto ball                 = std::make_unique<Ball>();
                state                     = BallModel<double>::StateVec(ukf.get_state());
                ball->rBWw                = Eigen::Vector3d(state.rBWw.x(), state.rBWw.y(), fd.ball_radius);
                ball->vBw                 = Eigen::Vector3d(state.vBw.x(), state.vBw.y(), 0);
                ball->confidence          = 1.0;  // Full confidence in our own measurements
                ball->time_of_measurement = last_time_update;
                ball->Hcw                 = balls.Hcw;
                if (log_level <= DEBUG) {
                    log<DEBUG>("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z());
                    log<DEBUG>("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z());
                    emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                    emit(graph("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z()));
                }

                emit(ball);
            });

        // Backup for when we have lost the ball: the cooperative ball, fused from every teammate's detections
        cooperative_ball_reaction =
            on<Trigger<CooperativeBall>, Single, BallSync>().then([this](const CooperativeBall& cooperative) {
                if (cfg.use_cooperative_ball) {
                    cooperative_fallback(cooperative);
                }
            });
    }

    void BallLocalisation::cooperative_fallback(const CooperativeBall& cooperative) {
        const auto now = NUClear::clock::now();
        // Only a backup: while we are seeing the ball ourselves, our own estimate is used
        if (std::chrono::duration<double>(now - last_own_ball_time).count() < cfg.cooperative_ball_timeout) {
            return;
        }
        // The cooperative ball is only worth using while someone has seen the ball recently
        if (std::chrono::duration<double>(now - cooperative.ball.time_of_measurement).count()
            > cfg.cooperative_ball_max_age) {
            return;
        }

        // Move our own filter onto the cooperative estimate, so our next detection of the ball is gated around where
        // the team thinks it is, rather than around where we lost it
        BallModel<double>::StateVec state;
        state.rBWw = cooperative.ball.rBWw.head<2>();
        state.vBw  = cooperative.ball.vBw.head<2>();
        ukf.set_state(state.getStateVec(), cooperative.ball.covariance);
        last_time_update      = now;
        last_cooperative_time = now;
        rejection_count       = 0;

        auto ball                 = std::make_unique<Ball>(cooperative.ball);
        ball->confidence          = 0.0;  // Not our own measurement
        ball->Hcw                 = last_Hcw;
        ball->average_rBWw        = cooperative.ball.rBWw;
        ball->time_of_measurement = cooperative.ball.time_of_measurement;
        log<DEBUG>("Using the cooperative ball, seen by", cooperative.observers.size(), "robots");
        emit(ball);
    }

}  // namespace module::localisation
