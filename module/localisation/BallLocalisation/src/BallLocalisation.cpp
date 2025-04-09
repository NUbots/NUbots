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

#include "message/eye/DataPoint.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/vision/Ball.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using Ball        = message::localisation::Ball;
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

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

            // Set configuration for robot to robot communication balls
            cfg.max_robots               = config["max_robots"].as<int>();
            cfg.team_ball_recency        = config["team_ball_recency"].as<double>();
            cfg.team_guess_error         = config["team_guess_error"].as<double>();
            cfg.team_guess_default_timer = config["team_guess_default_timer"].as<double>();

            team_guesses.resize(cfg.max_robots);

            last_time_update = NUClear::clock::now();
        });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<FieldDescription>>().then(
            [this](const VisionBalls& balls, const FieldDescription& fd) {
                if (!balls.balls.empty()) {
                    Eigen::Isometry3d Hwc = Eigen::Isometry3d(balls.Hcw.cast<double>()).inverse();
                    auto state            = BallModel<double>::StateVec(ukf.get_state());

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
                    log<DEBUG>("Rejection count: ", rejection_count);
                    log<DEBUG>("Accept ball: ", accept_ball);

                    // Data association: if we have rejected too many balls, accept the closest one
                    bool low_confidence = false;
                    if (rejection_count > cfg.max_rejections) {
                        accept_ball     = true;
                        rejection_count = 0;
                        low_confidence  = true;
                    }

                    Eigen::Vector3d team_guess_average = Eigen::Vector3d::Zero();

                    const bool accept_team_guess = get_team_guess(team_guess_average);

                    if (accept_ball || accept_team_guess) {
                        // Compute the time since the last update (in seconds)
                        const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                                  - last_time_update)
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
                        auto ball = std::make_unique<Ball>();

                        if (low_confidence && accept_team_guess) {
                            ball->rBWw = balls.Hcw * team_guess_average;
                            ball->vBw  = Eigen::Vector3d::Zero();
                        }
                        else {
                            ball->rBWw = Eigen::Vector3d(state.rBWw.x(), state.rBWw.y(), fd.ball_radius);
                            ball->vBw  = Eigen::Vector3d(state.vBw.x(), state.vBw.y(), 0);
                        }

                        ball->time_of_measurement = last_time_update;
                        ball->Hcw                 = balls.Hcw;
                        if (log_level <= DEBUG) {
                            log<DEBUG>("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z());
                            log<DEBUG>("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z());
                            emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                            emit(graph("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z()));
                        }

                        emit(ball);
                    }
                }
            });

        // Stores ball positions received from teammates
        on<Trigger<RoboCup>>().then([this](const RoboCup& robocup) {
            Eigen::Vector3d rBFf = robocup.ball.position.cast<double>();

            team_guesses[robocup.current_pose.player_id - 1].last_heard = NUClear::clock::now();
            team_guesses[robocup.current_pose.player_id - 1].rBFf       = rBFf;
        });

        // Called once a second to default to teammates balls if we haven't seen one recently
        on<Every<1, Per<std::chrono::seconds>>, Optional<With<VisionBalls>>>().then(
            [this](const std::shared_ptr<const VisionBalls>& balls) {
                if (!balls->balls.empty()) {
                    last_Hcw = Eigen::Isometry3d(balls->Hcw.cast<double>());
                }

                const auto dt =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_time_update)
                        .count();

                if (dt > cfg.team_guess_default_timer) {

                    Eigen::Vector3d average = Eigen::Vector3d::Zero();

                    get_team_guess(average);

                    last_time_update = NUClear::clock::now();

                    auto ball = std::make_unique<Ball>();

                    ball->rBWw                = last_Hcw * average;
                    ball->vBw                 = Eigen::Vector3d::Zero();
                    ball->time_of_measurement = last_time_update;
                    ball->Hcw                 = last_Hcw;
                    emit(ball);
                }
            });
    }

    // Run to calculate balls using robot to robot communication
    // Returns whether we have a valid guess from teammates balls
    // Error calculation is optional
    bool BallLocalisation::get_team_guess(Eigen::Vector3d& average) {

        std::vector<Eigen::Vector3d> to_check;

        for (auto guess : team_guesses) {

            // Check if our teammates ball message meets our recency threshold
            if (std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - guess.last_heard)
                    .count()
                < cfg.team_ball_recency) {
                to_check.emplace_back(guess.rBFf);
            }
        }

        // Check if we have enough guesses from teammates
        if (to_check.size() > 0) {

            Eigen::Vector3d error = Eigen::Vector3d::Zero();

            if (to_check.size() > 1) {

                for (auto i = to_check.begin() + 1; i != to_check.end(); ++i) {
                    error -= (*to_check.begin() - *i).cwiseAbs();
                }
            }

            for (auto guess : to_check) {
                average += guess;
            }
            average /= to_check.size();

            if (error.norm() < 1.0) {

                return true;
            }
        }
        return false;
    }

}  // namespace module::localisation
