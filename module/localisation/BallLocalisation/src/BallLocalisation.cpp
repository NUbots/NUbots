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
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
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
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

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
            cfg.use_r2r_balls            = config["use_r2r_balls"].as<bool>();
            cfg.team_ball_recency        = config["team_ball_recency"].as<double>();
            cfg.team_guess_error         = config["team_guess_error"].as<double>();
            cfg.team_guess_default_timer = config["team_guess_default_timer"].as<double>();

            last_time_update = NUClear::clock::now();
        });

        /* To run whenever a ball has been detected */
        on<Trigger<VisionBalls>, With<FieldDescription>, With<Field>, Single>().then([this](const VisionBalls& balls,
                                                                                            const FieldDescription& fd,
                                                                                            const Field& field) {
            // If no balls, return
            if (balls.balls.empty()) {
                return;
            }

            Eigen::Isometry3d Hwc = Eigen::Isometry3d(balls.Hcw.cast<double>()).inverse();
            last_Hcw              = balls.Hcw;
            auto state            = BallModel<double>::StateVec(ukf.get_state());

            // The closest ball in the vision detections
            Eigen::Vector3d rBWw = closest_ball(balls, Hwc, state.rBWw);

            bool low_confidence = false;
            bool accept         = accept_ball((rBWw.head<2>() - state.rBWw).squaredNorm(), low_confidence);
            first_ball_seen     = accept ? true : first_ball_seen;

            bool accept_team_guess       = false;
            Eigen::Vector3d average_rBFf = Eigen::Vector3d::Zero();
            if (cfg.use_r2r_balls) {
                auto [valid, avg] = get_average_team_rBFf();
                accept_team_guess = valid;
                average_rBFf      = avg;
            }

            // Don't continue if we don't accept the ball or team guess
            if (!(accept || accept_team_guess)) {
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

            // Get the new state, here we are assuming ball is on the ground
            state = BallModel<double>::StateVec(ukf.get_state());

            // Generate and emit message
            auto ball = std::make_unique<Ball>();

            if (low_confidence && accept_team_guess) {
                ball->rBWw       = field.Hfw.inverse() * average_rBFf;
                ball->vBw        = Eigen::Vector3d::Zero();
                ball->confidence = 0.0;  // No confidence in other teammates' guesses
            }
            else {
                ball->rBWw       = Eigen::Vector3d(state.rBWw.x(), state.rBWw.y(), fd.ball_radius);
                ball->vBw        = Eigen::Vector3d(state.vBw.x(), state.vBw.y(), 0);
                ball->confidence = 1.0;  // Full confidence in our own measurements
            }

            ball->time_of_measurement = last_time_update;
            ball->Hcw                 = balls.Hcw;
            ball->average_rBWw        = field.Hfw.inverse() * average_rBFf;
            if (log_level <= DEBUG) {
                log<DEBUG>("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z());
                log<DEBUG>("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z());
                log<DEBUG>("average rBWw: ", ball->average_rBWw.x(), ball->average_rBWw.y(), ball->average_rBWw.z());
                emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                emit(graph("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z()));
                log<DEBUG>("average_rBWw: ", ball->average_rBWw.x(), ball->average_rBWw.y(), ball->average_rBWw.z());
            }

            emit(ball);
        });

        // Stores ball positions received from teammates
        on<Trigger<RoboCup>, With<Field>>().then([this](const RoboCup& robocup, const Field& field) {
            if (!cfg.use_r2r_balls) {
                return;
            }

            // This occurs when the ball position is an echo - ignore echos
            if (robocup.ball.confidence == 0.0) {
                // If the ball has no confidence, then we don't care about it
                return;
            }

            Eigen::Vector3d rBFf = robocup.ball.position.cast<double>();

            // Resize the vector of guesses if it is not large enough
            if (team_guesses.size() < robocup.current_pose.player_id) {
                team_guesses.resize(robocup.current_pose.player_id);
            }

            // Update this teammates information
            team_guesses[robocup.current_pose.player_id - 1].last_heard = NUClear::clock::now();
            team_guesses[robocup.current_pose.player_id - 1].rBFf       = rBFf;

            // Don't use teammates ball info if we have a recent ball measurement
            const auto dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_time_update)
                    .count();
            if (dt < cfg.team_guess_default_timer) {
                return;
            }

            // If we have a valid guess, emit a new ball message
            last_time_update          = NUClear::clock::now();
            auto ball                 = std::make_unique<Ball>();
            ball->rBWw                = field.Hfw.inverse() * get_average_team_rBFf().second;
            ball->vBw                 = Eigen::Vector3d::Zero();
            ball->confidence          = 0.0;  // No confidence in other teammates' guesses
            ball->time_of_measurement = last_time_update;
            ball->Hcw                 = last_Hcw;
            emit(ball);
        });
    }

    Eigen::Vector3d BallLocalisation::closest_ball(const VisionBalls& balls,
                                                   const Eigen::Isometry3d& Hwc,
                                                   const Eigen::Vector2d& state_rBWw) {
        Eigen::Vector3d closest = Eigen::Vector3d::Zero();
        double min_dist         = std::numeric_limits<double>::max();
        for (const auto& ball : balls.balls) {
            Eigen::Vector3d rBWw = Hwc * ball.measurements[0].rBCc.cast<double>();
            double dist          = (rBWw.head<2>() - state_rBWw).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                closest  = rBWw;
            }
        }
        return closest;
    }

    bool BallLocalisation::accept_ball(double lowest_distance, bool& low_confidence) {
        if (lowest_distance > cfg.acceptance_radius && !first_ball_seen) {
            rejection_count++;
            if (rejection_count > cfg.max_rejections) {
                rejection_count = 0;
                low_confidence  = true;
                return true;
            }
            return false;
        }
        rejection_count = 0;
        return true;
    }

    std::pair<bool, Eigen::Vector3d> BallLocalisation::get_average_team_rBFf() {
        // Determine which balls to consider
        std::vector<Eigen::Vector3d> to_check{};
        for (auto guess : team_guesses) {
            // Check if the teammates ball message meets the recency threshold
            if (std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - guess.last_heard)
                    .count()
                < cfg.team_ball_recency) {
                to_check.emplace_back(guess.rBFf);
            }
        }

        // Require at least one guess
        if (to_check.empty()) {
            return {false, Eigen::Vector3d::Zero()};
        }

        // Compute average
        Eigen::Vector3d average = std::accumulate(to_check.begin(), to_check.end(), Eigen::Vector3d::Zero().eval());
        average /= static_cast<double>(to_check.size());

        // Compute mean absolute error from average
        double error = 0.0;
        for (const auto& guess : to_check) {
            error += (guess - average).norm();
        }
        error /= static_cast<double>(to_check.size());

        return {error < cfg.team_guess_error, average};
    }

}  // namespace module::localisation
