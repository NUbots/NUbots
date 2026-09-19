/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
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
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::localisation::GroundTruthMode;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::nusight::graph;
    using utility::support::Expression;

    namespace {
        double seconds(const NUClear::clock::duration& d) {
            return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
        }
    }  // namespace

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("BallLocalisation.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Measurement noise grows with range: vision error is roughly proportional to distance
            cfg.ukf.measurement_base      = config["ukf"]["noise"]["measurement"]["base"].as<double>();
            cfg.ukf.measurement_per_metre = config["ukf"]["noise"]["measurement"]["per_metre"].as<double>();

            // Motion model
            cfg.ukf.acceleration_noise      = config["ukf"]["noise"]["process"]["acceleration"].as<double>();
            cfg.ukf.rolling_deceleration    = config["motion"]["rolling_deceleration"].as<double>();
            ukf.model.acceleration_noise    = cfg.ukf.acceleration_noise;
            ukf.model.rolling_deceleration  = cfg.ukf.rolling_deceleration;
            cfg.ukf.initial_covariance.rBWw = config["ukf"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.vBw  = config["ukf"]["initial"]["covariance"]["velocity"].as<Expression>();

            // Association
            cfg.association.gate              = config["association"]["gate"].as<double>();
            cfg.association.max_ball_speed    = config["association"]["max_ball_speed"].as<double>();
            cfg.association.kick_velocity_std = config["association"]["kick_velocity_std"].as<double>();
            cfg.association.confirm_radius    = config["association"]["confirm_radius"].as<double>();
            cfg.association.reacquire_after   = config["association"]["reacquire_after"].as<double>();

            // Set configuration for robot to robot communication balls
            cfg.use_r2r_balls            = config["use_r2r_balls"].as<bool>();
            cfg.team_ball_recency        = config["team_ball_recency"].as<double>();
            cfg.team_guess_error         = config["team_guess_error"].as<double>();
            cfg.team_guess_default_timer = config["team_guess_default_timer"].as<double>();

            cfg.max_distance_from_field = config["max_distance_from_field"].as<double>();

            // Start acquiring from scratch with the new settings
            tracking         = false;
            last_time_update = NUClear::clock::now();
        });

        // While ground truth perception is on, Ball comes from the simulator instead, so stop our own reactions
        on<Trigger<GroundTruthMode>>().then([this](const GroundTruthMode& mode) {
            vision_ball_reaction.enable(!mode.enabled);
            team_ball_reaction.enable(!mode.enabled);
        });

        /* To run whenever a ball has been detected */
        vision_ball_reaction = on<Trigger<VisionBalls>, With<FieldDescription>, With<Field>, Single>().then(
            [this](const VisionBalls& balls, const FieldDescription& fd, const Field& field) {
                // The filter runs in image time: the detections describe the ball when the image was taken,
                // however long vision took to deliver them
                const NUClear::clock::time_point image_time = balls.timestamp;
                last_Hcw                                    = balls.Hcw;
                const Eigen::Isometry3d Hwc                 = Eigen::Isometry3d(balls.Hcw).inverse();

                // Candidate detections on or near the field, in world space, each with its own noise
                std::vector<Candidate> candidates{};
                for (const auto& ball : balls.balls) {
                    if (ball.is_invalid || ball.measurements.empty()) {
                        continue;
                    }
                    const Eigen::Vector3d rBCc = ball.measurements[0].rBCc.cast<double>();
                    const Eigen::Vector3d rBWw = Hwc * rBCc;
                    const Eigen::Vector3d rBFf = field.Hfw * Eigen::Vector3d(rBWw.x(), rBWw.y(), 0);
                    if (std::abs(rBFf.x()) > fd.dimensions.field_length / 2 + cfg.max_distance_from_field
                        || std::abs(rBFf.y()) > fd.dimensions.field_width / 2 + cfg.max_distance_from_field) {
                        continue;
                    }
                    candidates.push_back(
                        {rBWw.head<2>(), cfg.ukf.measurement_base + cfg.ukf.measurement_per_metre * rBCc.norm()});
                }

                std::optional<Candidate> accepted{};
                bool restarted = false;

                if (tracking) {
                    const double dt = seconds(image_time - filter_time);
                    if (dt > 0.0) {
                        ukf.time(dt);
                        filter_time = image_time;
                    }
                    const BallModel<double>::StateVec state(ukf.get_state());
                    const Eigen::Matrix2d P   = ukf.get_covariance().topLeftCorner<2, 2>();
                    const double since_accept = seconds(image_time - last_accept_time);
                    // A kick is only believable if the ball could have got there since we last saw it; cap the
                    // window so a long gap cannot make any far-off false positive "reachable"
                    const double kick_window = std::min(since_accept, cfg.association.reacquire_after);

                    const Candidate* inside = nullptr;
                    const Candidate* kick   = nullptr;
                    double inside_d2        = std::numeric_limits<double>::infinity();
                    double kick_d2          = std::numeric_limits<double>::infinity();
                    for (const auto& c : candidates) {
                        const Eigen::Vector2d innovation = c.rBWw - state.rBWw;
                        const Eigen::Matrix2d S          = P + Eigen::Matrix2d::Identity() * c.sigma * c.sigma;
                        const double d2                  = innovation.dot(S.ldlt().solve(innovation));
                        if (d2 <= cfg.association.gate) {
                            if (d2 < inside_d2) {
                                inside_d2 = d2;
                                inside    = &c;
                            }
                        }
                        else if (d2 < kick_d2
                                 && innovation.norm() <= cfg.association.max_ball_speed * kick_window + 3.0 * c.sigma
                                 && confirmed(c, image_time)) {
                            kick_d2 = d2;
                            kick    = &c;
                        }
                    }

                    if (inside != nullptr) {
                        accepted = *inside;
                    }
                    else if (kick != nullptr) {
                        // Kicked: the constant-velocity prediction no longer holds. Open up the velocity (and the
                        // position, so the jump is taken) and let the detection pull the track across.
                        BallModel<double>::StateMat covariance = ukf.get_covariance();
                        covariance.block<2, 2>(BallModel<double>::StateVec::VX, BallModel<double>::StateVec::VX) +=
                            Eigen::Matrix2d::Identity() * std::pow(cfg.association.kick_velocity_std, 2);
                        covariance.topLeftCorner<2, 2>() +=
                            Eigen::Matrix2d::Identity() * (kick->rBWw - state.rBWw).squaredNorm();
                        ukf.set_state(state.getStateVec(), covariance);
                        accepted = *kick;
                        log<DEBUG>("Ball kick detected, innovation", (kick->rBWw - state.rBWw).norm(), "m");
                    }
                    else if (since_accept > cfg.association.reacquire_after) {
                        // Lost the ball: jump to the confirmed detection nearest where it should be
                        const Candidate* best = nullptr;
                        double best_distance  = std::numeric_limits<double>::infinity();
                        for (const auto& c : candidates) {
                            const double distance = (c.rBWw - state.rBWw).norm();
                            if (distance < best_distance && confirmed(c, image_time)) {
                                best_distance = distance;
                                best          = &c;
                            }
                        }
                        if (best != nullptr) {
                            reset_track(*best, image_time);
                            restarted = true;
                            log<DEBUG>("Ball reacquired", best_distance, "m from the lost track");
                        }
                    }
                }
                else {
                    // Not tracking yet: start on the nearest detection seen in two consecutive images
                    const Candidate* best = nullptr;
                    for (const auto& c : candidates) {
                        if (confirmed(c, image_time) && (best == nullptr || c.sigma < best->sigma)) {
                            best = &c;
                        }
                    }
                    if (best != nullptr) {
                        reset_track(*best, image_time);
                        restarted = true;
                    }
                }

                if (accepted) {
                    ukf.measure(accepted->rBWw,
                                Eigen::Matrix2d(Eigen::Matrix2d::Identity() * accepted->sigma * accepted->sigma),
                                MeasurementType::BALL_POSITION());
                    last_accept_time = image_time;
                }

                previous_candidates = std::move(candidates);
                previous_time       = image_time;
                if (!accepted && !restarted) {
                    return;
                }

                // Publish the ball where it is now: predict the image-time estimate over vision's latency
                const BallModel<double>::StateVec state(ukf.get_state());
                const double latency = std::clamp(seconds(NUClear::clock::now() - image_time), 0.0, 0.5);
                const BallModel<double>::StateVec now_state(ukf.model.time(state, latency));

                auto ball                 = std::make_unique<Ball>();
                ball->rBWw                = Eigen::Vector3d(now_state.rBWw.x(), now_state.rBWw.y(), fd.ball_radius);
                ball->vBw                 = Eigen::Vector3d(now_state.vBw.x(), now_state.vBw.y(), 0);
                ball->covariance          = ukf.get_covariance();
                ball->confidence          = 1.0;  // Full confidence in our own measurements
                ball->time_of_measurement = image_time;
                ball->Hcw                 = balls.Hcw;
                if (cfg.use_r2r_balls) {
                    ball->average_rBWw = field.Hfw.inverse() * get_average_team_rBFf().second;
                }
                last_time_update = NUClear::clock::now();

                if (log_level <= DEBUG) {
                    emit(graph("rBWw: ", ball->rBWw.x(), ball->rBWw.y(), ball->rBWw.z()));
                    emit(graph("vBw: ", ball->vBw.x(), ball->vBw.y(), ball->vBw.z()));
                }
                emit(ball);
            });

        // Stores ball positions received from teammates
        team_ball_reaction = on<Trigger<RoboCup>, With<Field>>().then([this](const RoboCup& robocup, const Field& field) {
            if (!cfg.use_r2r_balls) {
                return;
            }

            // This occurs when the ball has not been seen
            if (robocup.ball.age < 0.0f) {
                // If the ball has not been seen, then we don't care about it
                return;
            }

            Eigen::Vector3d rBFf = robocup.ball.position.cast<double>();

            // Resize the vector of guesses if it is not large enough
            if (team_guesses.capacity() < robocup.current_pose.player_id) {
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

    void BallLocalisation::reset_track(const Candidate& candidate, const NUClear::clock::time_point& time) {
        BallModel<double>::StateVec mean{};
        mean.rBWw = candidate.rBWw;
        ukf.set_state(mean.getStateVec(), cfg.ukf.initial_covariance.asDiagonal());
        tracking         = true;
        filter_time      = time;
        last_accept_time = time;
    }

    bool BallLocalisation::confirmed(const Candidate& candidate, const NUClear::clock::time_point& time) const {
        const double dt = seconds(time - previous_time);
        if (dt <= 0.0 || dt > cfg.association.reacquire_after) {
            return false;
        }
        const double reach = cfg.association.confirm_radius + cfg.association.max_ball_speed * dt;
        return std::any_of(previous_candidates.begin(), previous_candidates.end(), [&](const Candidate& previous) {
            return (previous.rBWw - candidate.rBWw).norm() <= reach;
        });
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
