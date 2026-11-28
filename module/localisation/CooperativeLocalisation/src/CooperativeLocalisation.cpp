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
#include "CooperativeLocalisation.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

#include "extension/Configuration.hpp"

#include "message/input/RoboCup.hpp"
#include "message/localisation/Cooperative.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Robot.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::input::RoboCup;
    using message::localisation::CooperativeBall;
    using message::localisation::CooperativeDetections;
    using message::localisation::CooperativeRobots;
    using message::localisation::Field;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;
    using VisionBalls  = message::vision::Balls;
    using VisionRobots = message::vision::Robots;

    namespace {
        /// @brief A duration in seconds
        double seconds(const NUClear::clock::duration& d) {
            return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
        }

        /// @brief Undo the mixed-team protocol's field rotation (our field frame rotated 180 degrees about the centre,
        /// see network::RobotCommunication), for a position sent in centimetres
        Eigen::Vector2d from_team_cm(const int x, const int y) {
            return Eigen::Vector2d(-x, -y) / 100.0;
        }
    }  // namespace

    double CooperativeLocalisation::Track::max_std() const {
        return std::sqrt(
            std::max(0.0,
                     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(position_covariance()).eigenvalues().maxCoeff()));
    }

    CooperativeLocalisation::CooperativeLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration, Sync<CooperativeLocalisation>>("CooperativeLocalisation.yaml")
            .then([this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();

                auto read_motion = [](const YAML::Node& node) {
                    Config::Motion m;
                    m.friction             = node["friction"].as<double>();
                    m.acceleration_std     = node["acceleration_std"].as<double>();
                    m.initial_velocity_std = node["initial_velocity_std"].as<double>();
                    return m;
                };
                cfg.ball  = read_motion(config["motion"]["ball"]);
                cfg.robot = read_motion(config["motion"]["robot"]);

                auto read_measurement = [](const YAML::Node& node) {
                    Config::Measurement m;
                    m.range_base         = node["range_base"].as<double>();
                    m.range_quad         = node["range_quad"].as<double>();
                    m.bearing_std        = node["bearing_std"].as<double>();
                    m.observer_inflation = node["observer_inflation"].as<double>();
                    return m;
                };
                cfg.ball_measurement      = read_measurement(config["measurement"]["ball"]);
                cfg.robot_measurement     = read_measurement(config["measurement"]["robot"]);
                cfg.observer_position_std = config["measurement"]["observer_position_std"].as<double>();
                cfg.observer_heading_std  = config["measurement"]["observer_heading_std"].as<double>();

                cfg.gate_chi2               = config["association"]["gate_chi2"].as<double>();
                cfg.association_log_det     = config["association"]["log_det"].as<bool>();
                cfg.self_exclusion          = config["association"]["self_exclusion"].as<double>();
                cfg.min_spawn_range         = config["association"]["min_spawn_range"].as<double>();
                cfg.max_distance_from_field = config["association"]["max_distance_from_field"].as<double>();
                cfg.max_detection_age       = config["association"]["max_detection_age"].as<double>();

                cfg.teammate_gate_chi2 = config["teammates"]["gate_chi2"].as<double>();
                cfg.teammate_max_age   = config["teammates"]["max_age"].as<double>();
                cfg.teammate_max_speed = config["teammates"]["max_speed"].as<double>();

                cfg.robot_lost_std      = config["tracks"]["robot_lost_std"].as<double>();
                cfg.ball_lost_std       = config["tracks"]["ball_lost_std"].as<double>();
                cfg.merge_chi2          = config["tracks"]["merge_chi2"].as<double>();
                cfg.merge_max_std       = config["tracks"]["merge_max_std"].as<double>();
                cfg.ball_reset_count    = config["tracks"]["ball_reset_count"].as<int>();
                cfg.spawn_max_range_std = config["tracks"]["spawn_max_range_std"].as<double>();
                cfg.pending_window      = config["tracks"]["pending_window"].as<double>();
                cfg.observer_window     = config["tracks"]["observer_window"].as<double>();
            });

        on<Trigger<GlobalConfig>, Sync<CooperativeLocalisation>>().then(
            [this](const GlobalConfig& global_config) { player_id = global_config.player_id; });

        on<Trigger<FieldDescription>, Sync<CooperativeLocalisation>>().then([this](const FieldDescription& fd) {
            field_description      = fd;
            have_field_description = true;
        });

        // Our own ball detections
        on<Trigger<VisionBalls>, With<Field>, Sync<CooperativeLocalisation>>().then(
            [this](const VisionBalls& vision_balls, const Field& field) {
                // Without a trusted pose our detections cannot be put on the field
                if (!field.localised) {
                    return;
                }
                const Eigen::Isometry3d Hwc(Eigen::Isometry3d(vision_balls.Hcw).inverse());
                const Eigen::Isometry3d Hfw(field.Hfw);
                const Eigen::Vector2d rCFf = (Hfw * Hwc.translation()).head<2>();

                teammates[player_id] = TeammatePosition{vision_balls.timestamp, rCFf};

                std::vector<Observation> observations{};
                for (const auto& ball : vision_balls.balls) {
                    if (ball.is_invalid || ball.measurements.empty()) {
                        continue;
                    }
                    const Eigen::Vector3d rBWw = Hwc * Eigen::Vector3d(ball.measurements[0].rBCc);
                    observations.push_back(
                        Observation{Target::BALL, rCFf, (Hfw * rBWw).head<2>(), vision_balls.timestamp, player_id});
                }

                if (!observations.empty()) {
                    CooperativeDetections::Detections shared;
                    shared.timestamp = vision_balls.timestamp;
                    shared.rCFf      = rCFf;
                    for (const auto& o : observations) {
                        shared.positions.emplace_back(o.rXFf);
                    }
                    detections.balls = shared;
                    emit(std::make_unique<CooperativeDetections>(detections));
                }

                process_scan(observations);
            });

        // Our own robot detections
        on<Trigger<VisionRobots>, With<Field>, Sync<CooperativeLocalisation>>().then(
            [this](const VisionRobots& vision_robots, const Field& field) {
                if (!field.localised) {
                    return;
                }
                const Eigen::Isometry3d Hwc(Eigen::Isometry3d(vision_robots.Hcw).inverse());
                const Eigen::Isometry3d Hfw(field.Hfw);
                const Eigen::Vector2d rCFf = (Hfw * Hwc.translation()).head<2>();

                teammates[player_id] = TeammatePosition{vision_robots.timestamp, rCFf};

                std::vector<Observation> observations{};
                for (const auto& robot : vision_robots.robots) {
                    const Eigen::Vector3d rRWw = Hwc * Eigen::Vector3d(robot.rRCc);
                    observations.push_back(
                        Observation{Target::ROBOT, rCFf, (Hfw * rRWw).head<2>(), vision_robots.timestamp, player_id});
                }

                if (!observations.empty()) {
                    CooperativeDetections::Detections shared;
                    shared.timestamp = vision_robots.timestamp;
                    shared.rCFf      = rCFf;
                    for (const auto& o : observations) {
                        shared.positions.emplace_back(o.rXFf);
                    }
                    detections.robots = shared;
                    emit(std::make_unique<CooperativeDetections>(detections));
                }

                process_scan(observations);
            });

        // Teammates' positions and the detections they share
        on<Trigger<RoboCup>, Sync<CooperativeLocalisation>>().then([this](const RoboCup& robocup) {
            const uint32_t sender = robocup.current_pose.player_id;
            if (sender == 0 || sender == player_id) {
                return;
            }
            const auto now = NUClear::clock::now();

            // Positions are in the mixed-team protocol's field frame, see network::RobotCommunication
            teammates[sender] = TeammatePosition{
                now,
                Eigen::Vector2d(-robocup.current_pose.position.x(), -robocup.current_pose.position.y())};

            auto read = [&](const message::input::NUbotsTeamData::Detections& shared, const Target target) {
                std::vector<Observation> observations{};
                if (shared.observer.size() != 2 || shared.positions.size() % 2 != 0) {
                    return observations;
                }
                // Clocks are not synchronised between robots, so date the frame from its age on arrival
                const auto time =
                    now
                    - std::chrono::duration_cast<NUClear::clock::duration>(std::chrono::milliseconds(shared.age_ms));
                const Eigen::Vector2d rCFf = from_team_cm(shared.observer[0], shared.observer[1]);
                for (size_t i = 0; i + 1 < size_t(shared.positions.size()); i += 2) {
                    observations.push_back(Observation{target,
                                                       rCFf,
                                                       from_team_cm(shared.positions[i], shared.positions[i + 1]),
                                                       time,
                                                       sender});
                }
                return observations;
            };
            process_scan(read(robocup.nubots.balls, Target::BALL));
            process_scan(read(robocup.nubots.robots, Target::ROBOT));
        });

        on<Every<UPDATE_RATE, Per<std::chrono::seconds>>, With<Field>, Sync<CooperativeLocalisation>>().then(
            [this](const Field& field) {
                const auto now = NUClear::clock::now();
                predict(now);
                maintenance();

                // Field {f} to world {w}. The two share the ground plane, so only x, y and yaw differ
                const Eigen::Isometry3d Hwf(Eigen::Isometry3d(field.Hfw).inverse());
                const Eigen::Matrix2d Rwf   = Hwf.linear().topLeftCorner<2, 2>();
                Eigen::Matrix4d T           = Eigen::Matrix4d::Zero();
                T.topLeftCorner<2, 2>()     = Rwf;
                T.bottomRightCorner<2, 2>() = Rwf;

                auto robots = std::make_unique<CooperativeRobots>();
                for (const auto& track : tracks) {
                    const Eigen::Vector4d x = track.filter.mean();
                    const Eigen::Matrix4d P = T * track.filter.covariance() * T.transpose();
                    if (track.target == Target::BALL) {
                        const double z                 = have_field_description ? field_description.ball_radius : 0.0;
                        auto ball                      = std::make_unique<CooperativeBall>();
                        ball->ball.rBWw                = Hwf * Eigen::Vector3d(x.x(), x.y(), z);
                        ball->ball.vBw                 = Hwf.linear() * Eigen::Vector3d(x.z(), x.w(), 0.0);
                        ball->ball.covariance          = P;
                        ball->ball.time_of_measurement = track.last_measurement;
                        ball->ball.Hcw                 = Eigen::Isometry3d::Identity();
                        for (const auto& [observer, time] : track.observers) {
                            if (seconds(now - time) <= cfg.observer_window) {
                                ball->observers.push_back(observer);
                            }
                        }
                        emit(ball);
                    }
                    else {
                        message::localisation::Robot robot;
                        robot.id                  = track.id;
                        robot.rRWw                = Hwf * Eigen::Vector3d(x.x(), x.y(), 0.0);
                        robot.vRw                 = Hwf.linear() * Eigen::Vector3d(x.z(), x.w(), 0.0);
                        robot.covariance          = P;
                        robot.time_of_measurement = track.last_measurement;
                        robot.teammate            = false;
                        robots->robots.push_back(robot);
                    }
                }
                emit(robots);

                if (log_level <= DEBUG) {
                    for (const auto& track : tracks) {
                        log<DEBUG>(track.target == Target::BALL ? "Ball" : "Robot",
                                   track.id,
                                   "at",
                                   track.position().transpose(),
                                   "std",
                                   track.max_std(),
                                   "fusions",
                                   track.fusions,
                                   "observers",
                                   track.observers.size());
                    }
                }
            });
    }

    void CooperativeLocalisation::predict(const NUClear::clock::time_point& time) {
        for (auto& track : tracks) {
            const double dt = seconds(time - track.time);
            if (dt <= 0.0) {
                continue;
            }
            const auto& m  = motion(track.target);
            const double b = std::exp(-m.friction * dt);
            const double a = m.friction > 0.0 ? (1.0 - b) / m.friction : dt;

            // Constant velocity with velocity damping, driven by white acceleration noise on the velocity
            Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
            F(0, 2)           = a;
            F(1, 3)           = a;
            F(2, 2)           = b;
            F(3, 3)           = b;
            Eigen::Matrix<double, 4, 2> G;
            G << dt / 2.0, 0.0, 0.0, dt / 2.0, 1.0, 0.0, 0.0, 1.0;
            const Eigen::Matrix2d Q = m.acceleration_std * m.acceleration_std * dt * Eigen::Matrix2d::Identity();

            track.filter.time(F, G, Q);
            track.time = time;
        }
    }

    bool CooperativeLocalisation::on_field(const Eigen::Vector2d& rXFf) const {
        if (!have_field_description) {
            return true;
        }
        return std::abs(rXFf.x()) <= field_description.dimensions.field_length / 2.0 + cfg.max_distance_from_field
               && std::abs(rXFf.y()) <= field_description.dimensions.field_width / 2.0 + cfg.max_distance_from_field;
    }

    double CooperativeLocalisation::range_std(const Target target, const double r) const {
        const auto& m = measurement(target);
        return m.range_base + m.range_quad * r * r;
    }

    Eigen::Matrix2d CooperativeLocalisation::observation_covariance(const Observation& observation) const {
        const auto& m           = measurement(observation.target);
        const Eigen::Vector2d d = observation.rXFf - observation.rCFf;
        const double r          = std::max(d.norm(), 0.05);
        const Eigen::Vector2d u = d / r;
        const Eigen::Vector2d t(-u.y(), u.x());
        const double C = cfg.observer_position_std * cfg.observer_position_std * m.observer_inflation;

        const double sr       = range_std(observation.target, r);
        const double s2_range = sr * sr + C;
        const double s2_tangential =
            r * r * (m.bearing_std * m.bearing_std + cfg.observer_heading_std * cfg.observer_heading_std) + C;
        return s2_range * u * u.transpose() + s2_tangential * t * t.transpose();
    }

    bool CooperativeLocalisation::is_teammate(const Observation& observation) const {
        const Eigen::Matrix2d Rd = observation_covariance(observation);
        const double pose_var    = cfg.observer_position_std * cfg.observer_position_std;
        for (const auto& [id, teammate] : teammates) {
            if (id == observation.observer) {
                continue;  // A robot never detects itself
            }
            const double age = std::abs(seconds(observation.time - teammate.time));
            if (age > cfg.teammate_max_age) {
                continue;
            }
            const double growth     = cfg.teammate_max_speed * age;
            const Eigen::Matrix2d S = Rd + (pose_var + growth * growth) * Eigen::Matrix2d::Identity();
            const Eigen::Vector2d e = observation.rXFf - teammate.rRFf;
            if (e.dot(S.ldlt().solve(e)) <= cfg.teammate_gate_chi2) {
                return true;
            }
        }
        return false;
    }

    CooperativeLocalisation::Prediction CooperativeLocalisation::predict_measurement(
        const Track& track,
        const Observation& observation) const {
        const auto& m = measurement(observation.target);
        Prediction p;

        // A detection from before the track's time (a teammate's, delayed by the network) measures where the target
        // was then: p(t - lag) = p(t) - lag * v(t), plus the process noise of the lag
        const double lag              = std::max(0.0, seconds(track.time - observation.time));
        Eigen::Matrix<double, 2, 4> L = Eigen::Matrix<double, 2, 4>::Zero();
        L.leftCols<2>().setIdentity();
        L.rightCols<2>()     = -lag * Eigen::Matrix2d::Identity();
        const double q       = motion(track.target).acceleration_std;
        const double lag_var = q * q * lag * lag * lag / 3.0;

        const Eigen::Vector2d d = L * track.filter.mean() - observation.rCFf;
        const double r          = std::max(d.norm(), 0.05);
        const Eigen::Vector2d u = d / r;
        const Eigen::Vector2d t(-u.y(), u.x());
        const Eigen::Matrix2d A = (Eigen::Matrix2d::Identity() - u * u.transpose()) / r;

        p.h << u, r;
        p.H.topRows<2>() = A * L;
        p.H.row(2)       = u.transpose() * L;

        const Eigen::Vector2d dm = observation.rXFf - observation.rCFf;
        const double rm          = std::max(dm.norm(), 0.05);
        p.y << dm / rm, rm;

        // Bearing noise: detector, observer heading, and observer position across the line of sight. Range noise:
        // detector (grows with range, from ground projection) and observer position along the line of sight.
        const double C          = cfg.observer_position_std * cfg.observer_position_std * m.observer_inflation;
        const double sr         = range_std(observation.target, rm);
        const double s2_bearing = m.bearing_std * m.bearing_std + cfg.observer_heading_std * cfg.observer_heading_std
                                  + (C + lag_var) / (r * r);
        const double s2_range = sr * sr + C + lag_var;
        p.R                   = Eigen::Vector3d(s2_bearing, s2_bearing, s2_range).asDiagonal();
        return p;
    }

    std::pair<double, double> CooperativeLocalisation::gate(const Track& track, const Observation& observation) const {
        const Prediction p       = predict_measurement(track, observation);
        const Eigen::Matrix3d S  = p.H * track.filter.covariance() * p.H.transpose() + p.R;
        const Eigen::Vector3d nu = p.y - p.h;
        const double d2          = nu.dot(S.ldlt().solve(nu));
        const double cost        = cfg.association_log_det ? d2 + std::log(std::max(S.determinant(), 1e-300)) : d2;
        return {d2, cost};
    }

    void CooperativeLocalisation::fuse(Track& track, const Observation& observation) {
        const Prediction p      = predict_measurement(track, observation);
        const Eigen::Vector3d z = p.y - p.h + p.H * track.filter.mean();
        if (!track.filter.measure(z, p.H, p.R)) {
            log<WARN>("Cooperative track", track.id, "update failed, skipping the detection");
            return;
        }
        track.fusions++;
        track.last_measurement                = std::max(track.last_measurement, observation.time);
        track.observers[observation.observer] = std::max(track.observers[observation.observer], observation.time);
    }

    CooperativeLocalisation::Track& CooperativeLocalisation::spawn(const Observation& observation) {
        const auto& m    = motion(observation.target);
        const auto now   = NUClear::clock::now();
        const double lag = std::max(0.0, seconds(now - observation.time));

        // The track starts at the detection with the detection's own covariance: deep along the line of sight, narrow
        // across it. Over a delayed detection's lag the (unknown) velocity could have moved it further.
        Eigen::Matrix4d P           = Eigen::Matrix4d::Zero();
        const double drift          = m.initial_velocity_std * lag;
        P.topLeftCorner<2, 2>()     = observation_covariance(observation) + drift * drift * Eigen::Matrix2d::Identity();
        P.bottomRightCorner<2, 2>() = m.initial_velocity_std * m.initial_velocity_std * Eigen::Matrix2d::Identity();
        Eigen::Vector4d x           = Eigen::Vector4d::Zero();
        x.head<2>()                 = observation.rXFf;

        Track track;
        track.id                              = next_id++;
        track.target                          = observation.target;
        track.filter                          = utility::math::filter::SRIF<double, 4>(x, P);
        track.time                            = now;
        track.last_measurement                = observation.time;
        track.fusions                         = 1;
        track.observers[observation.observer] = observation.time;
        log<DEBUG>("New", observation.target == Target::BALL ? "ball" : "robot", "track", track.id);
        tracks.push_back(std::move(track));
        return tracks.back();
    }

    bool CooperativeLocalisation::corroborate(const Observation& observation) {
        // Forget pending detections too far from this one in time
        pending.erase(std::remove_if(pending.begin(),
                                     pending.end(),
                                     [&](const Pending& p) {
                                         return std::abs(seconds(observation.time - p.observation.time))
                                                > cfg.pending_window;
                                     }),
                      pending.end());

        const Eigen::Vector2d d2 = observation.rXFf - observation.rCFf;
        const double r2          = d2.norm();
        const Eigen::Vector2d u2 = d2 / std::max(r2, 0.05);
        const double s2          = range_std(observation.target, r2);

        for (auto it = pending.begin(); it != pending.end(); ++it) {
            const Observation& other = it->observation;
            if (other.observer == observation.observer || other.target != observation.target) {
                continue;
            }
            const Eigen::Vector2d d1 = other.rXFf - other.rCFf;
            const double r1          = d1.norm();
            const Eigen::Vector2d u1 = d1 / std::max(r1, 0.05);
            const double s1          = range_std(other.target, r1);

            // The two rays must cross in front of both robots, at a decent angle, near both measured ranges
            Eigen::Matrix2d A;
            A << u1, -u2;
            if (std::abs(A.determinant()) < std::sin(10.0 * M_PI / 180.0)) {
                continue;
            }
            const Eigen::Vector2d s = A.partialPivLu().solve(observation.rCFf - other.rCFf);
            if (s.x() <= 0.0 || s.y() <= 0.0 || std::abs(s.x() - r1) > 3.0 * s1 + 0.5
                || std::abs(s.y() - r2) > 3.0 * s2 + 0.5) {
                continue;
            }

            // Start the ball at the intersection along the first ray, then fuse the second detection
            Observation first = other;
            first.rXFf        = other.rCFf + s.x() * u1;
            pending.erase(it);
            Track& track = spawn(first);
            fuse(track, observation);
            log<DEBUG>("Ball track", track.id, "started from two robots' detections");
            return true;
        }

        pending.push_back(Pending{observation});
        return false;
    }

    void CooperativeLocalisation::process_scan(std::vector<Observation> observations) {
        const auto now = NUClear::clock::now();

        // Drop detections that are stale, off the field, or of a teammate
        observations.erase(std::remove_if(observations.begin(),
                                          observations.end(),
                                          [&](const Observation& o) {
                                              return seconds(now - o.time) > cfg.max_detection_age || !on_field(o.rXFf)
                                                     || (o.target == Target::ROBOT && is_teammate(o));
                                          }),
                           observations.end());
        if (observations.empty()) {
            return;
        }

        predict(now);

        // Association cost of every detection to every track of the same kind, infinite outside the gate
        const size_t n_dets   = observations.size();
        const size_t n_tracks = tracks.size();
        const double inf      = std::numeric_limits<double>::infinity();
        std::vector<std::vector<double>> cost(n_dets, std::vector<double>(n_tracks, inf));
        for (size_t i = 0; i < n_dets; ++i) {
            for (size_t k = 0; k < n_tracks; ++k) {
                if (tracks[k].target != observations[i].target) {
                    continue;
                }
                // The observer is never in its own detections, so a robot track sitting on it can't take them
                if (tracks[k].target == Target::ROBOT
                    && (tracks[k].position() - observations[i].rCFf).norm() < cfg.self_exclusion) {
                    continue;
                }
                const auto [d2, c] = gate(tracks[k], observations[i]);
                if (d2 <= cfg.gate_chi2) {
                    cost[i][k] = c;
                }
            }
        }

        // Greedy global nearest neighbour, one detection per track. Tracks are independent filters, so costs computed
        // before any fusion stay valid
        std::vector<long> assignment(n_dets, -1);
        while (true) {
            double best = inf;
            size_t bi = 0, bk = 0;
            for (size_t i = 0; i < n_dets; ++i) {
                for (size_t k = 0; k < n_tracks; ++k) {
                    if (cost[i][k] < best) {
                        best = cost[i][k];
                        bi   = i;
                        bk   = k;
                    }
                }
            }
            if (!std::isfinite(best)) {
                break;
            }
            assignment[bi] = long(bk);
            std::fill(cost[bi].begin(), cost[bi].end(), inf);
            for (auto& row : cost) {
                row[bk] = inf;
            }
        }

        // If this scan's ball detections all fail the ball track's gate too many scans in a row, the ball track is
        // wrong: end it and let the detection start a new one
        std::set<unsigned long> removed{};
        const bool has_ball_detection = std::any_of(observations.begin(), observations.end(), [](const auto& o) {
            return o.target == Target::BALL;
        });
        if (has_ball_detection) {
            bool ball_assigned = false;
            for (size_t i = 0; i < n_dets; ++i) {
                ball_assigned |= observations[i].target == Target::BALL && assignment[i] >= 0;
            }
            for (size_t k = 0; k < n_tracks; ++k) {
                if (tracks[k].target != Target::BALL) {
                    continue;
                }
                tracks[k].rejections = ball_assigned ? 0 : tracks[k].rejections + 1;
                if (tracks[k].rejections >= cfg.ball_reset_count) {
                    log<DEBUG>("Restarting ball track", tracks[k].id, "after repeated gate failures");
                    removed.insert(tracks[k].id);
                }
            }
        }

        // Fuse or start tracks. New tracks are appended, so indices into tracks stay valid.
        std::vector<unsigned long> took(n_dets, 0);
        for (size_t i = 0; i < n_dets; ++i) {
            const Observation& o = observations[i];
            if (assignment[i] >= 0) {
                Track& track = tracks[size_t(assignment[i])];
                fuse(track, o);
                took[i] = track.id;
                continue;
            }
            const double r = (o.rXFf - o.rCFf).norm();
            // Close to the observer the detection is dominated by the observer's own pose error
            if (o.target == Target::ROBOT && r < cfg.min_spawn_range) {
                continue;
            }
            // A poorly ranged ball never starts a track on its own; it waits for a second robot to agree
            if (o.target == Target::BALL && range_std(o.target, r) > cfg.spawn_max_range_std) {
                if (corroborate(o)) {
                    took[i] = tracks.back().id;
                }
                continue;
            }
            took[i] = spawn(o).id;
        }

        // Cannot-link: one scan sees each object at most once, so tracks that took different detections from the same
        // scan are different objects and must never be merged
        for (size_t a = 0; a < n_dets; ++a) {
            for (size_t b = a + 1; b < n_dets; ++b) {
                if (took[a] == 0 || took[b] == 0 || took[a] == took[b]) {
                    continue;
                }
                for (auto& track : tracks) {
                    if (track.id == took[a]) {
                        track.distinct.insert(took[b]);
                    }
                    else if (track.id == took[b]) {
                        track.distinct.insert(took[a]);
                    }
                }
            }
        }

        tracks.erase(
            std::remove_if(tracks.begin(), tracks.end(), [&](const Track& t) { return removed.contains(t.id); }),
            tracks.end());

        maintenance();
    }

    void CooperativeLocalisation::maintenance() {
        std::set<unsigned long> removed{};

        // Lost: the uncertainty has grown so much the track carries no information. Or it has left the field.
        for (const auto& track : tracks) {
            const double lost_std = track.target == Target::BALL ? cfg.ball_lost_std : cfg.robot_lost_std;
            if (track.max_std() > lost_std || !on_field(track.position())) {
                log<DEBUG>("Removing lost track", track.id);
                removed.insert(track.id);
            }
        }

        // Duplicates: two confident tracks of the same kind that agree are one object tracked twice. Keep the one with
        // more fusions (the older on a tie)
        for (size_t a = 0; a < tracks.size(); ++a) {
            for (size_t b = a + 1; b < tracks.size(); ++b) {
                const Track& ta = tracks[a];
                const Track& tb = tracks[b];
                if (ta.target != tb.target || removed.contains(ta.id) || removed.contains(tb.id)
                    || ta.distinct.contains(tb.id) || tb.distinct.contains(ta.id)) {
                    continue;
                }
                if (ta.max_std() > cfg.merge_max_std || tb.max_std() > cfg.merge_max_std) {
                    continue;
                }
                const Eigen::Vector2d dm = ta.position() - tb.position();
                const Eigen::Matrix2d S  = ta.position_covariance() + tb.position_covariance();
                if (dm.dot(S.ldlt().solve(dm)) > cfg.merge_chi2) {
                    continue;
                }
                const bool keep_a        = ta.fusions > tb.fusions || (ta.fusions == tb.fusions && ta.id < tb.id);
                const unsigned long drop = keep_a ? tb.id : ta.id;
                log<DEBUG>("Removing duplicate track", drop);
                removed.insert(drop);
            }
        }

        // There is only one ball: keep the ball track with the most information
        const Track* best_ball = nullptr;
        for (const auto& track : tracks) {
            if (track.target == Target::BALL && !removed.contains(track.id)
                && (best_ball == nullptr
                    || track.position_covariance().determinant() < best_ball->position_covariance().determinant())) {
                best_ball = &track;
            }
        }
        for (const auto& track : tracks) {
            if (track.target == Target::BALL && best_ball != nullptr && track.id != best_ball->id) {
                removed.insert(track.id);
            }
        }

        tracks.erase(
            std::remove_if(tracks.begin(), tracks.end(), [&](const Track& t) { return removed.contains(t.id); }),
            tracks.end());
    }

}  // namespace module::localisation
