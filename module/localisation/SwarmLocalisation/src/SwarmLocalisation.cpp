/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "SwarmLocalisation.hpp"

#include "extension/Configuration.hpp"

#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Swarm.hpp"
#include "message/localisation/SwarmDebug.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::input::RoboCup;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::SwarmDebug;
    using message::localisation::SwarmState;
    using message::localisation::SwarmTrackedOpponent;
    using message::localisation::TeammateState;
    using utility::nusight::graph;

    SwarmLocalisation::SwarmLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("SwarmLocalisation.yaml").then([this](const Configuration& config) {
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.confidence_cost_threshold = config["confidence_cost_threshold"].as<double>();
            cfg.stale_timeout_s           = config["stale_timeout_s"].as<double>();
            cfg.min_ball_confidence       = config["min_ball_confidence"].as<double>();
            cfg.teammate_filter_radius    = config["teammate_filter_radius"].as<double>();
            cfg.gate_distance             = config["gate_distance"].as<double>();
            cfg.process_noise_pos         = config["process_noise_pos"].as<double>();
            cfg.process_noise_vel         = config["process_noise_vel"].as<double>();
            cfg.measurement_noise         = config["measurement_noise"].as<double>();
            cfg.min_observations          = config["min_observations"].as<int>();
            cfg.min_sources               = config["min_sources"].as<int>();
            cfg.max_missed_s              = config["max_missed_s"].as<double>();
            cfg.min_source_interval_s     = config["min_source_interval_s"].as<double>();
            cfg.ball_process_noise        = config["ball_process_noise"].as<double>();
            cfg.ball_vel_process_noise    = config["ball_vel_process_noise"].as<double>();
            cfg.ball_measurement_noise    = config["ball_measurement_noise"].as<double>();
            cfg.ball_max_missed_s         = config["ball_max_missed_s"].as<double>();
            cfg.max_opponents             = config["max_opponents"].as<int>();
            cfg.merge_radius              = config["merge_radius"].as<double>();
        });

        // Process incoming RoboCup broadcasts from teammates.
        // RobotCommunication re-emits these locally; they are already filtered to
        // exclude our own robot and opposing-team robots.
        on<Trigger<RoboCup>>().then([this](const RoboCup& robocup) {
            const int player_id = static_cast<int>(robocup.current_pose.player_id);
            auto now            = NUClear::clock::now();

            // --- Update teammate state ---
            TeammateData& td = teammate_data[player_id];
            td.position_Ff   = Eigen::Vector3d(robocup.current_pose.position.x(),
                                             robocup.current_pose.position.y(),
                                             robocup.current_pose.position.z());  // z = yaw
            td.cost          = robocup.current_pose.cost;
            td.timestamp     = now;

            td.ball_confidence = static_cast<float>(robocup.ball.confidence);
            td.ball_seen       = td.ball_confidence >= static_cast<float>(cfg.min_ball_confidence);
            if (td.ball_seen) {
                td.ball_Ff =
                    Eigen::Vector3d(robocup.ball.position.x(), robocup.ball.position.y(), robocup.ball.position.z());
            }

            // Debug graphs
            emit(graph("Swarm/teammate_" + std::to_string(player_id) + "/cost", static_cast<double>(td.cost)));
            emit(graph("Swarm/teammate_" + std::to_string(player_id) + "/pos_x", td.position_Ff.x()));
            emit(graph("Swarm/teammate_" + std::to_string(player_id) + "/pos_y", td.position_Ff.y()));
            emit(graph("Swarm/teammate_" + std::to_string(player_id) + "/ball_confidence",
                       static_cast<double>(td.ball_confidence)));

            const bool confident = td.cost < static_cast<float>(cfg.confidence_cost_threshold);

            // --- Opponent track updates ---
            // IMPORTANT: only accept detections from well-localised teammates.
            // A robot that doesn't know where it is will map opponent positions wrongly
            // in field frame, poisoning the shared track table.
            if (confident) {
                // robocup.others contains visually-detected robots. Filter out any
                // detection that is within teammate_filter_radius of a known teammate's
                // self-reported position — these are likely misidentified teammates that
                // haven't been confirmed via RoboCup association yet.
                std::vector<Eigen::Vector2d> filtered_detections;
                for (const auto& other : robocup.others) {
                    const Eigen::Vector2d det(other.position.x(), other.position.y());
                    bool near_teammate = false;
                    for (const auto& [tm_id, tm_data] : teammate_data) {
                        if ((det - tm_data.position_Ff.head<2>()).norm() < cfg.teammate_filter_radius) {
                            near_teammate = true;
                            break;
                        }
                    }
                    if (!near_teammate) {
                        filtered_detections.push_back(det);
                    }
                }

                emit(graph("Swarm/teammate_" + std::to_string(player_id) + "/num_detections",
                           static_cast<int>(filtered_detections.size())));
                log<DEBUG>("Swarm: teammate ",
                           player_id,
                           " confident, contributing ",
                           filtered_detections.size(),
                           " detections (filtered from ",
                           robocup.others.size(),
                           " others)");

                if (!filtered_detections.empty()) {
                    predict_tracks(now);
                    associate_and_update(player_id, filtered_detections, now);
                }

                // Update ball KF with this teammate's ball report
                if (td.ball_seen) {
                    update_ball_kf(td.ball_Ff.head<2>(), td.cost);
                }
            }
            else {
                // Teammate cost too high — skip their detections this frame.
                // (Not logged: fires every frame until the teammate localises.)
            }

            prune_stale();
            prune_tracks();
            emit_swarm_state();
            emit_swarm_debug();
        });

        // Include the robot's own ball estimate in the fused ball KF.
        // RobotCommunication filters out own RoboCup broadcasts, so we subscribe to
        // the local Ball message directly to include self as a measurement source.
        on<Trigger<Ball>, Optional<With<Field>>>().then(
            [this](const Ball& ball, const std::shared_ptr<const Field>& field) {
                if (ball.confidence < cfg.min_ball_confidence) {
                    return;
                }
                if (!field) {
                    return;
                }

                // Transform ball position from world frame to field frame
                const Eigen::Isometry3d Hfw(field->Hfw);
                const Eigen::Vector2d ball_Ff = (Hfw * ball.rBWw).head<2>();

                // Use own localisation cost to weight this measurement
                update_ball_kf(ball_Ff, static_cast<float>(field->cost));
                emit_swarm_state();
                emit_swarm_debug();
            });
    }

    // ---------------------------------------------------------------------------
    // KF track management
    // ---------------------------------------------------------------------------

    void SwarmLocalisation::predict_tracks(NUClear::clock::time_point now) {
        for (auto& [id, track] : opponent_tracks) {
            if (track.last_predict_time == NUClear::clock::time_point{}) {
                // Track was just created; nothing to predict
                track.last_predict_time = now;
                continue;
            }

            double dt = std::chrono::duration<double>(now - track.last_predict_time).count();
            if (dt < 0.005) {
                continue;  // skip trivially small steps
            }

            // Constant-velocity state transition: x += vx*dt, y += vy*dt
            Eigen::Matrix4d F        = Eigen::Matrix4d::Identity();
            F(0, 2)                  = dt;
            F(1, 3)                  = dt;

            // Discrete-time process noise derived from continuous-time spectral densities.
            // Position noise: q_pos * [dt, 0, dt²/2, 0; ...]
            // Velocity noise: q_vel * [0, 0, dt, 0; ...]
            const double dt2         = dt * dt;
            Eigen::Matrix4d Q        = Eigen::Matrix4d::Zero();
            Q(0, 0)                  = cfg.process_noise_pos * dt;
            Q(1, 1)                  = cfg.process_noise_pos * dt;
            Q(0, 2) = Q(2, 0)       = cfg.process_noise_pos * dt2 * 0.5;
            Q(1, 3) = Q(3, 1)       = cfg.process_noise_pos * dt2 * 0.5;
            Q(2, 2)                  = cfg.process_noise_vel * dt;
            Q(3, 3)                  = cfg.process_noise_vel * dt;

            track.state              = F * track.state;
            track.covariance         = F * track.covariance * F.transpose() + Q;
            track.last_predict_time  = now;
        }
    }

    void SwarmLocalisation::associate_and_update(int source_id,
                                                 const std::vector<Eigen::Vector2d>& detections,
                                                 NUClear::clock::time_point now) {
        // Measurement matrix: we observe [x, y] from the 4D state [x, y, vx, vy]
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0)                       = 1.0;
        H(1, 1)                       = 1.0;

        // Scale measurement noise by reporter's localisation cost: higher cost → less trust.
        // cost=0 (perfect) → factor=1.0; cost=confidence_threshold → factor=2.0.
        double reporter_cost =
            teammate_data.count(source_id) ? static_cast<double>(teammate_data.at(source_id).cost) : cfg.confidence_cost_threshold;
        double cost_factor          = 1.0 + (reporter_cost / std::max(cfg.confidence_cost_threshold, 1e-6));
        const Eigen::Matrix2d R     = Eigen::Matrix2d::Identity() * cfg.measurement_noise * cost_factor;

        std::vector<bool> matched(detections.size(), false);

        for (auto& [id, track] : opponent_tracks) {
            // Rate-limit: same source cannot update same track more often than min_source_interval_s.
            // This prevents a single well-positioned robot from dominating the filter estimate.
            auto src_it = track.last_source_update.find(source_id);
            if (src_it != track.last_source_update.end()) {
                double since_last = std::chrono::duration<double>(now - src_it->second).count();
                if (since_last < cfg.min_source_interval_s) {
                    continue;
                }
            }

            // Innovation covariance S = H P H' + R
            const Eigen::Matrix2d S     = H * track.covariance * H.transpose() + R;
            const Eigen::Matrix2d S_inv = S.inverse();

            // Nearest-neighbour assignment: find closest unmatched detection within gate
            int best_det    = -1;
            double best_dist = cfg.gate_distance;
            for (size_t i = 0; i < detections.size(); ++i) {
                if (matched[i]) {
                    continue;
                }
                const Eigen::Vector2d innov = detections[i] - H * track.state;
                const double mahal =
                    std::sqrt(std::max(0.0, static_cast<double>((innov.transpose() * S_inv * innov)(0, 0))));
                if (mahal < best_dist) {
                    best_dist = mahal;
                    best_det  = static_cast<int>(i);
                }
            }

            if (best_det >= 0) {
                const Eigen::Vector2d innov       = detections[best_det] - H * track.state;
                const Eigen::Matrix<double, 4, 2> K = track.covariance * H.transpose() * S_inv;

                track.state += K * innov;

                // Joseph stabilised form for numerical symmetry
                const Eigen::Matrix4d IKH = Eigen::Matrix4d::Identity() - K * H;
                track.covariance          = IKH * track.covariance * IKH.transpose() + K * R * K.transpose();

                track.last_update_time              = now;
                track.last_source_update[source_id] = now;
                ++track.observation_count;
                matched[best_det] = true;

                // KF update logged via graph() — no per-frame log needed.
            }
        }

        // Spawn new tentative tracks for unmatched detections.
        // Hard Euclidean pre-gate: if a detection is within merge_radius of any existing track
        // (even one we couldn't update due to rate limiting), treat it as that track and skip
        // spawning a duplicate. This prevents the same physical robot creating N tracks when
        // multiple measurements arrive slightly spread out.
        for (size_t i = 0; i < detections.size(); ++i) {
            if (matched[i]) {
                continue;
            }

            // Check if close enough to any existing track to skip spawning
            bool near_existing = false;
            for (const auto& [existing_id, existing_track] : opponent_tracks) {
                const double dist = (existing_track.state.head<2>() - detections[i]).norm();
                if (dist < cfg.merge_radius) {
                    near_existing = true;
                    break;
                }
            }
            if (near_existing) {
                continue;
            }

            const int new_id = next_track_id++;

            OpponentTrack new_track;
            new_track.id                           = new_id;
            new_track.state.head<2>()              = detections[i];
            // Initial covariance: position initialised from measurement noise,
            // velocity initialised to a wide prior (we have no velocity info yet)
            new_track.covariance                   = Eigen::Matrix4d::Zero();
            new_track.covariance.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * cfg.measurement_noise * 4.0;
            new_track.covariance.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * 4.0;
            new_track.last_predict_time            = now;
            new_track.last_update_time             = now;
            new_track.last_source_update[source_id] = now;
            new_track.observation_count            = 1;

            log<DEBUG>("Swarm: new track ", new_id, " at (", detections[i].x(), ",", detections[i].y(), ")");
            opponent_tracks[new_id] = std::move(new_track);
        }
    }

    void SwarmLocalisation::prune_tracks() {
        auto now = NUClear::clock::now();
        for (auto it = opponent_tracks.begin(); it != opponent_tracks.end();) {
            const double age = std::chrono::duration<double>(now - it->second.last_update_time).count();
            if (age > cfg.max_missed_s) {
                log<DEBUG>("Swarm: pruning track ", it->first, " (age=", age, "s)");
                it = opponent_tracks.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    // ---------------------------------------------------------------------------
    // Teammate pruning
    // ---------------------------------------------------------------------------

    void SwarmLocalisation::prune_stale() {
        auto now      = NUClear::clock::now();
        auto deadline = now - std::chrono::duration_cast<NUClear::clock::duration>(
                                  std::chrono::duration<double>(cfg.stale_timeout_s));

        for (auto it = teammate_data.begin(); it != teammate_data.end();) {
            if (it->second.timestamp < deadline) {
                log<DEBUG>("Swarm: pruning stale teammate ", it->first);
                it = teammate_data.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    // ---------------------------------------------------------------------------
    // State emission
    // ---------------------------------------------------------------------------

    void SwarmLocalisation::emit_swarm_state() {
        auto swarm = std::make_unique<SwarmState>();

        // --- Teammates ---
        int num_confident            = 0;
        float best_ball_conf         = 0.0f;
        uint32_t best_ball_id        = 0;
        Eigen::Vector3d best_ball_Ff = Eigen::Vector3d::Zero();
        bool any_ball_seen           = false;

        for (const auto& [player_id, td] : teammate_data) {
            TeammateState ts;
            ts.player_id   = static_cast<uint32_t>(player_id);
            ts.position_Ff = Eigen::Vector3f(static_cast<float>(td.position_Ff.x()),
                                             static_cast<float>(td.position_Ff.y()),
                                             static_cast<float>(td.position_Ff.z()));
            ts.cost        = td.cost;
            ts.confident   = td.cost < static_cast<float>(cfg.confidence_cost_threshold);
            if (ts.confident) {
                ++num_confident;
            }
            swarm->teammates.push_back(ts);

            if (td.ball_seen && td.ball_confidence > best_ball_conf) {
                best_ball_conf = td.ball_confidence;
                best_ball_id   = static_cast<uint32_t>(player_id);
                best_ball_Ff   = td.ball_Ff;
                any_ball_seen  = true;
            }
        }

        // --- Opponent tracks ---
        // Collect confirmed tracks, sort by observation count (most confident first),
        // then cap at max_opponents to avoid flooding with false positives.
        int num_tentative = 0;
        std::vector<const OpponentTrack*> confirmed_tracks;
        for (const auto& [id, track] : opponent_tracks) {
            const int num_src = static_cast<int>(track.last_source_update.size());
            if (track.observation_count >= cfg.min_observations && num_src >= cfg.min_sources) {
                confirmed_tracks.push_back(&track);
            }
            else {
                ++num_tentative;
            }
        }
        std::sort(confirmed_tracks.begin(), confirmed_tracks.end(), [](const OpponentTrack* a, const OpponentTrack* b) {
            return a->observation_count > b->observation_count;
        });
        if (cfg.max_opponents > 0 && static_cast<int>(confirmed_tracks.size()) > cfg.max_opponents) {
            confirmed_tracks.resize(cfg.max_opponents);
        }
        const int num_confirmed = static_cast<int>(confirmed_tracks.size());
        for (const auto* track : confirmed_tracks) {
            swarm->opponent_positions_Ff.push_back(
                Eigen::Vector3f(static_cast<float>(track->state.x()), static_cast<float>(track->state.y()), 0.0f));
        }

        // --- Ball ---
        swarm->ball_seen        = any_ball_seen;
        swarm->ball_confidence  = best_ball_conf;
        swarm->ball_reporter_id = best_ball_id;
        if (any_ball_seen) {
            swarm->ball_position_Ff = Eigen::Vector3f(static_cast<float>(best_ball_Ff.x()),
                                                      static_cast<float>(best_ball_Ff.y()),
                                                      static_cast<float>(best_ball_Ff.z()));
        }

        // Summary debug graphs
        emit(graph("Swarm/num_teammates", static_cast<int>(teammate_data.size())));
        emit(graph("Swarm/num_confident_teammates", num_confident));
        emit(graph("Swarm/num_tracks_total", static_cast<int>(opponent_tracks.size())));
        emit(graph("Swarm/num_tracks_confirmed", num_confirmed));
        emit(graph("Swarm/num_tracks_tentative", num_tentative));
        emit(graph("Swarm/ball_seen", any_ball_seen ? 1 : 0));
        emit(graph("Swarm/ball_confidence", static_cast<double>(best_ball_conf)));

        emit(std::move(swarm));
    }

    // ---------------------------------------------------------------------------
    // Ball KF
    // ---------------------------------------------------------------------------

    void SwarmLocalisation::update_ball_kf(const Eigen::Vector2d& ball_pos, float reporter_cost) {
        auto now = NUClear::clock::now();

        // Reset if stale
        if (ball_kf.seen) {
            double age = std::chrono::duration<double>(now - ball_kf.last_update_time).count();
            if (age > cfg.ball_max_missed_s) {
                ball_kf.state      = Eigen::Vector4d::Zero();
                ball_kf.covariance = Eigen::Matrix4d::Identity() * 9.0;
                ball_kf.seen       = false;
            }
            else {
                // Predict: constant-velocity model x' = F*x, P' = F*P*F^T + Q
                Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
                F(0, 2)           = age;  // x  += vx * dt
                F(1, 3)           = age;  // y  += vy * dt

                Eigen::Matrix4d Q          = Eigen::Matrix4d::Zero();
                Q(0, 0) = Q(1, 1)          = cfg.ball_process_noise * age;      // position diffusion
                Q(2, 2) = Q(3, 3)          = cfg.ball_vel_process_noise * age;  // velocity diffusion

                ball_kf.state      = F * ball_kf.state;
                ball_kf.covariance = F * ball_kf.covariance * F.transpose() + Q;
            }
        }

        // Measurement noise scaled by reporter confidence (higher cost = noisier report)
        double cost_factor =
            1.0 + (static_cast<double>(reporter_cost) / std::max(cfg.confidence_cost_threshold, 1e-6));
        const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * cfg.ball_measurement_noise * cost_factor;

        // H = [1 0 0 0 ; 0 1 0 0] — observe position only from [x, y, vx, vy]
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0)                        = 1.0;
        H(1, 1)                        = 1.0;

        const Eigen::Matrix<double, 4, 2> PH_t = ball_kf.covariance * H.transpose();
        const Eigen::Matrix2d S                  = H * PH_t + R;
        const Eigen::Matrix<double, 4, 2> K     = PH_t * S.inverse();
        const Eigen::Vector2d innov              = ball_pos - H * ball_kf.state;

        ball_kf.state           += K * innov;
        ball_kf.covariance       = (Eigen::Matrix4d::Identity() - K * H) * ball_kf.covariance;
        ball_kf.last_update_time = now;
        ball_kf.seen             = true;
    }

    // ---------------------------------------------------------------------------
    // SwarmDebug emission
    // ---------------------------------------------------------------------------

    void SwarmLocalisation::emit_swarm_debug() {
        auto debug = std::make_unique<SwarmDebug>();

        // --- Confirmed opponent tracks with covariance ---
        // Collect, sort by observation count descending, cap at max_opponents.
        std::vector<const OpponentTrack*> confirmed_tracks;
        for (const auto& [id, track] : opponent_tracks) {
            const int num_src = static_cast<int>(track.last_source_update.size());
            if (track.observation_count >= cfg.min_observations && num_src >= cfg.min_sources) {
                // Log once when a track first becomes confirmed
                if (previously_confirmed_tracks.find(id) == previously_confirmed_tracks.end()) {
                    previously_confirmed_tracks.insert(id);
                    log<INFO>("Swarm: track ",
                              id,
                              " CONFIRMED at (",
                              track.state.x(),
                              ", ",
                              track.state.y(),
                              ") obs=",
                              track.observation_count,
                              " sources=",
                              num_src);
                }
                confirmed_tracks.push_back(&track);
            }
        }
        std::sort(confirmed_tracks.begin(), confirmed_tracks.end(), [](const OpponentTrack* a, const OpponentTrack* b) {
            return a->observation_count > b->observation_count;
        });
        if (cfg.max_opponents > 0 && static_cast<int>(confirmed_tracks.size()) > cfg.max_opponents) {
            confirmed_tracks.resize(cfg.max_opponents);
        }
        for (const auto* track : confirmed_tracks) {
            SwarmTrackedOpponent opp;
            opp.position_Ff = Eigen::Vector3f(static_cast<float>(track->state.x()),
                                              static_cast<float>(track->state.y()),
                                              0.0f);
            Eigen::Matrix2f cov2 = track->covariance.block<2, 2>(0, 0).cast<float>();
            opp.covariance       = cov2;
            debug->opponents.push_back(std::move(opp));
        }

        // --- Teammate self-reported positions and costs ---
        for (const auto& [player_id, td] : teammate_data) {
            debug->teammate_positions_Ff.push_back(
                Eigen::Vector3f(static_cast<float>(td.position_Ff.x()),
                                static_cast<float>(td.position_Ff.y()),
                                0.0f));
            debug->teammate_costs.push_back(td.cost);
        }

        // --- Ball KF state ---
        debug->ball_seen = ball_kf.seen;
        if (ball_kf.seen) {
            debug->ball_position_Ff =
                Eigen::Vector3f(static_cast<float>(ball_kf.state.x()),
                                static_cast<float>(ball_kf.state.y()),
                                0.0f);
            debug->ball_covariance = ball_kf.covariance.block<2, 2>(0, 0).cast<float>();
        }

        // Rate-limited INFO summary: print positions every ~1 second so the terminal
        // shows what NUsight should be visualising without spamming every frame.
        auto now_log = NUClear::clock::now();
        double since_last_log =
            last_position_log_time == NUClear::clock::time_point{}
                ? 999.0
                : std::chrono::duration<double>(now_log - last_position_log_time).count();

        if (since_last_log >= 1.0 && (!debug->opponents.empty() || debug->ball_seen)) {
            last_position_log_time = now_log;
            log<INFO>("SwarmDebug: ",
                      debug->opponents.size(),
                      " confirmed opponents, ",
                      debug->teammate_positions_Ff.size(),
                      " teammates, ball_seen=",
                      debug->ball_seen);
            for (size_t i = 0; i < debug->opponents.size(); ++i) {
                log<INFO>("  opponent[",
                          i,
                          "]: (",
                          debug->opponents[i].position_Ff.x(),
                          ", ",
                          debug->opponents[i].position_Ff.y(),
                          ")");
            }
            if (debug->ball_seen) {
                log<INFO>("  ball: (",
                          debug->ball_position_Ff.x(),
                          ", ",
                          debug->ball_position_Ff.y(),
                          ")");
            }
        }

        emit(std::move(debug));
    }

}  // namespace module::localisation
