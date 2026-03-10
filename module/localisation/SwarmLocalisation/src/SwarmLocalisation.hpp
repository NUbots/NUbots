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
#ifndef MODULE_LOCALISATION_SWARMLOCALISATION_HPP
#define MODULE_LOCALISATION_SWARMLOCALISATION_HPP

#include <Eigen/Core>
#include <map>
#include <nuclear>
#include <set>
#include <vector>

namespace module::localisation {

    class SwarmLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            /// @brief Localisation cost below which a teammate is considered confident.
            /// Only confident teammates contribute detections to the opponent track table.
            double confidence_cost_threshold = 1.0;
            /// @brief Seconds after which a teammate entry is considered stale and pruned
            double stale_timeout_s = 5.0;
            /// @brief Minimum ball confidence (0–1) for a teammate's ball report to be accepted
            double min_ball_confidence = 0.3;
            /// @brief Distance [m] within which a detection in robocup.others is classified as
            /// a known teammate and excluded from the opponent detection list.
            /// robocup.others contains ALL detected robots (teammates + opponents).
            double teammate_filter_radius = 1.0;

            // --- Opponent KF track parameters ---

            /// @brief Mahalanobis distance gate for track-detection association.
            /// Detections with Mahalanobis distance > gate_distance are left unmatched.
            double gate_distance = 3.0;
            /// @brief Continuous-time position process noise [m²/s].
            /// Controls how fast position uncertainty grows between updates.
            double process_noise_pos = 0.5;
            /// @brief Continuous-time velocity process noise [m²/s³].
            /// Controls how fast velocity uncertainty grows between updates.
            double process_noise_vel = 2.0;
            /// @brief Position measurement noise [m²].
            /// Represents expected accuracy of a teammate's opponent detection in field frame.
            double measurement_noise = 0.25;
            /// @brief Minimum measurement updates before a track is reported in SwarmState.
            /// Suppresses spurious single-frame ghost detections.
            int min_observations = 3;
            /// @brief Minimum number of distinct source robots that must have updated a track
            /// before it is reported in SwarmState.
            /// This is the key cross-validation gate: a track seen only by one robot is very
            /// likely a misidentified teammate (race condition where the teammate hasn't
            /// broadcast yet and was not filtered). Two independent robots reporting the same
            /// position is strong evidence of a real opponent.
            int min_sources = 2;
            /// @brief Seconds without any update before a track is pruned from the table.
            double max_missed_s = 3.0;
            /// @brief Minimum seconds between updates from the same source to the same track.
            /// Prevents a single robot from over-weighting the filter relative to others.
            double min_source_interval_s = 0.2;

            // --- Ball KF parameters ---

            /// @brief Ball position process noise [m²/s] — uncertainty growth in x,y.
            double ball_process_noise = 1.0;
            /// @brief Ball velocity process noise [m²/s³] — how fast velocity can change.
            double ball_vel_process_noise = 2.0;
            /// @brief Ball position measurement noise [m²].
            double ball_measurement_noise = 0.5;
            /// @brief Seconds without any update before the ball KF is reset.
            double ball_max_missed_s = 2.0;

            /// @brief Maximum number of confirmed opponents to report in SwarmState/SwarmDebug.
            /// Tracks are ranked by observation count (most-seen first) so the most
            /// confident opponents are always reported. Set to 0 for unlimited.
            int max_opponents = 3;

            /// @brief Euclidean distance [m] below which an unmatched detection is considered
            /// to belong to an existing track and will NOT spawn a new track.
            /// Prevents the same physical robot from generating duplicate tentative tracks
            /// when measurements arrive with slight spatial spread.
            double merge_radius = 1.0;
        } cfg;

        /// @brief Internal state per teammate, keyed by player_id
        struct TeammateData {
            /// @brief Self-reported field-frame position (x, y, θ); z = yaw
            Eigen::Vector3d position_Ff{};
            /// @brief Localisation cost from this teammate's broadcast
            float cost = 0.0f;
            /// @brief Wall-clock time when this entry was last updated
            NUClear::clock::time_point timestamp{};
            /// @brief Ball position reported by this teammate in field frame
            Eigen::Vector3d ball_Ff{};
            /// @brief Ball confidence reported by this teammate (0.0–1.0)
            float ball_confidence = 0.0f;
            /// @brief True if this teammate reported a ball above min_ball_confidence
            bool ball_seen = false;
        };

        /// @brief A shared opponent track maintained by the distributed KF tracker.
        ///
        /// State vector: [x, y, vx, vy] in field frame.
        /// Uses a constant-velocity motion model with process noise to handle
        /// opponents accelerating/decelerating between updates.
        ///
        /// Multi-source updates: any confident teammate can contribute a detection
        /// to this track. Per-source rate limiting prevents over-weighting.
        struct OpponentTrack {
            int id = 0;
            /// @brief KF state [x, y, vx, vy] in field frame [m, m, m/s, m/s]
            Eigen::Vector4d state = Eigen::Vector4d::Zero();
            /// @brief KF error covariance (4×4)
            Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
            /// @brief Last time this track was propagated forward (for dt computation)
            NUClear::clock::time_point last_predict_time{};
            /// @brief Last time this track received a measurement update (for pruning)
            NUClear::clock::time_point last_update_time{};
            /// @brief Total measurement updates received; used to gate reporting
            int observation_count = 0;
            /// @brief Per-source (teammate player_id) last update timestamp.
            /// Used to enforce min_source_interval_s rate limiting.
            std::map<int, NUClear::clock::time_point> last_source_update{};
        };

        /// @brief Constant-velocity KF for ball, state [x, y, vx, vy] in field frame.
        /// Fuses position measurements from own robot + all confident teammates.
        struct BallKF {
            Eigen::Vector4d state      = Eigen::Vector4d::Zero();             // [x, y, vx, vy]
            Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity() * 9.0;  // wide prior
            NUClear::clock::time_point last_update_time{};
            bool seen = false;
        } ball_kf{};

        /// @brief Map from player_id → most recent teammate data
        std::map<int, TeammateData> teammate_data{};

        /// @brief Shared opponent track table, keyed by track_id
        std::map<int, OpponentTrack> opponent_tracks{};

        /// @brief Monotonically increasing ID counter for new tracks
        int next_track_id = 0;

        /// @brief Track IDs that have already been logged as confirmed (avoids repeated INFO logs)
        std::set<int> previously_confirmed_tracks{};

        /// @brief Last time the periodic position summary was logged
        NUClear::clock::time_point last_position_log_time{};

        /// @brief Remove teammate entries older than stale_timeout_s
        void prune_stale();

        /// @brief Propagate all tracks forward to `now` using the constant-velocity model
        void predict_tracks(NUClear::clock::time_point now);

        /// @brief Associate detections from `source_id` to existing tracks (KF update) or
        /// spawn new tentative tracks for unmatched detections.
        /// Uses nearest-neighbour assignment with Mahalanobis distance gating.
        void associate_and_update(int source_id,
                                  const std::vector<Eigen::Vector2d>& detections,
                                  NUClear::clock::time_point now);

        /// @brief Remove opponent tracks not updated for longer than max_missed_s
        void prune_tracks();

        /// @brief Update ball KF with a new observation from a confident teammate.
        void update_ball_kf(const Eigen::Vector2d& ball_pos, float reporter_cost);

        /// @brief Build and emit a SwarmState message from current data
        void emit_swarm_state();

        /// @brief Build and emit a SwarmDebug message for NUsight visualisation
        void emit_swarm_debug();

    public:
        /// @brief Called by the powerplant to build and set up the SwarmLocalisation reactor
        explicit SwarmLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_SWARMLOCALISATION_HPP
