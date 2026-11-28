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

/**
 * @file SideDisambiguator.hpp
 * @brief Out-of-field landmark map and field-side disambiguation.
 *
 * On-field landmarks cannot tell a pose from its 180 deg mirror, but the background can. This
 * builds a map of out-of-field corner landmarks online and compares how well the pose and its
 * mirror explain the corners in view. Sustained preference for the mirror requests a flip.
 *
 * Pipeline per video frame:
 *  1. FAST corners + ORB descriptors, classified out-of-field (OutOfFieldDetector).
 *  2. SNN association of out-of-field corners to map landmarks: surprisal
 *     (negative log predictive density in the ray tangent plane) nearest
 *     neighbour, descriptor-gated, greedy one-to-one, accepted while the
 *     surprisal beats the uniform clutter hypothesis.
 *  3. Robust side score at the pose and at its mirror: sum of
 *     log(inlier density / clutter density) over accepted associations.
 *     The difference accumulates into a forgetting log-likelihood ratio.
 *  4. Map maintenance (only while the pose is confident and the side is not in doubt):
 *     unmatched corners spawn bearing candidates, which are triangulated into landmarks in {f}
 *     once they have parallax. Dynamic objects fail a static-consistency test; landmarks
 *     repeatedly predicted visible but not seen are pruned.
 *
 * The map is only valid if built on the correct side, so building freezes whenever the
 * evidence favours the mirror.
 */
#ifndef MODULE_LOCALISATION_SRIF_SIDEDISAMBIGUATOR_HPP
#define MODULE_LOCALISATION_SRIF_SIDEDISAMBIGUATOR_HPP

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/core.hpp>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/OutOfFieldFeatures.hpp"
#include "srif/tangent_basis.hpp"

#include "message/input/Image.hpp"


namespace module::localisation::srif {

    class SideDisambiguator {
    public:
        /**
         * @brief Tuning options.
         */
        struct Options {
            // --- SNN association ---
            double sigma_angular   = 0.05;       ///< Corner bearing noise std dev [rad]
            double clutter_density = 5.0;        ///< Clutter density [features/sr]; below physical (~55) to
                                                 ///< tolerate estimator drift
            double pos_std_floor = 0.6;          ///< Floor on the camera position std used for gating [m]; the
                                                 ///< filter is optimistic. Applied to both sides alike.
            int max_descriptor_distance = 64;    ///< ORB Hamming gate for an association pair
            double pre_gate_angle       = 0.20;  ///< Dot-product pre-gate before surprisal [rad]
            int visible_margin          = 30;    ///< Predicted-visible only this far inside the image [px]
            double max_tangent_sigma    = 0.16;  ///< Skip landmarks whose predicted bearing sigma exceeds
                                                 ///< this [rad]; they cannot discriminate the mirror

            // --- candidate spawning and promotion ---
            double cand_gate_angle           = 0.10;  ///< Candidate re-match gate on ray angle in {f} [rad]
            int cand_max_descriptor_distance = 50;    ///< Hamming gate for growing a candidate track
            int min_obs                      = 4;     ///< Observations needed to attempt promotion
            double min_time_span             = 1.0;   ///< Track must span at least this long [s]
            double min_parallax              = 0.10;  ///< Max pairwise ray angle must exceed this [rad]

            // --- far (bearing-only) landmarks ---
            // Most background never gains reliable parallax, but its bearing alone discriminates
            // the mirror. Tight tracks promote at an assumed range with a large radial variance and
            // upgrade to points if parallax later accrues.
            int far_promote_obs          = 6;                    ///< Observations needed for a bearing-only promotion
            double far_promote_time_span = 2.0;                  ///< ... spanning at least this long [s]
            double far_max_spread        = 0.08;                 ///< Max RMS angle about the mean bearing [rad]
            double assumed_range         = 6.0;                  ///< Nominal range of a bearing-only landmark [m]
            double cand_max_age          = 3.0;                  ///< Drop candidates unmatched for this long [s]
            std::size_t max_candidates   = 600;                  ///< Candidate pool cap
            double max_elevation         = 50.0 * M_PI / 180.0;  ///< Ignore features above this elevation
                                                                 ///< [rad]; ceiling lights are near-symmetric

            // --- landmark maintenance / dynamic object rejection ---
            double sigma_static       = 0.04;    ///< Angular residual scale for the static-consistency test [rad]
            double static_chi2_mean   = 9.0;     ///< Reject when the mean normalised squared residual exceeds this
            int obs_window            = 12;      ///< Per-landmark observation window (re-triangulated each hit)
            int max_miss_streak       = 20;      ///< Prune after this many predicted-visible frames without a hit
            std::size_t max_landmarks = 500;     ///< Map cap; lowest hits/(1 + age/evict_half_life) evicted first
            double evict_half_life    = 20.0;    ///< Staleness that halves a landmark's cap score [s]. Decays
                                                 ///< rather than evicts, so landmarks behind the robot survive.
            double min_range            = 0.8;   ///< Reject triangulations closer than this [m]
            double max_range            = 40.0;  ///< Reject triangulations farther than this [m]
            double min_height_on_carpet = 1.5;   ///< Points over the carpet must be at least this high [m]
            double field_margin         = 0.30;  ///< Carpet margin matching the detector's classification [m]

            // --- map building gate ---
            double max_pos_std = 0.5;   ///< Build the map only below this horizontal position std [m]
            double freeze_llr  = -1.0;  ///< Freeze map building below this LLR
            double rebuild_llr = 5.0;   ///< After deep doubt, stay frozen until the LLR exceeds this;
                                        ///< forgetting alone is not evidence

            // --- side decision ---
            double forgetting          = 0.98;  ///< Per-frame forgetting factor on the LLR
            double llr_clamp           = 40.0;  ///< Clamp on the LLR [nats]
            double delta_clamp         = 6.0;   ///< Clamp on the per-frame LLR increment [nats]
            double max_yaw_rate        = 0.6;   ///< No LLR accumulation while turning faster than this [rad/s]
            double flip_threshold      = 25.0;  ///< Request a flip when the LLR is below -flip_threshold [nats]
            int flip_consecutive       = 10;    ///< ... for this many consecutive scored frames
            std::size_t min_flip_assoc = 3;     ///< ... each with at least this many mirror associations
            double flip_dominance      = 2.0;   ///< ... outnumbering own associations by this factor
            double flip_coverage       = 0.5;   ///< ... with own predicting at least this fraction of the
                                                ///< mirror's visible landmarks

            // --- blind-own flip escape ---
            // When the wrong-side pose faces unmapped territory flip_coverage can never be met. The
            // escape instead requires a near-clamped LLR, an essentially blind own pose that still
            // predicts landmarks (so it is contradicted, not merely uncovered), and no net ~180 deg
            // turn, which makes a symmetric background look mirrored.
            double flip_blind_llr                  = 35.0;      ///< LLR at or below -flip_blind_llr [nats]
            std::size_t flip_blind_own_max         = 1;         ///< ... at most this many own associations
            std::size_t flip_blind_min_assoc       = 2;         ///< ... at least this many mirror associations
            int flip_blind_consecutive             = 40;        ///< ... over this many (leaky) scored frames
            std::size_t flip_blind_min_visible_own = 1;         ///< Own must predict at least this many landmarks
            double blind_turn_tolerance = 60.0 * M_PI / 180.0;  ///< Refuse within this of a 180 deg net turn [rad]
            std::size_t blind_match_min_assoc = 10;   ///< Own counts as confirmed at this many associations ...
            double blind_match_fraction       = 0.1;  ///< ... and at least this fraction of its predicted-visible

            double flip_cooldown = 5.0;  ///< Freeze map building for this long after a flip [s]
        };

        /**
         * @brief A triangulated out-of-field landmark in the field frame {f}.
         */
        struct Landmark {
            Eigen::Vector3d rPFf;    ///< Triangulated (or assumed-range) position in {f}
            Eigen::Matrix3d P;       ///< Position covariance in {f} (huge radially if bearing-only)
            bool far = false;        ///< Bearing-only landmark (no reliable depth yet)
            cv::Mat descriptor;      ///< Latest matched ORB descriptor (1x32 CV_8U, owned)
            int hits         = 0;    ///< Number of associated observations
            int miss_streak  = 0;    ///< Consecutive predicted-visible frames without association
            double last_seen = 0.0;  ///< Time of the last associated observation [s]
            int last_status  = 0;    ///< LandmarkStatus this frame

            /// @brief One observation used for (re-)triangulation.
            struct Obs {
                Eigen::Vector3d rCFf;  ///< Camera position in {f}
                Eigen::Vector3d uFf;   ///< Unit ray towards the landmark in {f}
                double t;              ///< Observation time [s]
            };
            std::vector<Obs> obs;  ///< Bounded observation window
        };

        /**
         * @brief What happened to one detected corner this frame (visualiser colour key).
         *
         * Values are ordered by precedence: a corner that qualifies for several is
         * reported as the highest one.
         */
        enum FeatureStatus {
            FEATURE_ON_CARPET = 0,  ///< Masked out by the carpet classification: not a background feature
            FEATURE_UNMATCHED,      ///< Out-of-field, but nothing in the map or candidate pool claimed it
            FEATURE_CANDIDATE,      ///< Grew an existing candidate track (not a landmark yet)
            FEATURE_OUTLIER,        ///< Gated to a landmark but rejected: clutter explains it better, or
                                    ///< it lost the one-to-one assignment to a closer corner
            FEATURE_MIRROR,         ///< Associated only under the mirrored pose (wrong-side evidence)
            FEATURE_ASSOCIATED      ///< Associated to a map landmark at the current pose
        };

        /**
         * @brief What happened to one map landmark predicted into the current camera.
         */
        enum LandmarkStatus {
            LANDMARK_NOT_IN_VIEW = 0,  ///< Does not project into the image at the current pose
            LANDMARK_EDGE,             ///< Projects into the image but within visible_margin of the border,
                                       ///< so it is not counted as predicted-visible
            LANDMARK_AMBIGUOUS,        ///< Predicted bearing too smeared to discriminate the mirror
                                       ///< (max_tangent_sigma): excluded from matching and from scoring
            LANDMARK_MISSED,           ///< Predicted well inside the image, no corner associated with it
            LANDMARK_ASSOCIATED,       ///< Associated to a corner this frame
            LANDMARK_CULLED_MISSING,   ///< Culled this frame: max_miss_streak predicted-visible frames without a hit
            LANDMARK_CULLED_OUTLIER    ///< Culled this frame: its window stopped fitting one static point (mover)
        };

        /// @brief A map landmark projected into the camera image, with its per-frame status.
        struct LandmarkView {
            Eigen::Vector2d px       = Eigen::Vector2d::Zero();  ///< Predicted pixel position at the current pose
            Eigen::Vector2d match_px = Eigen::Vector2d::Zero();  ///< Pixel of the matched corner (ASSOCIATED only)
            int status               = LANDMARK_EDGE;            ///< LandmarkStatus
            bool far                 = false;                    ///< Bearing-only landmark (no reliable depth)
        };

        /**
         * @brief Per-frame result: association counts, side scores and the decision state.
         */
        struct FrameResult {
            std::size_t n_features          = 0;    ///< Detected corners
            std::size_t n_out_of_field      = 0;    ///< ... classified out-of-field
            std::size_t n_associated        = 0;    ///< ... associated to map landmarks at the current pose
            std::size_t n_associated_mirror = 0;    ///< ... associated at the mirrored pose
            std::size_t n_outlier           = 0;    ///< ... gated to a landmark but rejected (under either pose)
            std::size_t n_visible_own       = 0;    ///< Landmarks predicted well inside the image at the current pose
            std::size_t n_visible_mirror    = 0;    ///< ... at the mirrored pose
            std::size_t n_landmarks         = 0;    ///< Live map landmarks
            std::size_t n_candidates        = 0;    ///< Live bearing candidates
            double score_own                = 0.0;  ///< Robust side score at the current pose [nats]
            double score_mirror             = 0.0;  ///< Robust side score at the mirrored pose [nats]
            double side_delta               = 0.0;  ///< Clamped own-minus-mirror LLR increment [nats]; 0 if
                                                    ///< unscored. Input to add_side_log_evidence.
            double llr              = 0.0;          ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
            bool map_frozen         = false;        ///< Map building was frozen this frame
            bool flip_requested     = false;        ///< The evidence says the filter is on the wrong side
            double turn_since_match = 0.0;          ///< Net heading change since the own side was last confirmed [rad]
            bool blind_turn_blocked = false;        ///< The blind escape was refused because that turn explains
                                                    ///< the view
            std::vector<OutOfFieldFeature> features;   ///< The detected corners (for display)
            std::vector<int> feature_status;           ///< Per detected corner: a FeatureStatus
            std::vector<LandmarkView> landmark_views;  ///< Map landmarks projected into the image at the current pose
        };

        /**
         * @brief Construct the disambiguator.
         * @param lens Width-normalised lens calibration (shared with the detector), as carried on
         *             message::input::Image
         * @param dimensions Image dimensions in pixels {width, height}
         * @param dims Field dimensions (carpet extent)
         * @param options Tuning options
         */
        SideDisambiguator(const message::input::Image::Lens& lens,
                          const Eigen::Vector2d& dimensions,
                          const FieldDimensions& dims,
                          const Options& options);

        /// @brief Construct with default options.
        SideDisambiguator(const message::input::Image::Lens& lens,
                          const Eigen::Vector2d& dimensions,
                          const FieldDimensions& dims);

        /**
         * @brief Process one video frame.
         * @param t Frame time [s]
         * @param gray Grayscale camera frame (CV_8UC1, full lens resolution)
         * @param Tfc Estimated camera pose in {f} at the posterior mean
         * @param TfcMirror Camera pose under the 180 degree mirrored state
         * @param pos_std Horizontal position std of the filter [m] (map-building gate)
         * @param yaw_std Yaw std of the filter [rad] (association gating)
         * @param yaw_rate_abs Magnitude of the current yaw rate [rad/s] (turn gating)
         * @param heading Estimated torso yaw in {f} [rad] (for the blind-escape turn gate)
         * @return Association/score/decision summary for this frame
         */
        FrameResult process(double t,
                            const cv::Mat& gray,
                            const Eigen::Isometry3d& Tfc,
                            const Eigen::Isometry3d& TfcMirror,
                            double pos_std,
                            double yaw_std,
                            double yaw_rate_abs,
                            double heading);

        /**
         * @brief Notify that the filter state was mirrored (flip applied or kidnap injected).
         *
         * Negates the LLR, resets the flip streak and freezes map building for flip_cooldown.
         *
         * @param t Time the flip was applied [s]
         */
        void notify_flip_applied(double t);

        const std::vector<Landmark>& landmarks() const {
            return landmarks_;
        }

        /// @brief Cumulative diagnostics of the map-building funnel.
        struct Stats {
            std::size_t promote_attempts =
                0;  ///< Candidate windows that reached the parallax test with enough obs/time
            std::size_t parallax_wait        = 0;  ///< ... still waiting for parallax
            std::size_t tri_fail_geometry    = 0;  ///< ... normal matrix singular (rays too parallel)
            std::size_t tri_fail_range       = 0;  ///< ... solution behind a ray or out of range
            std::size_t tri_fail_chi2        = 0;  ///< ... inconsistent with one static point (dynamic object)
            std::size_t background_fail      = 0;  ///< ... triangulated onto the carpet (dynamic clutter)
            std::size_t promoted             = 0;  ///< ... promoted to point landmarks
            std::size_t promoted_far         = 0;  ///< ... promoted to bearing-only landmarks
            std::size_t far_spread_fail      = 0;  ///< ... bearing-only promotion rejected (jittery track)
            std::size_t upgraded             = 0;  ///< Bearing-only landmarks upgraded to points by later parallax
            std::size_t landmark_culled_chi2 = 0;  ///< Landmarks culled by the static-consistency re-test
            std::size_t landmark_culled_miss = 0;  ///< Landmarks culled by the miss-streak rule
        };
        const Stats& stats() const {
            return stats_;
        }

        Options options;

    private:
        /// @brief A bearing-only track waiting for enough parallax to triangulate.
        struct Candidate {
            cv::Mat descriptor;              ///< Latest matched descriptor (owned)
            std::vector<Landmark::Obs> obs;  ///< Observations so far
            double last_seen = 0.0;
        };

        /// @brief One accepted feature<->landmark association.
        struct Association {
            std::size_t feature;   ///< Index into the frame's feature vector
            std::size_t landmark;  ///< Index into landmarks_
            double surprisal;      ///< Negative log predictive density [nats]
        };

        /// @brief One landmark predicted into a camera, and what became of it.
        struct Prediction {
            std::size_t landmark;         ///< Index into landmarks_
            Eigen::Vector2d px;           ///< Predicted pixel position
            bool well_inside;             ///< Predicted at least visible_margin inside the image border
            bool ambiguous;               ///< Bearing too smeared to discriminate (max_tangent_sigma)
            bool associated     = false;  ///< Claimed by a corner in the one-to-one assignment
            std::size_t feature = 0;      ///< The claiming corner (associated only)
        };

        /**
         * @brief SNN association of out-of-field features against the map at a pose.
         * @param features Detected features (only out-of-field ones participate)
         * @param Tfc Camera pose to predict the landmarks under
         * @param pos_std Horizontal position std [m] (inflates the predictive covariance)
         * @param yaw_std Yaw std [rad] (inflates the predictive covariance)
         * @param score Output: robust side score, sum of log(inlier/clutter) density ratios
         * @param predictions Output: every landmark that projects into the image, with its outcome
         * @param feature_outlier Output (per feature): gated to a landmark but left unassociated
         */
        std::vector<Association> associate(const std::vector<OutOfFieldFeature>& features,
                                           const Eigen::Isometry3d& Tfc,
                                           double pos_std,
                                           double yaw_std,
                                           double& score,
                                           std::vector<Prediction>& predictions,
                                           std::vector<char>& feature_outlier) const;

        /// @brief Why a triangulation attempt was rejected (or OK).
        enum class TriResult { OK, GEOMETRY, RANGE, CHI2 };

        /**
         * @brief Triangulate a static point from an observation window by linear least squares.
         * @param obs Observations (camera positions and unit rays in {f})
         * @param rPFf Output: triangulated position
         * @param P Output: position covariance (from the ray geometry and sigma_static)
         * @param mean_chi2 Output: mean normalised squared perpendicular residual
         * @return OK if the window is consistent with one static in-range point, else the failure reason
         */
        TriResult triangulate(const std::vector<Landmark::Obs>& obs,
                              Eigen::Vector3d& rPFf,
                              Eigen::Matrix3d& P,
                              double& mean_chi2) const;

        /// @brief True if a triangulated point is plausible background (not on-carpet clutter).
        bool is_background_point(const Eigen::Vector3d& rPFf) const;

        /**
         * @brief Fit a bearing-only landmark from an observation window.
         *
         * Placed at assumed_range along the mean bearing with a large radial variance. Fails if
         * the bearings are not tight.
         */
        bool fit_far(const std::vector<Landmark::Obs>& obs, Eigen::Vector3d& rPFf, Eigen::Matrix3d& P) const;

        /**
         * @brief Grow candidate tracks with unmatched features; promote mature ones.
         * @param feature_grew_track Output (per feature): the corner extended an existing candidate track
         */
        void update_candidates(const std::vector<OutOfFieldFeature>& features,
                               const std::vector<bool>& feature_used,
                               const Eigen::Isometry3d& Tfc,
                               double t,
                               std::vector<char>& feature_grew_track);

        message::input::Image::Lens lens_;  ///< Width-normalised lens calibration
        Eigen::Vector2d dimensions_;        ///< Image dimensions in pixels {width, height}
        OutOfFieldDetector detector_;
        double half_carpet_length_;  ///< Field half-length + border strip + margin [m]
        double half_carpet_width_;   ///< Field half-width + border strip + margin [m]

        std::vector<Landmark> landmarks_;
        std::vector<Candidate> candidates_;

        double llr_                  = 0.0;    ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
        int flip_streak_             = 0;      ///< Consecutive scored frames at/below the flip threshold
        int blind_streak_            = 0;      ///< Leaky streak for the blind-own flip escape
        bool doubt_                  = false;  ///< Latched after deep doubt; cleared only by positive evidence
        double heading_at_own_match_ = 0.0;    ///< Heading when the map last confirmed the own side [rad]
        bool have_own_match_         = false;  ///< Whether heading_at_own_match_ has been set
        double map_freeze_until_     = -std::numeric_limits<double>::infinity();  ///< Post-flip map-building freeze [s]
        Stats stats_;                                                             ///< Map-building funnel diagnostics
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_SIDEDISAMBIGUATOR_HPP
