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
 * @file SideDisambiguator.h
 * @brief Out-of-field landmark map and field-side disambiguation.
 *
 * The field is symmetric under a 180 degree rotation about its centre, so on-field
 * landmarks cannot tell own half from opponent half, but the background scenery can.
 * This builds a map of out-of-field corner landmarks and compares how well the current
 * pose and its 180 degree mirror explain the corners seen now. A sustained preference
 * for the mirror requests a flip.
 *
 * Per video frame:
 *  1. FAST corners + ORB descriptors, classified out-of-field (OutOfFieldDetector).
 *  2. Association of out-of-field corners to map landmarks by nearest surprisal,
 *     descriptor-gated, one-to-one, accepted while it beats the clutter hypothesis.
 *  3. Side score at the pose and at its mirror: the sum of log(inlier density / clutter
 *     density) over associations. The difference accumulates into a forgetting
 *     log-likelihood ratio (LLR).
 *  4. Map maintenance while the pose is confident and the side is not in doubt:
 *     unmatched corners become bearing candidates, candidates with enough parallax are
 *     triangulated into landmarks in {f}, and landmarks that do not fit a static point
 *     or are repeatedly not re-observed are pruned.
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

#include "message/input/Image.hpp"

#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/gaussian_filtering/rotation.hpp"

namespace module::localisation::srif {

    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::tangent_basis;

    class SideDisambiguator {
    public:
        /// @brief Tuning options
        struct Options {
            // SNN association
            double sigma_angular        = 0.05;  ///< Corner bearing noise std dev [rad]
            double clutter_density      = 5.0;   ///< Effective clutter density [features/steradian]
            double pos_std_floor        = 0.6;   ///< Floor on the camera position std used for gating [m]
            int max_descriptor_distance = 64;    ///< ORB Hamming gate for an association pair
            double pre_gate_angle       = 0.20;  ///< Cheap dot-product pre-gate before surprisal evaluation [rad]
            int visible_margin          = 30;  ///< Landmark counts as predicted-visible this far inside the image [px]
            double max_tangent_sigma    = 0.16;  ///< Skip landmarks whose predicted bearing sigma exceeds this [rad]

            // Candidate spawning and promotion
            double cand_gate_angle           = 0.10;  ///< Candidate re-match gate on ray angle in {f} [rad]
            int cand_max_descriptor_distance = 50;    ///< Stricter Hamming gate for growing a candidate track
            int min_obs                      = 4;     ///< Observations needed to attempt promotion
            double min_time_span             = 1.0;   ///< Track must span at least this long [s]
            double min_parallax              = 0.10;  ///< Max pairwise ray angle must exceed this [rad]

            // Far (bearing-only) landmarks: directionally tight tracks without parallax are
            // promoted at an assumed range with a large radial variance
            int far_promote_obs          = 6;     ///< Observations needed for a bearing-only promotion
            double far_promote_time_span = 2.0;   ///< ... spanning at least this long [s]
            double far_max_spread        = 0.08;  ///< RMS angle about the mean bearing must stay below this [rad]
            double assumed_range         = 6.0;   ///< Nominal range of a bearing-only landmark [m]
            double cand_max_age          = 3.0;   ///< Drop candidates unmatched for this long [s]
            std::size_t max_candidates   = 600;   ///< Candidate pool cap (oldest dropped first)
            double max_elevation         = 50.0 * M_PI / 180.0;  ///< Don't track features above this elevation [rad]

            // Landmark maintenance and outlier (dynamic object) rejection
            double sigma_static         = 0.04;  ///< Angular residual scale for the static-consistency test [rad]
            double static_chi2_mean     = 9.0;   ///< Reject when the mean normalised squared residual exceeds this
            int obs_window              = 12;    ///< Bounded per-landmark observation window
            int max_miss_streak         = 20;    ///< Prune after this many predicted-visible frames without a hit
            std::size_t max_landmarks   = 500;   ///< Map cap, lowest hits/(1 + age/evict_half_life) evicted first
            double evict_half_life      = 20.0;  ///< Staleness that halves a landmark's cap score [s]
            double min_range            = 0.8;   ///< Reject triangulations closer than this to the camera [m]
            double max_range            = 40.0;  ///< Reject triangulations farther than this [m]
            double min_height_on_carpet = 1.5;   ///< Points above the carpet in xy must be at least this high [m]
            double field_margin         = 0.30;  ///< Carpet margin matching the detector's classification [m]

            // Map building gate
            double max_pos_std = 0.5;   ///< Build the map only when horizontal position std is below this [m]
            double freeze_llr  = -1.0;  ///< Freeze map building when the accumulated LLR drops below this
            double rebuild_llr = 5.0;   ///< After a flip-level LLR, building stays frozen until the LLR is above this

            // Side decision
            double forgetting          = 0.98;  ///< Per-frame forgetting factor on the accumulated LLR
            double llr_clamp           = 40.0;  ///< Clamp on the accumulated LLR [nats]
            double delta_clamp         = 6.0;   ///< Clamp on the per-frame LLR increment [nats]
            double max_yaw_rate        = 0.6;   ///< Don't accumulate LLR while turning faster than this [rad/s]
            double flip_threshold      = 25.0;  ///< Request a flip when the LLR is below the negative of this [nats]
            int flip_consecutive       = 10;    ///< ... for at least this many consecutive scored frames
            std::size_t min_flip_assoc = 3;     ///< ... each backed by at least this many mirror-side associations
            double flip_dominance      = 2.0;   ///< ... and the mirror associations must outnumber own by this factor
            double flip_coverage       = 0.5;   ///< ... and own must have at least this fraction of the mirror's
                                                ///< predicted-visible landmarks

            // Blind-own flip escape: when own sees unmapped territory the coverage condition
            // above cannot be met, so a flip is also accepted on a deeper, longer LLR while own
            // has almost no associations and the mirror keeps matching
            double flip_blind_llr = 35.0;          ///< Blind path needs the LLR at or below the negative of this [nats]
            std::size_t flip_blind_own_max   = 1;  ///< ... with at most this many own-side associations
            std::size_t flip_blind_min_assoc = 2;  ///< ... and at least this many mirror-side associations
            int flip_blind_consecutive       = 40;  ///< ... over this many (leaky) scored frames

            // The escape requires that own predicts landmarks and fails to match them, and is
            // refused after a net turn near 180 deg, which makes a symmetric background look
            // mirrored
            std::size_t flip_blind_min_visible_own = 1;  ///< Own must predict at least this many landmarks in view
            double blind_turn_tolerance = 60.0 * M_PI / 180.0;  ///< Refuse the escape within this of a 180 deg net
                                                                ///< turn [rad]
            std::size_t blind_match_min_assoc = 10;   ///< Own counts as confirmed at this many associations ...
            double blind_match_fraction       = 0.1;  ///< ... and at least this fraction of its predicted-visible

            double flip_cooldown = 5.0;  ///< Freeze map building for this long after a flip [s]
        };

        /// @brief A triangulated out-of-field landmark in the field frame {f}
        struct Landmark {
            Eigen::Vector3d rPFf;    ///< Triangulated (or assumed-range) position in {f}
            Eigen::Matrix3d P;       ///< Position covariance in {f}
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

        /// @brief What happened to one detected corner this frame. A corner that qualifies for
        ///        several is reported as the highest.
        enum FeatureStatus {
            FEATURE_ON_CARPET = 0,  ///< Masked out by the carpet classification
            FEATURE_UNMATCHED,      ///< Out-of-field, claimed by no landmark or candidate
            FEATURE_CANDIDATE,      ///< Grew an existing candidate track
            FEATURE_OUTLIER,        ///< Gated to a landmark but rejected
            FEATURE_MIRROR,         ///< Associated only under the mirrored pose
            FEATURE_ASSOCIATED      ///< Associated to a map landmark at the current pose
        };

        /// @brief What happened to one map landmark predicted into the current camera
        enum LandmarkStatus {
            LANDMARK_NOT_IN_VIEW = 0,  ///< Does not project into the image at the current pose
            LANDMARK_EDGE,             ///< Projects within visible_margin of the image border
            LANDMARK_AMBIGUOUS,        ///< Predicted bearing sigma above max_tangent_sigma
            LANDMARK_MISSED,           ///< Predicted well inside the image, no corner associated with it
            LANDMARK_ASSOCIATED,       ///< Associated to a corner this frame
            LANDMARK_CULLED_MISSING,   ///< Culled this frame by the miss-streak rule
            LANDMARK_CULLED_OUTLIER    ///< Culled this frame by the static-consistency test
        };

        /// @brief A map landmark projected into the camera image, with its per-frame status.
        struct LandmarkView {
            Eigen::Vector2d px       = Eigen::Vector2d::Zero();  ///< Predicted pixel position at the current pose
            Eigen::Vector2d match_px = Eigen::Vector2d::Zero();  ///< Pixel of the matched corner (ASSOCIATED only)
            int status               = LANDMARK_EDGE;            ///< LandmarkStatus
            bool far                 = false;                    ///< Bearing-only landmark
        };

        /// @brief Per-frame result: association counts, side scores and the decision state
        struct FrameResult {
            std::size_t n_features          = 0;      ///< Detected corners
            std::size_t n_out_of_field      = 0;      ///< ... classified out-of-field
            std::size_t n_associated        = 0;      ///< ... associated to map landmarks at the current pose
            std::size_t n_associated_mirror = 0;      ///< ... associated at the mirrored pose
            std::size_t n_outlier           = 0;      ///< ... gated to a landmark but rejected (under either pose)
            std::size_t n_visible_own       = 0;      ///< Landmarks predicted well inside the image at the current pose
            std::size_t n_visible_mirror    = 0;      ///< ... at the mirrored pose
            std::size_t n_landmarks         = 0;      ///< Live map landmarks
            std::size_t n_candidates        = 0;      ///< Live bearing candidates
            double score_own                = 0.0;    ///< Robust side score at the current pose [nats]
            double score_mirror             = 0.0;    ///< Robust side score at the mirrored pose [nats]
            double side_delta               = 0.0;    ///< Clamped own-minus-mirror increment this frame [nats]
            double llr                      = 0.0;    ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
            bool map_frozen                 = false;  ///< Map building was frozen this frame
            bool flip_requested             = false;  ///< The evidence favours the mirror
            double turn_since_match         = 0.0;  ///< Net heading change since the own side was last confirmed [rad]
            bool blind_turn_blocked         = false;   ///< The blind escape was refused because of that turn
            std::vector<OutOfFieldFeature> features;   ///< The detected corners
            std::vector<int> feature_status;           ///< Per detected corner: a FeatureStatus
            std::vector<LandmarkView> landmark_views;  ///< Map landmarks projected into the image at the current pose
        };

        /**
         * @brief Construct the disambiguator.
         * @param lens Width-normalised lens calibration, as carried on message::input::Image
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
                            const Pose<double>& Tfc,
                            const Pose<double>& TfcMirror,
                            double pos_std,
                            double yaw_std,
                            double yaw_rate_abs,
                            double heading);

        /**
         * @brief Notify that the filter state was mirrored. Negates the accumulated LLR, resets
         *        the flip streak and freezes map building for the cooldown.
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
            bool ambiguous;               ///< Predicted bearing sigma above max_tangent_sigma
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
         * @param feature_outlier Output (per feature): the corner was gated to a landmark but
         *                       ended up unassociated
         */
        std::vector<Association> associate(const std::vector<OutOfFieldFeature>& features,
                                           const Pose<double>& Tfc,
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
         * @brief Fit a bearing-only landmark from an observation window, at the assumed range
         *        along the mean bearing. Fails if the bearings are not directionally tight.
         */
        bool fit_far(const std::vector<Landmark::Obs>& obs, Eigen::Vector3d& rPFf, Eigen::Matrix3d& P) const;

        /**
         * @brief Grow candidate tracks with unmatched features; promote mature ones.
         * @param feature_grew_track Output (per feature): the corner extended an existing candidate track
         */
        void update_candidates(const std::vector<OutOfFieldFeature>& features,
                               const std::vector<bool>& feature_used,
                               const Pose<double>& Tfc,
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
        bool doubt_                  = false;  ///< Latched at a flip-level LLR, cleared by positive evidence
        double heading_at_own_match_ = 0.0;    ///< Heading when the map last confirmed the own side [rad]
        bool have_own_match_         = false;  ///< Whether heading_at_own_match_ has been set
        double map_freeze_until_     = -std::numeric_limits<double>::infinity();  ///< Post-flip map-building freeze [s]
        Stats stats_;                                                             ///< Map-building funnel diagnostics
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_SIDEDISAMBIGUATOR_HPP
