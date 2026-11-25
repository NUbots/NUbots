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
 * The field is symmetric under a 180 degree rotation about its centre, so
 * on-field landmarks cannot tell own-half from opponent-half. The background
 * scenery (walls, posters, furniture, ceiling structure) is not symmetric.
 * This module builds a persistent map of out-of-field corner landmarks online
 * and continuously compares how well the current pose estimate and its 180
 * degree mirror explain the corners seen now. A decisive, sustained preference
 * for the mirror indicates the filter is on the wrong side (mid-game kidnap /
 * wrong-side convergence) and requests a flip.
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
 *  4. Map maintenance (only while the pose is confident and the side is not
 *     in doubt): unmatched corners spawn bearing candidates; re-observed
 *     candidates with enough parallax are triangulated by linear least squares
 *     into 3D landmarks in {f}. Dynamic objects (crowd, robots, handlers) are
 *     rejected by a static-consistency test: the bounded observation window
 *     must fit a single static point (normalised triangulation residual),
 *     and landmarks that are repeatedly predicted visible but not re-observed
 *     are pruned.
 *
 * The map is only trustworthy if it was anchored while the filter was on the
 * correct side (e.g. built early in the game, when the start-in-own-half prior
 * guarantees the side). Map building therefore freezes whenever the accumulated
 * evidence starts favouring the mirror.
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
        /**
         * @brief Tuning options.
         */
        struct Options {
            // --- SNN association ---
            double sigma_angular =
                0.05;  ///< Corner bearing noise std dev [rad] (~3 deg: pixel noise + unmodelled pose drift)
            double clutter_density = 5.0;        ///< Effective clutter density [features/steradian] (physical is ~55;
                                                 ///< lower widens acceptance so the true side survives estimator drift)
            double pos_std_floor = 0.6;          ///< Floor on the camera position std used for gating [m]: the filter
                                                 ///< is known to be optimistic, and a too-tight gate starves the TRUE
                                                 ///< side of matches whenever the estimate drifts (both sides get the
                                                 ///< same widening, so the comparison stays fair)
            int max_descriptor_distance = 64;    ///< ORB Hamming gate for an association pair
            double pre_gate_angle       = 0.20;  ///< Cheap dot-product pre-gate before surprisal evaluation [rad]
            int visible_margin          = 30;  ///< Landmark counts as predicted-visible this far inside the image [px]
            double max_tangent_sigma    = 0.16;  ///< Skip landmarks whose predicted bearing sigma exceeds this [rad]:
                                                 ///< a bearing-only landmark viewed far from its anchor spans an
                                                 ///< acceptance corridor that aliases; it carries no side information
                                                 ///< at this baseline and must neither match nor count as missed

            // --- candidate spawning and promotion ---
            double cand_gate_angle =
                0.10;  ///< Candidate re-match gate on ray angle in {f} [rad] (rotation cancels; covers pose jitter)
            int cand_max_descriptor_distance = 50;   ///< Stricter Hamming gate for growing a candidate track
            int min_obs                      = 4;    ///< Observations needed to attempt promotion
            double min_time_span             = 1.0;  ///< Track must span at least this long [s]
            double min_parallax =
                0.10;  ///< Max pairwise ray angle must exceed this [rad] (~6 deg; keeps depth above pose noise)

            // --- far (bearing-only) landmarks ---
            // Most background structure never accrues reliable parallax (pose jitter
            // is the same order as the true parallax), yet its bearing alone fully
            // discriminates the 180 deg mirror. Directionally tight tracks therefore
            // promote as bearing-only landmarks at an assumed range with a large
            // radial variance; the SNN tangent-plane covariance handles the missing
            // depth, and later parallax upgrades them to proper points.
            int far_promote_obs          = 6;     ///< Observations needed for a bearing-only promotion
            double far_promote_time_span = 2.0;   ///< ... spanning at least this long [s]
            double far_max_spread        = 0.08;  ///< RMS angle about the mean bearing must stay below this [rad]
            double assumed_range         = 6.0;   ///< Nominal range of a bearing-only landmark [m]
            double cand_max_age          = 3.0;   ///< Drop candidates unmatched for this long [s]
            std::size_t max_candidates   = 600;   ///< Candidate pool cap (oldest dropped first)
            double max_elevation =
                50.0 * M_PI / 180.0;  ///< Don't track features above this elevation [rad]:
                                      ///< overhead lighting grids are near-symmetric under the field's
                                      ///< 180 deg rotation and would feed the mirror hypothesis

            // --- landmark maintenance / outlier (dynamic object) rejection ---
            double sigma_static =
                0.04;  ///< Angular residual scale for the static-consistency test [rad] (incl. pose drift)
            double static_chi2_mean   = 9.0;   ///< Reject when the mean normalised squared residual exceeds this
            int obs_window            = 12;    ///< Bounded per-landmark observation window (re-triangulated each hit)
            int max_miss_streak       = 20;    ///< Prune after this many predicted-visible frames without a hit
            std::size_t max_landmarks = 500;   ///< Map cap (lowest-scoring evicted first, see evict_half_life)
            double evict_half_life    = 20.0;  ///< Staleness that halves a landmark's cap score [s].
                                               ///< The map sits pinned at max_landmarks for most of a run, so the
                                               ///< cap -- not the miss rule -- is the dominant eviction path, and
                                               ///< ranking on lifetime hits alone let a landmark that earned its
                                               ///< hits early outrank a fresh one that is matching NOW. Scoring
                                               ///< hits/(1 + age/half_life) keeps established landmarks (50 hits
                                               ///< unseen 100 s still beats 5 hits seen now) while letting truly
                                               ///< dead ones fall out. Not pure recency on purpose: a landmark
                                               ///< behind the robot is not failing, and the mirror test needs
                                               ///< coverage in every direction, so staleness must decay influence
                                               ///< rather than decide eviction outright.
            double min_range = 0.8;            ///< Reject triangulations closer than this to the camera [m]
            double max_range = 40.0;           ///< Reject triangulations farther than this [m]
            double min_height_on_carpet =
                1.5;                     ///< Points above the carpet in xy must be at least this high [m] (ceiling)
            double field_margin = 0.30;  ///< Carpet margin matching the detector's classification [m]

            // --- map building gate ---
            double max_pos_std = 0.5;   ///< Build the map only when horizontal position std is below this [m]
            double freeze_llr  = -1.0;  ///< Freeze map building when the accumulated LLR drops below this
            double rebuild_llr = 5.0;   ///< After deep doubt (LLR past the flip threshold), building stays
                                        ///< frozen until the LLR earns its way back above this: forgetting
                                        ///< decays the LLR towards zero on its own, and decay is not
                                        ///< evidence, so it must never re-arm building on a doubtful side

            // --- side decision ---
            double forgetting  = 0.98;          ///< Per-frame forgetting factor on the accumulated LLR
            double llr_clamp   = 40.0;          ///< Clamp on the accumulated LLR [nats]
            double delta_clamp = 6.0;           ///< Clamp on the per-frame LLR increment [nats]: one frame of
                                                ///< aliased matches must not swing the decision by itself
            double max_yaw_rate = 0.6;          ///< Don't accumulate LLR while turning faster than this [rad/s]:
                                                ///< motion blur and fresh viewpoints starve the true side of matches
            double flip_threshold      = 25.0;  ///< Request a flip when the LLR is below the negative of this [nats]
            int flip_consecutive       = 10;    ///< ... for at least this many consecutive scored frames
            std::size_t min_flip_assoc = 3;     ///< ... each backed by at least this many mirror-side associations
                                             ///< (a sparse map must not flip on one or two symmetric-structure matches)
            double flip_dominance = 2.0;  ///< ... and the mirror associations must outnumber own by this factor:
                                          ///< a degraded own pose weakening its own matches is "no decision",
                                          ///< not evidence for the other side
            double flip_coverage = 0.5;   ///< ... and the own hypothesis must have at least this fraction of the
                                          ///< mirror's predicted-visible landmarks: when own looks at unmapped
                                          ///< territory the comparison is structurally unfair, so no decision

            // --- blind-own flip escape ---
            // When the wrong-side pose stares at territory the map never covered,
            // the coverage condition above can never become fair and the fair path
            // never fires. The escape accepts much deeper, longer evidence instead:
            // the LLR must sit near its clamp while the own pose is essentially
            // blind (at most one, likely aliased, association) and the mirror keeps
            // matching real structure. On the recorded false-flip episode (own
            // merely degraded, not mirrored) own retains associations on most
            // frames, so the blind streak leaks and never accumulates (max 7 vs
            // threshold 40); on a genuine mirror lock it reaches 40 within ~2 s.
            double flip_blind_llr = 35.0;          ///< Blind path needs the LLR at or below the negative of this [nats]
            std::size_t flip_blind_own_max   = 1;  ///< ... with at most this many own-side associations
            std::size_t flip_blind_min_assoc = 2;  ///< ... and at least this many mirror-side associations
            int flip_blind_consecutive       = 40;  ///< ... over this many (leaky) scored frames

            // The escape's premise is that a pose staring at unmapped territory while
            // the mirror matches must BE the mirror. That is only sound if the robot
            // has not simply turned to face somewhere it never mapped. Against a
            // background that is itself 180 deg symmetric -- the webots stadium, both
            // ends alike -- turning around in place and being mirrored produce the
            // SAME observation, exactly, and the escape resolves the tie towards
            // "mirrored" by construction. It did so on data4_webots at t = 65.8 s
            // after the robot physically turned 164.6 deg, flipping a pose that was
            // correct to 2 cm and costing the rest of the run.
            //
            // The escape therefore requires that the own hypothesis PREDICTS something
            // and fails to match it. A hypothesis that predicts nothing cannot be
            // contradicted by anything: vis_own == 0 says the map does not cover where
            // own is looking, which is what happens every time the robot turns to face
            // new scenery, and says nothing whatever about which side it is on.
            // Scoring it as mirror evidence is the reasoning error.
            //
            // A net-180 deg turn gate is kept as a second line: after a turn the map
            // may still predict a handful of landmarks near the edge of view, and a
            // turn of about half a revolution is precisely the motion that makes a
            // symmetric background look mirrored. The odometry knows about that turn
            // and a flip asserts a 180 deg jump contradicting it, so "no decision" is
            // the honest answer. A kidnapped robot is carried, not turned under its own
            // gyroscope, so a genuine displacement still gets through; the fair path
            // above is untouched either way.
            std::size_t flip_blind_min_visible_own = 1;  ///< Own must predict at least this many landmarks in view
            double blind_turn_tolerance = 60.0 * M_PI / 180.0;  ///< Refuse the escape within this of a 180 deg net
                                                                ///< turn [rad]
            std::size_t blind_match_min_assoc = 10;   ///< Own counts as confirmed at this many associations ...
            double blind_match_fraction       = 0.1;  ///< ... and at least this fraction of its predicted-visible

            double flip_cooldown = 5.0;  ///< Freeze map building for this long after a flip [s], so the
                                         ///< estimator can re-converge before new observations are trusted
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
            int last_status  = 0;    ///< LandmarkStatus this frame (travels with the landmark, so it
                                     ///< stays correct across map compaction; for the 3D view)

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
            double side_delta               = 0.0;  ///< Clamped, turn-gated own-minus-mirror increment this frame
                                                    ///< [nats]; 0 on an unscored frame. This is the per-frame
                                                    ///< evidence to fold into a hypothesis bank (SystemLocalisation::
            ///< add_side_log_evidence) -- the same quantity that drives the LLR.
            double llr              = 0.0;    ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
            bool map_frozen         = false;  ///< Map building was frozen this frame
            bool flip_requested     = false;  ///< The evidence says the filter is on the wrong side
            double turn_since_match = 0.0;    ///< Net heading change since the own side was last confirmed [rad]
            bool blind_turn_blocked = false;  ///< The blind escape was refused because that turn explains
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
                            const Pose<double>& Tfc,
                            const Pose<double>& TfcMirror,
                            double pos_std,
                            double yaw_std,
                            double yaw_rate_abs,
                            double heading);

        /**
         * @brief Notify that the filter state was mirrored (flip applied or kidnap injected).
         *
         * Negates the accumulated LLR (evidence for the old "own" side is evidence
         * against the new one), resets the flip streak and freezes map building for
         * the cooldown period.
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
            bool ambiguous;               ///< Predicted bearing too smeared to discriminate (max_tangent_sigma):
                                          ///< excluded from matching, from scoring and from the miss count
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
         * @param feature_outlier Output (per feature): the corner was gated to at least one landmark
         *                       but ended up unassociated -- rejected as clutter or beaten in the
         *                       one-to-one assignment
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
         * @brief Fit a bearing-only landmark from an observation window.
         *
         * Places the landmark at the assumed range along the mean bearing from the
         * mean camera position, with a large radial variance standing in for the
         * unknown depth. Fails if the bearings are not directionally tight (jittery
         * track: dynamic object or association hops).
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
        bool doubt_                  = false;  ///< Latched after deep doubt; cleared only by positive evidence
        double heading_at_own_match_ = 0.0;    ///< Heading when the map last confirmed the own side [rad]
        bool have_own_match_         = false;  ///< Whether heading_at_own_match_ has been set
        double map_freeze_until_     = -std::numeric_limits<double>::infinity();  ///< Post-flip map-building freeze [s]
        Stats stats_;                                                             ///< Map-building funnel diagnostics
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_SIDEDISAMBIGUATOR_HPP
