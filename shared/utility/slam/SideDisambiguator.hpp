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
#ifndef SIDEDISAMBIGUATOR_HPP
#define SIDEDISAMBIGUATOR_HPP

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/core.hpp>
#include <vector>

#include "FieldMap.hpp"
#include "OutOfFieldFeatures.hpp"
#include "camera/FisheyeLens.hpp"
#include "camera/Pose.hpp"

namespace utility::slam {

    using utility::slam::camera::FisheyeLens;
    using utility::slam::camera::Pose;

    class SideDisambiguator {
    public:
        /**
         * @brief Tuning options.
         */
        struct Options {
            // --- SNN association ---
            double sigmaAngular =
                0.05;  ///< Corner bearing noise std dev [rad] (~3 deg: pixel noise + unmodelled pose drift)
            double clutterDensity = 5.0;       ///< Effective clutter density [features/steradian] (physical is ~55;
                                               ///< lower widens acceptance so the true side survives estimator drift)
            double posStdFloor = 0.6;          ///< Floor on the camera position std used for gating [m]: the filter
                                               ///< is known to be optimistic, and a too-tight gate starves the TRUE
                                               ///< side of matches whenever the estimate drifts (both sides get the
                                               ///< same widening, so the comparison stays fair)
            int maxDescriptorDistance = 64;    ///< ORB Hamming gate for an association pair
            double preGateAngle       = 0.20;  ///< Cheap dot-product pre-gate before surprisal evaluation [rad]
            int visibleMargin         = 30;    ///< Landmark counts as predicted-visible this far inside the image [px]
            double maxTangentSigma    = 0.16;  ///< Skip landmarks whose predicted bearing sigma exceeds this [rad]:
                                               ///< a bearing-only landmark viewed far from its anchor spans an
                                               ///< acceptance corridor that aliases; it carries no side information
                                               ///< at this baseline and must neither match nor count as missed

            // --- candidate spawning and promotion ---
            double candGateAngle =
                0.10;  ///< Candidate re-match gate on ray angle in {f} [rad] (rotation cancels; covers pose jitter)
            int candMaxDescriptorDistance = 50;   ///< Stricter Hamming gate for growing a candidate track
            int minObs                    = 4;    ///< Observations needed to attempt promotion
            double minTimeSpan            = 1.0;  ///< Track must span at least this long [s]
            double minParallax =
                0.10;  ///< Max pairwise ray angle must exceed this [rad] (~6 deg; keeps depth above pose noise)

            // --- far (bearing-only) landmarks ---
            // Most background structure never accrues reliable parallax (pose jitter
            // is the same order as the true parallax), yet its bearing alone fully
            // discriminates the 180 deg mirror. Directionally tight tracks therefore
            // promote as bearing-only landmarks at an assumed range with a large
            // radial variance; the SNN tangent-plane covariance handles the missing
            // depth, and later parallax upgrades them to proper points.
            int farPromoteObs         = 6;     ///< Observations needed for a bearing-only promotion
            double farPromoteTimeSpan = 2.0;   ///< ... spanning at least this long [s]
            double farMaxSpread       = 0.08;  ///< RMS angle about the mean bearing must stay below this [rad]
            double assumedRange       = 6.0;   ///< Nominal range of a bearing-only landmark [m]
            double candMaxAge         = 3.0;   ///< Drop candidates unmatched for this long [s]
            std::size_t maxCandidates = 600;   ///< Candidate pool cap (oldest dropped first)
            double maxElevation =
                50.0 * M_PI / 180.0;  ///< Don't track features above this elevation [rad]:
                                      ///< overhead lighting grids are near-symmetric under the field's
                                      ///< 180 deg rotation and would feed the mirror hypothesis

            // --- landmark maintenance / outlier (dynamic object) rejection ---
            double sigmaStatic =
                0.04;  ///< Angular residual scale for the static-consistency test [rad] (incl. pose drift)
            double staticChi2Mean    = 9.0;   ///< Reject when the mean normalised squared residual exceeds this
            int obsWindow            = 12;    ///< Bounded per-landmark observation window (re-triangulated each hit)
            int maxMissStreak        = 20;    ///< Prune after this many predicted-visible frames without a hit
            std::size_t maxLandmarks = 500;   ///< Map cap (lowest-scoring evicted first, see evictHalfLife)
            double evictHalfLife     = 20.0;  ///< Staleness that halves a landmark's cap score [s].
                                              ///< The map sits pinned at maxLandmarks for most of a run, so the
                                              ///< cap -- not the miss rule -- is the dominant eviction path, and
                                              ///< ranking on lifetime hits alone let a landmark that earned its
                                              ///< hits early outrank a fresh one that is matching NOW. Scoring
                                              ///< hits/(1 + age/halfLife) keeps established landmarks (50 hits
                                              ///< unseen 100 s still beats 5 hits seen now) while letting truly
                                              ///< dead ones fall out. Not pure recency on purpose: a landmark
                                              ///< behind the robot is not failing, and the mirror test needs
                                              ///< coverage in every direction, so staleness must decay influence
                                              ///< rather than decide eviction outright.
            double minRange          = 0.8;   ///< Reject triangulations closer than this to the camera [m]
            double maxRange          = 40.0;  ///< Reject triangulations farther than this [m]
            double minHeightOnCarpet = 1.5;  ///< Points above the carpet in xy must be at least this high [m] (ceiling)
            double fieldMargin       = 0.30;  ///< Carpet margin matching the detector's classification [m]

            // --- map building gate ---
            double maxPosStd  = 0.5;   ///< Build the map only when horizontal position std is below this [m]
            double freezeLlr  = -1.0;  ///< Freeze map building when the accumulated LLR drops below this
            double rebuildLlr = 5.0;   ///< After deep doubt (LLR past the flip threshold), building stays
                                       ///< frozen until the LLR earns its way back above this: forgetting
                                       ///< decays the LLR towards zero on its own, and decay is not
                                       ///< evidence, so it must never re-arm building on a doubtful side

            // --- side decision ---
            double forgetting = 0.98;         ///< Per-frame forgetting factor on the accumulated LLR
            double llrClamp   = 40.0;         ///< Clamp on the accumulated LLR [nats]
            double deltaClamp = 6.0;          ///< Clamp on the per-frame LLR increment [nats]: one frame of
                                              ///< aliased matches must not swing the decision by itself
            double maxYawRate = 0.6;          ///< Don't accumulate LLR while turning faster than this [rad/s]:
                                              ///< motion blur and fresh viewpoints starve the true side of matches
            double flipThreshold     = 25.0;  ///< Request a flip when the LLR is below the negative of this [nats]
            int flipConsecutive      = 10;    ///< ... for at least this many consecutive scored frames
            std::size_t minFlipAssoc = 3;     ///< ... each backed by at least this many mirror-side associations
                                           ///< (a sparse map must not flip on one or two symmetric-structure matches)
            double flipDominance = 2.0;  ///< ... and the mirror associations must outnumber own by this factor:
                                         ///< a degraded own pose weakening its own matches is "no decision",
                                         ///< not evidence for the other side
            double flipCoverage = 0.5;   ///< ... and the own hypothesis must have at least this fraction of the
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
            double flipBlindLlr           = 35.0;  ///< Blind path needs the LLR at or below the negative of this [nats]
            std::size_t flipBlindOwnMax   = 1;     ///< ... with at most this many own-side associations
            std::size_t flipBlindMinAssoc = 2;     ///< ... and at least this many mirror-side associations
            int flipBlindConsecutive      = 40;    ///< ... over this many (leaky) scored frames
            double flipCooldown           = 5.0;   ///< Freeze map building for this long after a flip [s], so the
                                                   ///< estimator can re-converge before new observations are trusted
        };

        /**
         * @brief A triangulated out-of-field landmark in the field frame {f}.
         */
        struct Landmark {
            Eigen::Vector3d rPFf;   ///< Triangulated (or assumed-range) position in {f}
            Eigen::Matrix3d P;      ///< Position covariance in {f} (huge radially if bearing-only)
            bool far = false;       ///< Bearing-only landmark (no reliable depth yet)
            cv::Mat descriptor;     ///< Latest matched ORB descriptor (1x32 CV_8U, owned)
            int hits        = 0;    ///< Number of associated observations
            int missStreak  = 0;    ///< Consecutive predicted-visible frames without association
            double lastSeen = 0.0;  ///< Time of the last associated observation [s]
            int lastStatus  = 0;    ///< LandmarkStatus this frame (travels with the landmark, so it
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
            LANDMARK_EDGE,             ///< Projects into the image but within visibleMargin of the border,
                                       ///< so it is not counted as predicted-visible
            LANDMARK_AMBIGUOUS,        ///< Predicted bearing too smeared to discriminate the mirror
                                       ///< (maxTangentSigma): excluded from matching and from scoring
            LANDMARK_MISSED,           ///< Predicted well inside the image, no corner associated with it
            LANDMARK_ASSOCIATED,       ///< Associated to a corner this frame
            LANDMARK_CULLED_MISSING,   ///< Culled this frame: maxMissStreak predicted-visible frames without a hit
            LANDMARK_CULLED_OUTLIER    ///< Culled this frame: its window stopped fitting one static point (mover)
        };

        /// @brief A map landmark projected into the camera image, with its per-frame status.
        struct LandmarkView {
            Eigen::Vector2d px      = Eigen::Vector2d::Zero();  ///< Predicted pixel position at the current pose
            Eigen::Vector2d matchPx = Eigen::Vector2d::Zero();  ///< Pixel of the matched corner (ASSOCIATED only)
            int status              = LANDMARK_EDGE;            ///< LandmarkStatus
            bool far                = false;                    ///< Bearing-only landmark (no reliable depth)
        };

        /**
         * @brief Per-frame result: association counts, side scores and the decision state.
         */
        struct FrameResult {
            std::size_t nFeatures         = 0;        ///< Detected corners
            std::size_t nOutOfField       = 0;        ///< ... classified out-of-field
            std::size_t nAssociated       = 0;        ///< ... associated to map landmarks at the current pose
            std::size_t nAssociatedMirror = 0;        ///< ... associated at the mirrored pose
            std::size_t nOutlier          = 0;        ///< ... gated to a landmark but rejected (under either pose)
            std::size_t nVisibleOwn       = 0;        ///< Landmarks predicted well inside the image at the current pose
            std::size_t nVisibleMirror    = 0;        ///< ... at the mirrored pose
            std::size_t nLandmarks        = 0;        ///< Live map landmarks
            std::size_t nCandidates       = 0;        ///< Live bearing candidates
            double scoreOwn               = 0.0;      ///< Robust side score at the current pose [nats]
            double scoreMirror            = 0.0;      ///< Robust side score at the mirrored pose [nats]
            double llr                    = 0.0;      ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
            bool mapFrozen                = false;    ///< Map building was frozen this frame
            bool flipRequested            = false;    ///< The evidence says the filter is on the wrong side
            std::vector<OutOfFieldFeature> features;  ///< The detected corners (for display)
            std::vector<int> featureStatus;           ///< Per detected corner: a FeatureStatus
            std::vector<LandmarkView> landmarkViews;  ///< Map landmarks projected into the image at the current pose
        };

        /**
         * @brief Construct the disambiguator.
         * @param lens Fisheye lens model (shared with the detector)
         * @param dims Field dimensions (carpet extent)
         * @param options Tuning options
         */
        SideDisambiguator(const FisheyeLens& lens, const FieldDimensions& dims, const Options& options);

        /// @brief Construct with default options.
        SideDisambiguator(const FisheyeLens& lens, const FieldDimensions& dims);

        /**
         * @brief Process one video frame.
         * @param t Frame time [s]
         * @param gray Grayscale camera frame (CV_8UC1, full lens resolution)
         * @param Tfc Estimated camera pose in {f} at the posterior mean
         * @param TfcMirror Camera pose under the 180 degree mirrored state
         * @param posStd Horizontal position std of the filter [m] (map-building gate)
         * @param yawStd Yaw std of the filter [rad] (association gating)
         * @param yawRateAbs Magnitude of the current yaw rate [rad/s] (turn gating)
         * @return Association/score/decision summary for this frame
         */
        FrameResult process(double t,
                            const cv::Mat& gray,
                            const Pose<double>& Tfc,
                            const Pose<double>& TfcMirror,
                            double posStd,
                            double yawStd,
                            double yawRateAbs);

        /**
         * @brief Notify that the filter state was mirrored (flip applied or kidnap injected).
         *
         * Negates the accumulated LLR (evidence for the old "own" side is evidence
         * against the new one), resets the flip streak and freezes map building for
         * the cooldown period.
         *
         * @param t Time the flip was applied [s]
         */
        void notifyFlipApplied(double t);

        const std::vector<Landmark>& landmarks() const {
            return landmarks_;
        }
        double llr() const {
            return llr_;
        }

        /// @brief Cumulative diagnostics of the map-building funnel.
        struct Stats {
            std::size_t promoteAttempts = 0;  ///< Candidate windows that reached the parallax test with enough obs/time
            std::size_t parallaxWait    = 0;  ///< ... still waiting for parallax
            std::size_t triFailGeometry = 0;  ///< ... normal matrix singular (rays too parallel)
            std::size_t triFailRange    = 0;  ///< ... solution behind a ray or out of range
            std::size_t triFailChi2     = 0;  ///< ... inconsistent with one static point (dynamic object)
            std::size_t backgroundFail  = 0;  ///< ... triangulated onto the carpet (dynamic clutter)
            std::size_t promoted        = 0;  ///< ... promoted to point landmarks
            std::size_t promotedFar     = 0;  ///< ... promoted to bearing-only landmarks
            std::size_t farSpreadFail   = 0;  ///< ... bearing-only promotion rejected (jittery track)
            std::size_t upgraded        = 0;  ///< Bearing-only landmarks upgraded to points by later parallax
            std::size_t landmarkCulledChi2 = 0;  ///< Landmarks culled by the static-consistency re-test
            std::size_t landmarkCulledMiss = 0;  ///< Landmarks culled by the miss-streak rule
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
            double lastSeen = 0.0;
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
            bool wellInside;              ///< Predicted at least visibleMargin inside the image border
            bool ambiguous;               ///< Predicted bearing too smeared to discriminate (maxTangentSigma):
                                          ///< excluded from matching, from scoring and from the miss count
            bool associated     = false;  ///< Claimed by a corner in the one-to-one assignment
            std::size_t feature = 0;      ///< The claiming corner (associated only)
        };

        /**
         * @brief SNN association of out-of-field features against the map at a pose.
         * @param features Detected features (only out-of-field ones participate)
         * @param Tfc Camera pose to predict the landmarks under
         * @param posStd Horizontal position std [m] (inflates the predictive covariance)
         * @param yawStd Yaw std [rad] (inflates the predictive covariance)
         * @param score Output: robust side score, sum of log(inlier/clutter) density ratios
         * @param predictions Output: every landmark that projects into the image, with its outcome
         * @param featureOutlier Output (per feature): the corner was gated to at least one landmark
         *                       but ended up unassociated -- rejected as clutter or beaten in the
         *                       one-to-one assignment
         */
        std::vector<Association> associate(const std::vector<OutOfFieldFeature>& features,
                                           const Pose<double>& Tfc,
                                           double posStd,
                                           double yawStd,
                                           double& score,
                                           std::vector<Prediction>& predictions,
                                           std::vector<char>& featureOutlier) const;

        /// @brief Why a triangulation attempt was rejected (or OK).
        enum class TriResult { OK, GEOMETRY, RANGE, CHI2 };

        /**
         * @brief Triangulate a static point from an observation window by linear least squares.
         * @param obs Observations (camera positions and unit rays in {f})
         * @param rPFf Output: triangulated position
         * @param P Output: position covariance (from the ray geometry and sigmaStatic)
         * @param meanChi2 Output: mean normalised squared perpendicular residual
         * @return OK if the window is consistent with one static in-range point, else the failure reason
         */
        TriResult triangulate(const std::vector<Landmark::Obs>& obs,
                              Eigen::Vector3d& rPFf,
                              Eigen::Matrix3d& P,
                              double& meanChi2) const;

        /// @brief True if a triangulated point is plausible background (not on-carpet clutter).
        bool isBackgroundPoint(const Eigen::Vector3d& rPFf) const;

        /**
         * @brief Fit a bearing-only landmark from an observation window.
         *
         * Places the landmark at the assumed range along the mean bearing from the
         * mean camera position, with a large radial variance standing in for the
         * unknown depth. Fails if the bearings are not directionally tight (jittery
         * track: dynamic object or association hops).
         */
        bool fitFar(const std::vector<Landmark::Obs>& obs, Eigen::Vector3d& rPFf, Eigen::Matrix3d& P) const;

        /**
         * @brief Grow candidate tracks with unmatched features; promote mature ones.
         * @param featureGrewTrack Output (per feature): the corner extended an existing candidate track
         */
        void updateCandidates(const std::vector<OutOfFieldFeature>& features,
                              const std::vector<bool>& featureUsed,
                              const Pose<double>& Tfc,
                              double t,
                              std::vector<char>& featureGrewTrack);

        const FisheyeLens& lens_;
        OutOfFieldDetector detector_;
        double halfCarpetLength_;  ///< Field half-length + border strip + margin [m]
        double halfCarpetWidth_;   ///< Field half-width + border strip + margin [m]

        std::vector<Landmark> landmarks_;
        std::vector<Candidate> candidates_;

        double llr_            = 0.0;    ///< Accumulated own-vs-mirror log-likelihood ratio [nats]
        int flipStreak_        = 0;      ///< Consecutive scored frames at/below the flip threshold
        int blindStreak_       = 0;      ///< Leaky streak for the blind-own flip escape
        bool doubt_            = false;  ///< Latched after deep doubt; cleared only by positive evidence
        double mapFreezeUntil_ = -std::numeric_limits<double>::infinity();  ///< Post-flip map-building freeze [s]
        Stats stats_;                                                       ///< Map-building funnel diagnostics
    };
}  // namespace utility::slam

#endif
