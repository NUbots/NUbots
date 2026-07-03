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
#ifndef MODULE_LOCALISATION_FIELDLOCALSATIONNLOPT_HPP
#define MODULE_LOCALISATION_FIELDLOCALSATIONNLOPT_HPP

#include <Eigen/Core>
#include <nlopt.hpp>
#include <optional>
#include <nuclear>

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldIntersections.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/Goal.hpp"

#include "localisation/PoseFilter.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"
#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using message::platform::RawSensors;
    using message::support::FieldDescription;
    using message::vision::FieldIntersection;
    using message::vision::FieldIntersections;
    using message::vision::FieldLines;
    using message::vision::Goals;

    using utility::localisation::Landmark;
    using utility::localisation::OccupancyMap;

    inline const std::map<std::string, nlopt::algorithm> nlopt_algorithm_map = {{"LN_COBYLA", nlopt::LN_COBYLA},
                                                                                {"LN_BOBYQA", nlopt::LN_BOBYQA},
                                                                                {"LN_NEWUOA", nlopt::LN_NEWUOA},
                                                                                {"LN_NELDERMEAD", nlopt::LN_NELDERMEAD},
                                                                                {"LN_SBPLX", nlopt::LN_SBPLX}};

    struct StartingSide {
        enum Value { UNKNOWN = 0, LEFT = 1, RIGHT = 2, EITHER = 3, CUSTOM = 4 };
        Value value = Value::UNKNOWN;

        // Constructors
        StartingSide() = default;
        StartingSide(int const& v) : value(static_cast<Value>(v)) {}
        StartingSide(Value const& v) : value(v) {}
        StartingSide(std::string const& str) {
            // clang-format off
            if      (str == "LEFT") { value = Value::LEFT; }
            else if (str == "RIGHT") { value = Value::RIGHT; }
            else if (str == "EITHER")  { value = Value::EITHER; }
            else if (str == "CUSTOM") { value = Value::CUSTOM; }
            else {
                value = Value::UNKNOWN;
                throw std::runtime_error("String " + str + " did not match any enum for StartingSide");
            }
            // clang-format on
        }

        // Conversions
        [[nodiscard]] operator Value() const {
            return value;
        }
        [[nodiscard]] operator std::string() const {
            switch (value) {
                case Value::LEFT: return "LEFT";
                case Value::RIGHT: return "RIGHT";
                case Value::EITHER: return "EITHER";
                case Value::CUSTOM: return "CUSTOM";
                default: throw std::runtime_error("enum Method's value is corrupt, unknown value stored");
            }
        }
    };

    /**
     * @brief Converts an Eigen vector to an std::vector.
     * @param eigen_vec The Eigen vector to convert.
     * @param nlopt_vec The resulting std::vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void eigen_to_nlopt(const Eigen::Matrix<Scalar, n, 1>& eigen_vec, std::vector<Scalar>& nlopt_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        Eigen::Map<Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the Eigen vector to the std::vector view
        nlopt_eigen_vec = eigen_vec;
    }

    /**
     * @brief Converts an std::vector to an Eigen vector.
     * @param nlopt_vec The std::vector to convert.
     * @param eigen_vec The resulting Eigen vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void nlopt_to_eigen(const std::vector<Scalar>& nlopt_vec, Eigen::Matrix<Scalar, n, 1>& eigen_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        const Eigen::Map<const Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the std::vector view to the Eigen vector
        eigen_vec = nlopt_eigen_vec;
    }

    /**
     * @brief Type definition for an objective function that takes an Eigen vector as input and returns a scalar value.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     */
    template <typename Scalar, int nv>
    using ObjectiveFunction =
        std::function<Scalar(const Eigen::Matrix<Scalar, nv, 1>&, Eigen::Matrix<Scalar, nv, 1>&, void*)>;

    /**
     * @brief Wrapper function that converts input and output between NLopt and Eigen formats for an objective function.
     * @param n The size of the input vector.
     * @param x The input vector in NLopt format.
     * @param grad The gradient vector in NLopt format.
     * @param data Pointer to additional data that is passed to the objective function.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     * @return The scalar value of the objective function.
     */
    template <typename Scalar, int nv>
    inline Scalar eigen_objective_wrapper(unsigned n, const Scalar* x, Scalar* grad, void* data) {
        ObjectiveFunction<Scalar, nv>& obj_fun = *static_cast<ObjectiveFunction<Scalar, nv>*>(data);

        // Convert input from NLopt format to Eigen format
        Eigen::Map<const Eigen::Matrix<Scalar, nv, 1>> eigen_x(x, n);
        Eigen::Matrix<Scalar, nv, 1> eigen_grad = Eigen::Matrix<Scalar, nv, 1>::Zero();

        if (grad) {
            eigen_grad.resize(n);
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        // Call the actual objective function implemented with Eigen
        Scalar result = obj_fun(eigen_x, eigen_grad, data);

        if (grad) {
            // Copy the gradient back to NLopt format
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        return result;
    }

    struct GoalPost {
        Eigen::Vector3d left;
        Eigen::Vector3d right;
    };

    /**
     * @brief Compute the rigid 2D transform (x, y, theta) that maps an observed world-space point pair
     * (p1w, p2w) onto a model field-space point pair (q1f, q2f). Pure geometry, used to turn a matched pair
     * of landmarks/observations directly into a candidate pose without any search.
     * @param p1w First observed point in world space (x, y)
     * @param p2w Second observed point in world space (x, y)
     * @param q1f Field-space model point corresponding to p1w
     * @param q2f Field-space model point corresponding to p2w
     * @return The candidate state (x, y, theta) of Hfw
     */
    Eigen::Vector3d pose_from_point_pair(const Eigen::Vector2d& p1w,
                                         const Eigen::Vector2d& p2w,
                                         const Eigen::Vector2d& q1f,
                                         const Eigen::Vector2d& q2f);

    /**
     * @brief Mirror a state across the field's central symmetry: (x,y,theta) -> (-x,-y,theta+pi). The field
     * localisation cost function is exactly invariant under this transform (see README), so mirrored states
     * are indistinguishable from vision alone and must be disambiguated structurally.
     * @param s The state to mirror
     * @return The mirrored state
     */
    Eigen::Vector3d mirror_pose(const Eigen::Vector3d& s);

    class FieldLocalisationNLopt : public NUClear::Reactor {
    private:
        // Define the model dimensions
        static constexpr size_t n_states = 3;

        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            double grid_size = 0.0;

            /// @brief Initial hypothesis of the robot's state (x,y,theta), saved for resetting
            std::vector<Eigen::Vector3d> initial_hypotheses{};

            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            Eigen::Vector3d initial_state{};

            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;

            /// @brief Minimum number of field line points for an update to run
            size_t min_field_line_points = 0;

            /// @brief Minimum number of field line intersections for an update to run
            size_t min_field_line_intersections = 0;

            /// @brief Start time delay for the particle filter
            std::chrono::seconds start_time_delay = std::chrono::seconds(0);

            /// @brief Bool to enable/disable using ground truth for localisation
            bool use_ground_truth_localisation;

            /// @brief Bool to enable the use of Hungarian algorithm for landmark association
            bool use_hungarian = false;

            /// @brief Starting side of the field (LEFT, RIGHT, EITHER, or CUSTOM)
            StartingSide starting_side = StartingSide::UNKNOWN;

            /// @brief Scalar weighting of cost associated with distance to field lines
            double field_line_distance_weight = 0.0;

            /// @brief Scalar weighting of cost associated with distance to field intersections
            double field_line_intersection_weight = 0.0;

            /// @brief Scalar weighting of cost associated with change in state
            double state_change_weight = 0.0;

            /// @brief Scalar weighting of cost associated with distance to goal posts
            double goal_post_distance_weight = 0.0;

            /// @brief Constraint on the maximum change in state
            Eigen::Vector3d change_limit = Eigen::Vector3d::Zero();

            /// @brief Normal optimisation - Relative tolerance on the optimisation parameters
            double normal_xtol_rel = 0.0;

            /// @brief Normal optimisation - Relative tolerance on the optimisation function value
            double normal_ftol_rel = 0.0;

            /// @brief Normal optimisation - Maximum number of evaluations for the optimisation
            size_t normal_maxeval = 0;

            /// @brief Normal optimisation - Algorithm to use
            nlopt::algorithm normal_algorithm = nlopt::LN_COBYLA;

            /// @brief Uncertainty optimisation - Relative tolerance on the optimisation parameters
            double uncertainty_xtol_rel = 0.0;

            /// @brief Uncertainty optimisation - Relative tolerance on the optimisation function value
            double uncertainty_ftol_rel = 0.0;

            /// @brief Uncertainty optimisation - Maximum number of evaluations for the optimisation
            size_t uncertainty_maxeval = 100;

            /// @brief Algorithm to use when localisation is uncertain
            nlopt::algorithm uncertainty_algorithm = nlopt::LN_SBPLX;

            /// @brief Goal error tolerance [m]
            double goal_post_error_tolerance = 0.0;

            /// @brief Maximum distance for landmark association
            double max_association_distance = 0.0;

            /// @brief Cost for a point being outside of the field
            double out_of_field_cost = 0.0;

            /// @brief Reset delay in seconds
            int reset_delay = 0;
            /// @brief Step size for the grid search during uncertainty reset
            double step_size = 0.0;
            /// @brief The window size for the local search during uncertainty reset
            double window_size = 0.0;
            /// @brief Number of yaw angles to try during uncertainty reset
            int num_angles = 0;

            /// @brief Process noise variance rate (x^2, y^2, theta^2) per unit odometry motion [m + rad] applied
            /// to the pose filter's covariance each frame, scaled by observed motion
            Eigen::Vector3d kalman_process_noise = Eigen::Vector3d::Zero();
            /// @brief Scalar mapping inverse Hessian curvature to measurement variance
            double kalman_measurement_scale = 1.0;
            /// @brief Finite-difference step size per axis for the Hessian [m, m, rad], clamped to
            /// >= 2*grid_size at use time to ride over grid quantisation and optimiser inexactness
            Eigen::Vector3d kalman_hessian_step = Eigen::Vector3d::Zero();

            /// @brief Field-line point registered as an inlier below this map distance [m]
            double validity_line_inlier_distance = 0.0;
            /// @brief Minimum matched-percept validity fraction to accept an optimisation result
            double validity_min_validity = 0.0;
            /// @brief Maximum number of consecutive invalid frames before triggering an uncertainty reset
            int validity_max_invalid_frames = 0;
            /// @brief Whether to trigger an uncertainty reset after validity_max_invalid_frames consecutive
            /// invalid frames
            bool validity_reset_on_invalid = false;

            /// @brief Use analytic goal/intersection-pair candidates before falling back to grid search
            bool reset_use_candidates = true;
            /// @brief Tolerance [m] on pair separation when matching observed intersection pairs to landmark
            /// pairs
            double reset_pair_separation_tolerance = 0.0;
            /// @brief Maximum number of reset candidates kept after deduplication
            int reset_max_candidates = 0;
            /// @brief Constrain resets/searches to the robot's current half of the field (avoids the mirror
            /// field problem)
            bool reset_constrain_to_half = true;

            /// @brief Act automatically on detected mirror-flip cues (e.g. PenaltyReset) rather than only
            /// logging a warning
            bool mirror_auto_flip = false;

            /// @brief Continuously probe reset candidates to escape local minima of the cost function
            bool recovery_enabled = true;
            /// @brief Number of frames of validity averaged per local-minimum escape probe window; the probe
            /// runs once per window
            int recovery_probe_period = 30;
            /// @brief A probe candidate must beat the window's mean validity by this margin for the escape to
            /// be accepted. This margin is the sole jump decision (together with the window_size locality
            /// bound); when well-localised the best candidate matches the current pose and no jump occurs.
            double recovery_improvement_margin = 0.1;
        } cfg;

        /// @brief State vector (x,y,yaw) of the Hfw transform, the raw (unfiltered) optimisation result. Kept
        /// separately from the filter's mean for debugging/graphing purposes.
        Eigen::Vector3d state = Eigen::Vector3d::Zero();

        /// @brief Probabilistic pose filter over (x,y,theta) of Hfw. The optimisation result is treated as a
        /// measurement with covariance derived from the cost function's curvature (Hessian) at the optimum.
        PoseFilter filter{};

        /// @brief The last robot-to-world transform (Sensors::Hrw), used to compute the odometry motion delta
        /// consumed by the filter's predict step
        Eigen::Isometry3d last_Hrw = Eigen::Isometry3d::Identity();

        /// @brief Whether last_Hrw currently holds a valid previous value (false immediately after a hard
        /// reset, since the motion delta across a reset is not meaningful)
        bool last_Hrw_valid = false;

        /// @brief Number of consecutive frames rejected by the validity gate
        int num_invalid_frames = 0;

        /// @brief Running sum of per-frame validity over the current local-minimum escape probe window
        double recovery_validity_sum = 0.0;

        /// @brief Number of frames accumulated in the current local-minimum escape probe window
        int recovery_frame_count = 0;

        /// @brief Field line distance map (encodes the minimum distance to a field line)
        OccupancyMap<double> fieldline_distance_map{};

        /// @brief List of landmarks (field intersections rLFf) in field space
        std::vector<Landmark> landmarks{};

        /// @brief Goal posts
        GoalPost own_goal_posts;
        GoalPost opp_goal_posts;

        /// @brief Expected goal post width
        double expected_goal_post_distance = 0.0;

        /// @brief Bool indicating where or not this is the first update
        bool startup = true;

        /// @brief Bool indicating ground truth localisation (Hfw) computed
        bool ground_truth_initialised = false;

        /// @brief Ground truth Hfw
        Eigen::Isometry3d ground_truth_Hfw = Eigen::Isometry3d::Identity();

    public:
        /// @brief Called by the powerplant to build and setup the FieldLocalisationNLopt reactor.
        explicit FieldLocalisationNLopt(std::unique_ptr<NUClear::Environment> environment);
        /// @brief The main field localisation loop
        ReactionHandle main_loop;

        /// @brief The last time the field localisation was reset
        NUClear::clock::time_point last_reset = NUClear::clock::now();

        /// @brief The last certain state of the robot (used for uncertainty reset)
        Eigen::Vector3d last_certain_state = Eigen::Vector3d::Zero();

        /**
         * @brief Compute Hfw, homogenous transformation from world {w} to field {f} space from state vector (x,y,theta)
         * @param state The state vector (x,y,theta)
         * @return Hfw, the homogenous transformation matrix from world {w} to field {f} space
         */
        Eigen::Isometry3d compute_Hfw(const Eigen::Vector3d& state);

        /**
         * @brief Find error between computed Hfw and ground truth if available
         * @param Hfw Computed Hfw to be compared against ground truth
         */
        void debug_field_localisation(Eigen::Isometry3d Hfw);

        /**
         * @brief Transform a field line point from world {w} to position in the distance map {m}
         * @param particle The state of the particle (x,y,theta)
         * @param rPWw The field point (x, y) in world space {w} [m]
         * @return Eigen::Vector2i
         */
        Eigen::Vector2i position_in_map(const Eigen::Vector3d& particle, const Eigen::Vector3d& rPWw);

        /**
         * @brief Evaluate the perception-only cost (field-line, field-line intersection and goal-post terms)
         * of a hypothesised state. Deliberately excludes the state-change regulariser: this function is
         * shared by the optimiser objective, the finite-difference Hessian and the validity metric, none of
         * which should see the artificial curvature the regulariser would inject in otherwise-unobservable
         * directions.
         * @param x The hypothesised state (x,y,theta)
         * @param field_lines The observations of the field line points
         * @param field_intersections The field intersections
         * @param goals The observed goals
         * @return The perception-only cost of the hypothesis
         */
        double evaluate_cost(const Eigen::Vector3d& x,
                             const std::vector<Eigen::Vector3d>& field_lines,
                             const std::shared_ptr<const FieldIntersections>& field_intersections,
                             const std::shared_ptr<const Goals>& goals);

        /**
         * @brief Run the field line optimisation
         * @param initial_guess The initial guess for the field line
         * @param observations The observations of the field line points
         * @param field_intersections The field intersections
         * @param goals The observed goals
         * @param uncertainty_optimisation Whether to use the (faster, less accurate) uncertainty optimisation
         * settings
         * @param box_bounds Optional per-axis box half-width around initial_guess to bound the optimisation.
         * If zero (default), falls back to cfg.change_limit.
         * @return Pair <optimisation solution (x,y,theta), final cost>
         */
        std::pair<Eigen::Vector3d, double> run_field_line_optimisation(
            const Eigen::Vector3d& initial_guess,
            const std::vector<Eigen::Vector3d>& field_lines,
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const std::shared_ptr<const Goals>& goals,
            bool uncertainty_optimisation                = false,
            const Eigen::Vector3d& box_bounds             = Eigen::Vector3d::Zero());

        /**
         * @brief Compute a validity metric in [0, 1] for a hypothesised state: the fraction of matched
         * percepts (field-line points registered within validity.line_inlier_distance of the field-line map,
         * blended 50/50 with the fraction of field intersections successfully associated with a landmark
         * within max_association_distance). Used to gate acceptance instead of raw optimisation cost, since
         * cost magnitudes are not directly comparable across differing numbers/mixes of observations.
         * @param state The hypothesised state (x,y,theta)
         * @param field_lines The observations of the field line points
         * @param field_intersections The field intersections
         * @return The validity of the hypothesis in [0, 1]
         */
        double compute_validity(const Eigen::Vector3d& state,
                                const std::vector<Eigen::Vector3d>& field_lines,
                                const std::shared_ptr<const FieldIntersections>& field_intersections);

        /**
         * @brief Central-difference Hessian of evaluate_cost at x (perception terms only, no state-change
         * regulariser). Used to derive a measurement covariance for the pose filter update.
         * @param x The point at which to evaluate the Hessian
         * @param field_lines The observations of the field line points
         * @param field_intersections The field intersections
         * @param goals The observed goals
         * @param h Per-axis finite-difference step size
         * @return The 3x3 Hessian matrix of evaluate_cost at x
         */
        Eigen::Matrix3d finite_difference_hessian(const Eigen::Vector3d& x,
                                                  const std::vector<Eigen::Vector3d>& field_lines,
                                                  const std::shared_ptr<const FieldIntersections>& field_intersections,
                                                  const std::shared_ptr<const Goals>& goals,
                                                  const Eigen::Vector3d& h);

        /**
         * @brief Map a cost-function Hessian to a measurement covariance. Eigen-decomposes H and maps each
         * eigenvalue lambda_i to a variance clamp(scale/lambda_i, r_min, r_max); non-positive or
         * near-singular eigenvalues (unobserved/unreliable directions) are clamped to r_max rather than
         * triggering a wholesale fallback. Pure/static: needs no reactor state, so it is directly unit
         * testable.
         * @param H The Hessian matrix (assumed symmetric)
         * @param scale Scalar mapping inverse curvature to variance
         * @param r_min Minimum variance per eigen-direction
         * @param r_max Maximum variance per eigen-direction (also used for non-positive/tiny eigenvalues)
         * @return The 3x3 measurement covariance matrix
         */
        static Eigen::Matrix3d covariance_from_hessian(const Eigen::Matrix3d& H,
                                                       double scale,
                                                       double r_min,
                                                       double r_max);

        /**
         * @brief Generate candidate poses from the two observed goal posts, tried against both assignment
         * orders and both field ends, gated by the expected goal post distance +- tolerance.
         * @param goals The observed goals
         * @return Up to 4 candidate states (x,y,theta)
         */
        std::vector<Eigen::Vector3d> goal_pair_candidates(const std::shared_ptr<const Goals>& goals);

        /**
         * @brief Generate candidate poses from pairs of observed field intersections matched against pairs of
         * model landmarks of matching type and similar separation. Mirror twins arise naturally from the
         * symmetric landmark layout. Deduplicated and capped at cfg.reset_max_candidates, preferring
         * widely-separated pairs (better angular conditioning).
         * @param field_intersections The field intersections
         * @return The candidate states (x,y,theta)
         */
        std::vector<Eigen::Vector3d> intersection_pair_candidates(
            const std::shared_ptr<const FieldIntersections>& field_intersections);

        /**
         * @brief Perform data association between intersection observations and landmarks using nearest neighbour
         * @param field_intersections The field intersections
         * @param Hfw The homogenous transformation matrix from world {w} to field {f} space
         * @return Pairs of landmarks and corresponding field intersections (known landmark, intersection
         */
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> data_association(
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const Eigen::Isometry3d& Hfw);

        /**
         * @brief Determines a new state by running a grid search and checking the cost of each hypothesis.
         * First, a local search is performed around the last certain state. If this does not find a low cost
         * hypothesis, a global search is performed on the half of the field that the robot is currently on, to reduce
         * computation and avoid the mirror field problem.
         *
         * @param fd The field description containing the field dimensions, in particular the field length and width.
         * @param field_lines Field lines, used to find the cost of hypotheses.
         * @param field_intersections Field intersections, used to find the cost of hypotheses.
         * @param goals Goals, used to find the cost of hypotheses.
         * @param Hrw The homogenous transformation from world {w} to robot {r} space.
         */
        void uncertainty_reset(const FieldDescription& fd,
                               const FieldLines& field_lines,
                               const std::shared_ptr<const FieldIntersections>& field_intersections,
                               const std::shared_ptr<const Goals>& goals,
                               const Eigen::Isometry3d& Hrw);

        /**
         * @brief Generate analytic pose candidates (goal pairs, intersection pairs, last certain state and
         * mirrors when not constrained to a half), refine each with the uncertainty optimiser, and return the
         * candidate with the highest validity. Shared by uncertainty_reset (stage 1-2) and the automatic
         * local-minimum escape.
         *
         * @param field_lines Field lines, used to refine and score candidates.
         * @param field_intersections Field intersections, used to generate and score candidates.
         * @param goals Goals, used to generate candidates.
         * @return Best (refined state, validity) pair, or nullopt if no candidates could be generated.
         */
        std::optional<std::pair<Eigen::Vector3d, double>> candidate_probe(
            const FieldLines& field_lines,
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const std::shared_ptr<const Goals>& goals);

        /**
         * @brief Attempt to escape a local minimum of the cost function by probing the analytic reset
         * candidates. The optimiser is local (seeded from the filter mean each frame), so a biased pose that
         * still explains most percepts (validity above the accept threshold but mediocre) is self-reinforcing
         * and never triggers the invalid-frames reset. The probe candidates are global and association-free,
         * so they can land in the true basin. The escape is accepted only if the best candidate beats the
         * probe window's mean validity by cfg.recovery_improvement_margin AND is within cfg.window_size
         * (position + wrapped angle) of the current estimate - the latter keeps escapes local and, in
         * particular, rejects mirror flips (a mirror twin is always at least pi away in heading).
         *
         * @param field_lines Field lines, used to refine and score candidates.
         * @param field_intersections Field intersections, used to generate and score candidates.
         * @param goals Goals, used to generate candidates.
         * @param mean_validity Mean validity over the probe window, the baseline the candidate must beat.
         * @return Whether an escape was performed (the filter has been reset to the new pose).
         */
        bool try_local_minimum_escape(const FieldLines& field_lines,
                                      const std::shared_ptr<const FieldIntersections>& field_intersections,
                                      const std::shared_ptr<const Goals>& goals,
                                      double mean_validity);


        /**
         * @brief Perform data association between intersection observations and landmarks using the Hungarian algorithm
         *
         * @param field_intersections The field intersections
         * @param Hfw The homogenous transformation matrix from world {w} to field {f} space
         * @return std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> The associated pairs of field intersections
         * and landmarks
         */
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> hungarian_association(
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const Eigen::Isometry3d& Hfw);

        /**
         * @brief Perform data association between intersection observations and landmarks using nearest neighbour
         *
         * @param field_intersections The field intersections
         * @param Hfw The homogenous transformation matrix from world {w} to field {f} space
         * @return std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> The associated pairs of field intersections
         * and landmarks
         */
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> greedy_association(
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const Eigen::Isometry3d& Hfw);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALSATIONNLOPT_HPP
