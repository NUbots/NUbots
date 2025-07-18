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
#include <nuclear>

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldIntersections.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/Goal.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"
#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/filter/KalmanFilter.hpp"
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

    class FieldLocalisationNLopt : public NUClear::Reactor {
    private:
        // Define the model dimensions
        static constexpr size_t n_states       = 3;
        static constexpr size_t n_inputs       = 0;
        static constexpr size_t n_measurements = 3;

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

            /// @brief Relative tolerance on the optimization parameters
            double xtol_rel = 0.0;

            /// @brief Relative tolerance on the optimization function value
            double ftol_rel = 0.0;

            /// @brief Maximum number of evaluations for the optimization
            size_t maxeval = 0;

            /// @brief Process model
            Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

            /// @brief Input model
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_states, n_inputs);

            /// @brief Measurement model
            Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n_measurements, n_states);

            /// @brief Process noise covariance
            Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();

            /// @brief Measurement noise covariance
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

            /// @brief initial covariance
            Eigen::Matrix<double, n_states, n_states> P0 = Eigen::Matrix<double, n_states, n_states>::Identity();

            /// @brief Goal error tolerance [m]
            double goal_post_error_tolerance = 0.0;

            /// @brief Maximum distance for landmark association
            double max_association_distance = 0.0;

            /// @brief Cost for a point being outside of the field
            double out_of_field_cost = 0.0;

            /// @brief When this is enabled, the field localisation will reset if the cost is too high
            bool reset_on_cost = false;
            /// @brief Cost threshold for resetting the filter
            double cost_threshold = 0.0;
            /// @brief Reset delay in seconds
            int reset_delay = 0;
            /// @brief Maximum number of times the cost can be over the threshold before resetting
            int max_over_cost = 0;
            /// @brief Step size for the grid search during uncertainty reset
            double step_size = 0.0;
            /// @brief The window size for the local search during uncertainty reset
            double window_size = 0.0;
            /// @brief Number of yaw angles to try during uncertainty reset
            int num_angles = 0;
        } cfg;

        // Kalman filter
        utility::math::filter::KalmanFilter<double, n_states, n_inputs, n_measurements> kf{};

        /// @brief State vector (x,y,yaw) of the Hfw transform
        Eigen::Vector3d state = Eigen::Vector3d::Zero();

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

        /// @brief Number of times the cost has been over the threshold
        int num_over_cost = 0;


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
         * @brief Run the field line optimisation
         * @param initial_guess The initial guess for the field line
         * @param observations The observations of the field line points
         * @param field_intersections The field intersections
         * @return Pair <optimisation solution (x,y,theta), final cost>
         */
        std::pair<Eigen::Vector3d, double> run_field_line_optimisation(
            const Eigen::Vector3d& initial_guess,
            const std::vector<Eigen::Vector3d>& field_lines,
            const std::shared_ptr<const FieldIntersections>& field_intersections,
            const std::shared_ptr<const Goals>& goals);

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
