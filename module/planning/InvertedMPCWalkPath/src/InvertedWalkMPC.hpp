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
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_INVERTEDWALKMPC_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_INVERTEDWALKMPC_HPP

#include <Eigen/Core>
#include <limits>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// Forward declaration of the generated acados solver, so including this header doesn't pull in acados
struct inverted_mpc_walk_path_solver_capsule;

namespace module::planning::inverted_walk_mpc {

    /// @brief One edge of the capability envelope: the constraint normal·velocity ≤ bound
    struct EnvelopeRow {
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        double bound           = 0.0;
    };

    /// @brief Reads the capability envelope from its configuration (written by codegen/import_response.py), a list of
    /// [n_vx, n_vy, n_wz, d] rows
    std::vector<EnvelopeRow> envelope_from_yaml(const YAML::Node& node);

    /// @brief Tuning of the MPC. The horizon, its step, the number of obstacle slots and the number of envelope rows
    /// are fixed by the generated solver (codegen/generate_solver.py); everything here can change at runtime.
    struct Config {
        /// @brief Smallest planned velocity per axis [vx (m/s), vy (m/s), vtheta (rad/s)]
        Eigen::Vector3d lower_velocity = Eigen::Vector3d(-0.15, -0.5, -1.5);
        /// @brief Largest planned velocity per axis [vx (m/s), vy (m/s), vtheta (rad/s)]
        Eigen::Vector3d upper_velocity = Eigen::Vector3d(1.0, 0.5, 1.5);

        /// @brief Weight of the pseudo-Huber distance to the target
        double w_position = 1.0;
        /// @brief Distance (m) below which the position cost is quadratic; above it, linear
        double huber_delta = 0.2;
        /// @brief Weight of the final-heading error, faded in near the target
        double w_heading = 1.0;
        /// @brief Weight of facing the direction of travel far from the target (0 leaves the heading free there)
        double w_face = 0.0;
        /// @brief Distance (m) over which the final-heading cost fades in
        double heading_radius = 0.6;
        /// @brief Weight of the planned velocity's magnitude, per axis
        Eigen::Vector3d w_effort = Eigen::Vector3d(0.01, 0.01, 0.01);
        /// @brief Weight of the change of planned velocity between steps, per axis
        Eigen::Vector3d w_rate = Eigen::Vector3d(1.0, 1.0, 0.5);

        /// @brief Clearance kept from each obstacle centre (m)
        double obstacle_radius = 0.8;
        /// @brief Linear penalty on getting inside an obstacle's clearance (the quadratic penalty is 1)
        double w_slack = 1000.0;

        /// @brief The capability envelope: the velocities the policy is known to deliver, at most ENVELOPE_ROWS rows.
        /// Empty leaves only the velocity limits.
        std::vector<EnvelopeRow> envelope{};
        /// @brief Linear penalty on planned velocities outside the envelope (the quadratic penalty is 1)
        double w_envelope = 1000.0;

        /// @brief SQP iterations per solve. Reaching the cap isn't a failure: the iterate is used.
        int max_iterations = 10;
    };

    /// @brief What one solve produced
    struct Solution {
        /// @brief Velocity planned for now [vx, vy, vtheta], within the velocity limits
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        /// @brief Whether the velocity is usable: the solve converged or stopped at its iteration cap
        bool success = false;
        /// @brief acados status (0 success, 2 iteration cap, see acados/utils/types.h), or -1 for non-finite input
        int status = -1;
        /// @brief SQP iterations used
        int iterations = 0;
        /// @brief Wall-clock time of the solve (s)
        double solve_time = 0.0;
        /// @brief Predicted poses (x, y, theta) over the horizon, in the robot frame at the time of the solve
        std::vector<Eigen::Vector3d> states{};
        /// @brief Planned velocities over the horizon
        std::vector<Eigen::Vector3d> velocities{};

        /// @brief Human-readable status
        [[nodiscard]] std::string status_string() const;
    };

    /// @brief MPCWalkPath's kinematic walk MPC with the capability envelope, solved with acados's SQP. It plans the
    /// velocity the robot delivers, assuming it is delivered at once; finding the command that delivers it is the
    /// caller's (ResponseTable). Plans in the robot frame at the time of each solve: the robot is at the origin facing
    /// +x, and the target and obstacles are given relative to it.
    class InvertedWalkMPC {
    public:
        /// @brief Steps in the horizon
        static const int N;
        /// @brief Horizon step and planner period (s)
        static const double DT;
        /// @brief Obstacle slots; obstacles beyond this are dropped, furthest first
        static const int MAX_OBSTACLES;
        /// @brief Rows the capability envelope can have
        static const int ENVELOPE_ROWS;

        explicit InvertedWalkMPC(const Config& config = Config{});
        ~InvertedWalkMPC();
        InvertedWalkMPC(const InvertedWalkMPC&)            = delete;
        InvertedWalkMPC& operator=(const InvertedWalkMPC&) = delete;
        InvertedWalkMPC(InvertedWalkMPC&&)                 = delete;
        InvertedWalkMPC& operator=(InvertedWalkMPC&&)      = delete;

        /// @brief Applies new tuning: the limits, weights, envelope, slack penalties and iteration cap. Throws if the
        /// envelope has too many rows.
        void configure(const Config& config);

        /// @brief Forgets the previous velocity and solution, so the next solve starts from standing still
        void reset();

        /// @brief Plans from the robot's current pose and returns the velocity to deliver now.
        /// On success, the velocity becomes the previous velocity the next solve's rate cost is measured from. On
        /// failure, nothing changes: call set_previous_velocity with whatever was delivered instead.
        /// @param target Goal pose (x, y, theta) in the robot frame
        /// @param obstacles Obstacle centres (x, y) in the robot frame
        Solution solve(const Eigen::Vector3d& target, const std::vector<Eigen::Vector2d>& obstacles);

        /// @brief Records a velocity that didn't come from solve (e.g. what a fallback's command delivers), clipped to
        /// the velocity limits, and drops the warm start
        void set_previous_velocity(const Eigen::Vector3d& velocity);

        /// @brief The velocity the next solve's rate cost is measured from
        [[nodiscard]] const Eigen::Vector3d& previous_velocity() const {
            return v_prev;
        }

        /// @brief Clips a velocity to the velocity limits
        [[nodiscard]] Eigen::Vector3d clip_to_limits(const Eigen::Vector3d& velocity) const;

        /// @brief How far a velocity is outside the capability envelope (≤ 0 inside, -infinity without one)
        [[nodiscard]] double envelope_violation(const Eigen::Vector3d& velocity) const;

    private:
        /// @brief Sets the parameters every stage shares (weights and obstacles) into params, for every stage
        void fill_shared_parameters(const std::vector<Eigen::Vector2d>& obstacles);

        inverted_mpc_walk_path_solver_capsule* capsule = nullptr;
        Config cfg{};

        /// @brief Last velocity planned (or delivered by a fallback)
        Eigen::Vector3d v_prev = Eigen::Vector3d::Zero();
        /// @brief Last solution's inputs (change of velocity per second), for the next warm start
        std::vector<Eigen::Vector3d> a_guess{};
        /// @brief Per-stage parameter vectors, N + 1 of them
        std::vector<std::vector<double>> params{};
    };

}  // namespace module::planning::inverted_walk_mpc

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_INVERTEDWALKMPC_HPP
