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
#ifndef MODULE_PLANNING_MODELLEDMPCWALKPATH_MODELLEDWALKMPC_HPP
#define MODULE_PLANNING_MODELLEDMPCWALKPATH_MODELLEDWALKMPC_HPP

#include <Eigen/Core>
#include <string>
#include <vector>

#include "HammersteinModel.hpp"

// Forward declaration of the generated acados solver, so including this header doesn't pull in acados
struct modelled_mpc_walk_path_solver_capsule;

namespace module::planning::modelled_walk_mpc {

    /// @brief Tuning of the MPC. The horizon, its step, the model's structure and the number of obstacle slots are
    /// fixed by the generated solver (codegen/generate_solver.py); everything here can change at runtime.
    struct Config {
        /// @brief Maximum velocity command per axis [vx (m/s), vy (m/s), vtheta (rad/s)]
        Eigen::Vector3d max_velocity = Eigen::Vector3d(1.0, 0.5, 1.5);
        /// @brief Maximum backward (negative x) velocity command magnitude (m/s)
        double max_backward_velocity = 0.15;
        /// @brief Maximum change of the command per second [m/s², m/s², rad/s²]
        Eigen::Vector3d max_acceleration = Eigen::Vector3d(1.0, 1.0, 2.0);

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
        /// @brief Weight of the delivered velocity's magnitude, per axis
        Eigen::Vector3d w_effort = Eigen::Vector3d(0.01, 0.01, 0.01);
        /// @brief Weight of the change of command between steps, per axis
        Eigen::Vector3d w_rate = Eigen::Vector3d(1.0, 1.0, 0.5);

        /// @brief Clearance kept from each obstacle centre (m)
        double obstacle_radius = 0.8;
        /// @brief Linear penalty on getting inside an obstacle's clearance (the quadratic penalty is 1)
        double w_slack = 1000.0;

        /// @brief The policy's response the MPC plans with
        HammersteinModel model{};
        /// @brief Width over which the solver's model smooths the kinks of the paths of each command axis
        Eigen::Vector3d kink_smoothing = Eigen::Vector3d(0.01, 0.01, 0.05);

        /// @brief SQP iterations per solve. Reaching the cap isn't a failure: the iterate is used.
        int max_iterations = 10;
    };

    /// @brief What one solve produced
    struct Solution {
        /// @brief Command to send now [vx, vy, vtheta], within the velocity and acceleration limits
        Eigen::Vector3d command = Eigen::Vector3d::Zero();
        /// @brief Whether the command is usable: the solve converged or stopped at its iteration cap
        bool success = false;
        /// @brief acados status (0 success, 2 iteration cap, see acados/utils/types.h), or -1 for non-finite input
        int status = -1;
        /// @brief SQP iterations used
        int iterations = 0;
        /// @brief Wall-clock time of the solve (s)
        double solve_time = 0.0;
        /// @brief Predicted poses (x, y, theta) over the horizon, in the robot frame at the time of the solve
        std::vector<Eigen::Vector3d> states{};
        /// @brief Predicted delivered velocities over the horizon
        std::vector<Eigen::Vector3d> velocities{};
        /// @brief Planned commands over the horizon
        std::vector<Eigen::Vector3d> commands{};

        /// @brief Human-readable status
        [[nodiscard]] std::string status_string() const;
    };

    /// @brief The walk MPC planning with the policy's identified response (MPC-2), solved with acados's SQP. Plans in
    /// the robot frame at the time of each solve: the robot is at the origin facing +x, and the target and obstacles
    /// are given relative to it.
    ///
    /// The delivered velocity isn't measured: the model's lag states are an estimate, run from the commands sent.
    /// Call advance with the time since the last solve before each solve.
    class ModelledWalkMPC {
    public:
        /// @brief Steps in the horizon
        static const int N;
        /// @brief Horizon step and planner period (s)
        static const double DT;
        /// @brief Obstacle slots; obstacles beyond this are dropped, furthest first
        static const int MAX_OBSTACLES;
        /// @brief The model's sample time the solver was generated for (s)
        static const double MODEL_TS;

        explicit ModelledWalkMPC(const Config& config);
        ~ModelledWalkMPC();
        ModelledWalkMPC(const ModelledWalkMPC&)            = delete;
        ModelledWalkMPC& operator=(const ModelledWalkMPC&) = delete;
        ModelledWalkMPC(ModelledWalkMPC&&)                 = delete;
        ModelledWalkMPC& operator=(ModelledWalkMPC&&)      = delete;

        /// @brief Applies new tuning and model. Throws if the model's sample time isn't the one the solver was
        /// generated for.
        void configure(const Config& config);

        /// @brief Forgets the previous command, the solution and the model's state: the next solve starts from
        /// standing still
        void reset();

        /// @brief Runs the model's estimate of the robot forward by elapsed seconds, with the previous command held
        void advance(double elapsed);

        /// @brief Plans from the robot's current pose and estimated velocity, and returns the command to send now.
        /// On success, the command becomes the previous command. On failure, nothing changes: call
        /// set_previous_command with whatever was sent instead.
        /// @param target Goal pose (x, y, theta) in the robot frame
        /// @param obstacles Obstacle centres (x, y) in the robot frame
        Solution solve(const Eigen::Vector3d& target, const std::vector<Eigen::Vector2d>& obstacles);

        /// @brief Records a command sent that didn't come from solve (e.g. a fallback's), clipped to the velocity
        /// limits, and drops the warm start
        void set_previous_command(const Eigen::Vector3d& command);

        /// @brief The command sent last, which the next solve's acceleration limits are measured from
        [[nodiscard]] const Eigen::Vector3d& previous_command() const {
            return u_prev;
        }

        /// @brief The model's estimate of the velocity the robot is delivering now
        [[nodiscard]] Eigen::Vector3d delivered_velocity() const {
            return HammersteinModel::delivered(lags);
        }

        /// @brief Clips a command to the velocity limits
        [[nodiscard]] Eigen::Vector3d clip_to_limits(const Eigen::Vector3d& command) const;

    private:
        /// @brief Sets the parameters every stage shares (weights, model and obstacles), for every stage
        void fill_shared_parameters(const std::vector<Eigen::Vector2d>& obstacles);

        modelled_mpc_walk_path_solver_capsule* capsule = nullptr;
        Config cfg{};

        /// @brief Last command sent
        Eigen::Vector3d u_prev = Eigen::Vector3d::Zero();
        /// @brief The model's lag states: its estimate of the robot's gait-averaged velocity, path by path
        HammersteinModel::Lags lags = HammersteinModel::Lags::Zero();
        /// @brief Time advanced that is less than a model sample, carried to the next advance (s)
        double leftover = 0.0;
        /// @brief Last solution's inputs (change of command per second), for the next warm start
        std::vector<Eigen::Vector3d> a_guess{};
        /// @brief Per-stage parameter vectors, N + 1 of them
        std::vector<std::vector<double>> params{};
    };

}  // namespace module::planning::modelled_walk_mpc

#endif  // MODULE_PLANNING_MODELLEDMPCWALKPATH_MODELLEDWALKMPC_HPP
