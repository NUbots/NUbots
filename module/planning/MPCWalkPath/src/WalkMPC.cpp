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
#include "WalkMPC.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_mpc_walk_path.h"
#include "mpc_walk_path_layout.h"

namespace module::planning::walk_mpc {

    const int WalkMPC::N             = MPC_WALK_PATH_N;
    const double WalkMPC::DT         = MPC_WALK_PATH_DT;
    const int WalkMPC::MAX_OBSTACLES = MPC_WALK_PATH_MAX_OBSTACLES;

    static_assert(MPC_WALK_PATH_NX == 6 && MPC_WALK_PATH_NU == 3, "WalkMPC expects the state (pose, command)");

    namespace {

        using Vector6d = Eigen::Matrix<double, 6, 1>;

        /// @brief Wraps an angle to [-pi, pi]
        double wrap(const double angle) {
            return std::remainder(angle, 2.0 * M_PI);
        }

        /// @brief Omnidirectional kinematics: the body twist u rotated into the solve frame
        Eigen::Vector3d f(const Eigen::Vector3d& pose, const Eigen::Vector3d& u) {
            const double c = std::cos(pose.z());
            const double s = std::sin(pose.z());
            return {c * u.x() - s * u.y(), s * u.x() + c * u.y(), u.z()};
        }

        /// @brief One RK4 step of the kinematics, as in the generated model
        Eigen::Vector3d rk4(const Eigen::Vector3d& pose, const Eigen::Vector3d& u, const double dt) {
            const Eigen::Vector3d k1 = f(pose, u);
            const Eigen::Vector3d k2 = f(pose + dt / 2 * k1, u);
            const Eigen::Vector3d k3 = f(pose + dt / 2 * k2, u);
            const Eigen::Vector3d k4 = f(pose + dt * k3, u);
            return pose + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        }

    }  // namespace

    std::string Solution::status_string() const {
        switch (status) {
            case -1: return "non-finite input";
            case 0: return "success";
            case 1: return "NaN detected";
            case 2: return "iteration cap";
            case 3: return "minimum step";
            case 4: return "QP failure";
            case 5: return "ready";
            case 6: return "unbounded";
            case 7: return "timeout";
            default: return "acados status " + std::to_string(status);
        }
    }

    WalkMPC::WalkMPC(const Config& config) {
        capsule = mpc_walk_path_acados_create_capsule();
        if (capsule == nullptr || mpc_walk_path_acados_create(capsule) != 0) {
            throw std::runtime_error("MPCWalkPath: creating the acados solver failed");
        }
        params.assign(N + 1, std::vector<double>(MPC_WALK_PATH_NP, 0.0));
        configure(config);
        reset();
    }

    WalkMPC::~WalkMPC() {
        mpc_walk_path_acados_free(capsule);
        mpc_walk_path_acados_free_capsule(capsule);
    }

    void WalkMPC::configure(const Config& config) {
        cfg                        = config;
        ocp_nlp_config* nlp_config = mpc_walk_path_acados_get_nlp_config(capsule);
        ocp_nlp_dims* nlp_dims     = mpc_walk_path_acados_get_nlp_dims(capsule);
        ocp_nlp_in* nlp_in         = mpc_walk_path_acados_get_nlp_in(capsule);
        ocp_nlp_out* nlp_out       = mpc_walk_path_acados_get_nlp_out(capsule);

        // Acceleration limits on the input, at every stage with an input
        std::array<double, 3> lbu{-cfg.max_acceleration.x(), -cfg.max_acceleration.y(), -cfg.max_acceleration.z()};
        std::array<double, 3> ubu{cfg.max_acceleration.x(), cfg.max_acceleration.y(), cfg.max_acceleration.z()};
        // Velocity limits on the command part of the state, at stages 1..N (stage 0 is pinned to the current state)
        std::array<double, 3> lbx{-cfg.max_backward_velocity, -cfg.max_velocity.y(), -cfg.max_velocity.z()};
        std::array<double, 3> ubx{cfg.max_velocity.x(), cfg.max_velocity.y(), cfg.max_velocity.z()};
        for (int k = 0; k < N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lbu", lbu.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "ubu", ubu.data());
        }
        for (int k = 1; k <= N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lbx", lbx.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "ubx", ubx.data());
        }

        // Slack penalty on the obstacle constraints (stages 1..N; stage 0 has none)
        std::vector<double> zu(MAX_OBSTACLES, cfg.w_slack);
        for (int k = 1; k <= N; ++k) {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, k, "zu", zu.data());
        }

        int max_iter = std::max(1, cfg.max_iterations);
        ocp_nlp_solver_opts_set(nlp_config, mpc_walk_path_acados_get_nlp_opts(capsule), "max_iter", &max_iter);

        // A new configuration can make the old warm start infeasible
        a_guess.clear();
        u_prev = clip_to_limits(u_prev);
    }

    void WalkMPC::reset() {
        u_prev = Eigen::Vector3d::Zero();
        a_guess.clear();
        // Zero the iterates and the QP solver's memory, but not the numerical values: that would put back the
        // generated code's placeholder limits and parameters
        mpc_walk_path_acados_reset(capsule, 1, 0, 0, 0);
    }

    Eigen::Vector3d WalkMPC::clip_to_limits(const Eigen::Vector3d& command) const {
        const Eigen::Vector3d lo(-cfg.max_backward_velocity, -cfg.max_velocity.y(), -cfg.max_velocity.z());
        return command.cwiseMax(lo).cwiseMin(cfg.max_velocity);
    }

    void WalkMPC::set_previous_command(const Eigen::Vector3d& command) {
        u_prev = command.allFinite() ? clip_to_limits(command) : Eigen::Vector3d::Zero();
        a_guess.clear();
    }

    void WalkMPC::fill_shared_parameters(const std::vector<Eigen::Vector2d>& obstacles) {
        for (auto& p : params) {
            p[MPC_WALK_PATH_P_W_POSITION]      = cfg.w_position;
            p[MPC_WALK_PATH_P_W_HEADING]       = cfg.w_heading;
            p[MPC_WALK_PATH_P_W_FACE]          = cfg.w_face;
            p[MPC_WALK_PATH_P_HUBER_DELTA]     = cfg.huber_delta;
            p[MPC_WALK_PATH_P_OBSTACLE_RADIUS] = cfg.obstacle_radius;
            for (int i = 0; i < 3; ++i) {
                p[MPC_WALK_PATH_P_W_EFFORT + i] = cfg.w_effort[i];
                p[MPC_WALK_PATH_P_W_RATE + i]   = cfg.w_rate[i];
            }
            for (int j = 0; j < MAX_OBSTACLES; ++j) {
                const bool used                          = j < int(obstacles.size());
                p[MPC_WALK_PATH_P_OBSTACLES + 2 * j]     = used ? obstacles[j].x() : 0.0;
                p[MPC_WALK_PATH_P_OBSTACLES + 2 * j + 1] = used ? obstacles[j].y() : 0.0;
                p[MPC_WALK_PATH_P_ACTIVE + j]            = used ? 1.0 : 0.0;
            }
        }
    }

    Solution WalkMPC::solve(const Eigen::Vector3d& target, const std::vector<Eigen::Vector2d>& all_obstacles) {
        Solution solution{};

        const bool finite =
            target.allFinite() && std::ranges::all_of(all_obstacles, [](const auto& o) { return o.allFinite(); });
        if (!finite) {
            // Bad input: report it and start the next solve cold
            a_guess.clear();
            solution.status = -1;
            return solution;
        }

        // Keep the nearest obstacles, as they are the ones the horizon can reach
        std::vector<Eigen::Vector2d> obstacles = all_obstacles;
        std::ranges::sort(obstacles, {}, &Eigen::Vector2d::squaredNorm);
        if (int(obstacles.size()) > MAX_OBSTACLES) {
            obstacles.resize(MAX_OBSTACLES);
        }

        // Initial guess: the last solution's inputs shifted by one step, or, from cold, a gentle turn towards the
        // target. With the target exactly behind, or a final heading exactly opposite, turning left and right are
        // equally good and the gradient is zero, so a solver started from standing still can stay there.
        if (int(a_guess.size()) == N) {
            std::rotate(a_guess.begin(), a_guess.begin() + 1, a_guess.end());
            a_guess.back().setZero();
        }
        else {
            a_guess.assign(N, Eigen::Vector3d::Zero());
            const double distance = target.head<2>().norm();
            const double turn_to  = distance > cfg.heading_radius ? std::atan2(target.y(), target.x()) : target.z();
            const double sign     = wrap(turn_to) < 0.0 ? -1.0 : 1.0;
            a_guess[0].z()        = std::clamp(sign * 0.3 / DT, -cfg.max_acceleration.z(), cfg.max_acceleration.z());
        }

        // Roll the guess out from the current state
        std::vector<Vector6d> s_guess(N + 1);
        s_guess[0] << 0.0, 0.0, 0.0, u_prev;
        for (int k = 0; k < N; ++k) {
            const Eigen::Vector3d u = s_guess[k].tail<3>() + a_guess[k] * DT;
            s_guess[k + 1] << rk4(s_guess[k].head<3>(), u, DT), u;
        }

        // Per-stage parameters: the heading fade-in and the travel bearing come from the guess, not the decision
        // variables. Were the fade-in a function of the optimised path, walking away from the target would make
        // heading errors cheaper, and the robot would swing out and back to turn on the spot.
        fill_shared_parameters(obstacles);
        for (int k = 0; k <= N; ++k) {
            const Eigen::Vector2d d       = target.head<2>() - s_guess[k].head<2>();
            auto& p                       = params[k];
            p[MPC_WALK_PATH_P_TARGET]     = target.x();
            p[MPC_WALK_PATH_P_TARGET + 1] = target.y();
            p[MPC_WALK_PATH_P_TARGET + 2] = target.z();
            p[MPC_WALK_PATH_P_GATE]       = std::exp(-d.squaredNorm() / (cfg.heading_radius * cfg.heading_radius));
            p[MPC_WALK_PATH_P_BEARING]    = d.norm() > 1e-3 ? std::atan2(d.y(), d.x()) : target.z();
            mpc_walk_path_acados_update_params(capsule, k, p.data(), MPC_WALK_PATH_NP);
        }

        ocp_nlp_config* nlp_config = mpc_walk_path_acados_get_nlp_config(capsule);
        ocp_nlp_dims* nlp_dims     = mpc_walk_path_acados_get_nlp_dims(capsule);
        ocp_nlp_in* nlp_in         = mpc_walk_path_acados_get_nlp_in(capsule);
        ocp_nlp_out* nlp_out       = mpc_walk_path_acados_get_nlp_out(capsule);
        ocp_nlp_solver* nlp_solver = mpc_walk_path_acados_get_nlp_solver(capsule);

        // Pin the first state to the current one
        Vector6d x0 = s_guess[0];
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0.data());
        for (int k = 0; k <= N; ++k) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", s_guess[k].data());
            if (k < N) {
                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", a_guess[k].data());
            }
        }

        const auto start    = std::chrono::steady_clock::now();
        solution.status     = mpc_walk_path_acados_solve(capsule);
        solution.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        ocp_nlp_get(nlp_solver, "sqp_iter", &solution.iterations);

        std::vector<Vector6d> s(N + 1);
        std::vector<Eigen::Vector3d> a(N);
        for (int k = 0; k <= N; ++k) {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", s[k].data());
            if (k < N) {
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", a[k].data());
            }
        }
        const bool solution_finite = std::ranges::all_of(a, [](const auto& v) { return v.allFinite(); })
                                     && std::ranges::all_of(s, [](const auto& v) { return v.allFinite(); });
        // 0 is converged; 2 is the iteration cap, which is how the solver is used (the iterate is still feasible
        // with respect to the bounds)
        solution.success = (solution.status == 0 || solution.status == 2) && solution_finite;

        solution.states.reserve(N + 1);
        solution.commands.reserve(N);
        for (int k = 0; k <= N; ++k) {
            solution.states.emplace_back(s[k].head<3>());
            if (k > 0) {
                solution.commands.emplace_back(s[k].tail<3>());
            }
        }

        if (!solution.success) {
            // Don't warm start from a failed solve; the caller decides what to send and reports it back
            a_guess.clear();
            return solution;
        }

        // The command applied in the first step, clipped so it is exactly within the limits despite the solver's
        // tolerance
        const Eigen::Vector3d a_max = cfg.max_acceleration * DT;
        solution.command            = clip_to_limits(s[1].tail<3>().cwiseMax(u_prev - a_max).cwiseMin(u_prev + a_max));
        a_guess                     = a;
        u_prev                      = solution.command;
        return solution;
    }

}  // namespace module::planning::walk_mpc
