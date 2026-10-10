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
#include "ModelledWalkMPC.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_modelled_mpc_walk_path.h"
#include "modelled_mpc_walk_path_layout.h"

namespace module::planning::modelled_walk_mpc {

    const int ModelledWalkMPC::N             = MODELLED_MPC_WALK_PATH_N;
    const double ModelledWalkMPC::DT         = MODELLED_MPC_WALK_PATH_DT;
    const int ModelledWalkMPC::MAX_OBSTACLES = MODELLED_MPC_WALK_PATH_MAX_OBSTACLES;
    const double ModelledWalkMPC::MODEL_TS   = MODELLED_MPC_WALK_PATH_MODEL_TS;
    const int ModelledWalkMPC::ENVELOPE_ROWS = MODELLED_MPC_WALK_PATH_ENVELOPE_ROWS;

    static_assert(MODELLED_MPC_WALK_PATH_NX == 15 && MODELLED_MPC_WALK_PATH_NU == 3,
                  "ModelledWalkMPC expects the state (pose, lags, command)");
    static_assert(MODELLED_MPC_WALK_PATH_PATHS == 9 && MODELLED_MPC_WALK_PATH_UNITS == HammersteinPath::UNITS,
                  "The generated solver's model structure doesn't match HammersteinModel's");

    namespace {

        using State = Eigen::Matrix<double, MODELLED_MPC_WALK_PATH_NX, 1>;

        /// @brief Wraps an angle to [-pi, pi]
        double wrap(const double angle) {
            return std::remainder(angle, 2.0 * M_PI);
        }

        /// @brief One model sample of the pose, moving with the delivered velocity v, as in the generated model
        Eigen::Vector3d move(const Eigen::Vector3d& pose, const Eigen::Vector3d& v, const double ts) {
            const double heading = pose.z() + v.z() * ts / 2;
            const double c       = std::cos(heading);
            const double s       = std::sin(heading);
            return pose + ts * Eigen::Vector3d(c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z());
        }

    }  // namespace

    std::vector<EnvelopeRow> envelope_from_yaml(const YAML::Node& node) {
        std::vector<EnvelopeRow> rows{};
        for (const auto& r : node) {
            const auto v = r.as<std::vector<double>>();
            if (v.size() != 4) {
                throw std::runtime_error("ModelledMPCWalkPath: an envelope row is [n_vx, n_vy, n_wz, d]");
            }
            rows.push_back({Eigen::Vector3d(v[0], v[1], v[2]), v[3]});
        }
        return rows;
    }

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

    ModelledWalkMPC::ModelledWalkMPC(const Config& config) {
        capsule = modelled_mpc_walk_path_acados_create_capsule();
        if (capsule == nullptr || modelled_mpc_walk_path_acados_create(capsule) != 0) {
            throw std::runtime_error("ModelledMPCWalkPath: creating the acados solver failed");
        }
        params.assign(N + 1, std::vector<double>(MODELLED_MPC_WALK_PATH_NP, 0.0));
        configure(config);
        reset();
    }

    ModelledWalkMPC::~ModelledWalkMPC() {
        modelled_mpc_walk_path_acados_free(capsule);
        modelled_mpc_walk_path_acados_free_capsule(capsule);
    }

    void ModelledWalkMPC::configure(const Config& config) {
        if (std::abs(config.model.sample_time - MODEL_TS) > 1e-9) {
            throw std::runtime_error("ModelledMPCWalkPath: the model's sample time is "
                                     + std::to_string(config.model.sample_time)
                                     + " s, but the solver was generated for " + std::to_string(MODEL_TS) + " s");
        }
        if (int(config.envelope.size()) > ENVELOPE_ROWS) {
            throw std::runtime_error("ModelledMPCWalkPath: the envelope has " + std::to_string(config.envelope.size())
                                     + " rows, but the solver was generated for " + std::to_string(ENVELOPE_ROWS));
        }
        cfg                        = config;
        ocp_nlp_config* nlp_config = modelled_mpc_walk_path_acados_get_nlp_config(capsule);
        ocp_nlp_dims* nlp_dims     = modelled_mpc_walk_path_acados_get_nlp_dims(capsule);
        ocp_nlp_in* nlp_in         = modelled_mpc_walk_path_acados_get_nlp_in(capsule);
        ocp_nlp_out* nlp_out       = modelled_mpc_walk_path_acados_get_nlp_out(capsule);

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

        // The capability envelope on the command part of the state, at stages 1..N; unused rows, and every row at
        // stage 0 (pinned to the current state, which may be outside after a fallback), are switched off
        constexpr double big = 1e9;
        const int nx         = MODELLED_MPC_WALK_PATH_NX;
        std::vector<double> C(ENVELOPE_ROWS * nx, 0.0);  // column major
        std::vector<double> lg(ENVELOPE_ROWS, -big);
        std::vector<double> ug(ENVELOPE_ROWS, big);
        std::vector<double> ug_off(ENVELOPE_ROWS, big);
        for (int r = 0; r < int(cfg.envelope.size()); ++r) {
            for (int i = 0; i < 3; ++i) {
                C[(MODELLED_MPC_WALK_PATH_S_COMMAND + i) * ENVELOPE_ROWS + r] = cfg.envelope[r].normal[i];
            }
            ug[r] = cfg.envelope[r].bound;
        }
        for (int k = 0; k <= N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "C", C.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lg", lg.data());
            ocp_nlp_constraints_model_set(nlp_config,
                                          nlp_dims,
                                          nlp_in,
                                          nlp_out,
                                          k,
                                          "ug",
                                          k == 0 ? ug_off.data() : ug.data());
        }

        // Slack penalties: the envelope's at every stage, then the obstacles' at stages 1..N (stage 0 has none)
        std::vector<double> zu(ENVELOPE_ROWS, cfg.w_envelope);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "zu", zu.data());
        zu.resize(ENVELOPE_ROWS + MAX_OBSTACLES, cfg.w_slack);
        for (int k = 1; k <= N; ++k) {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, k, "zu", zu.data());
        }

        int max_iter = std::max(1, cfg.max_iterations);
        ocp_nlp_solver_opts_set(nlp_config, modelled_mpc_walk_path_acados_get_nlp_opts(capsule), "max_iter", &max_iter);

        // A new configuration can make the old warm start infeasible
        a_guess.clear();
        u_prev = clip_to_limits(u_prev);
    }

    void ModelledWalkMPC::reset() {
        u_prev = Eigen::Vector3d::Zero();
        lags.setZero();
        leftover = 0.0;
        a_guess.clear();
        // Zero the iterates and the QP solver's memory, but not the numerical values: that would put back the
        // generated code's placeholder limits and parameters
        modelled_mpc_walk_path_acados_reset(capsule, 1, 0, 0, 0);
    }

    void ModelledWalkMPC::advance(const double elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0) {
            return;
        }
        // Whole samples, carrying the remainder to the next advance
        const double t = elapsed + leftover;
        const auto n   = std::floor(t / MODEL_TS + 1e-6);
        leftover       = std::max(0.0, t - n * MODEL_TS);
        lags           = cfg.model.step(lags, u_prev, int(std::min(n, 1e6)));
    }

    Eigen::Vector3d ModelledWalkMPC::clip_to_limits(const Eigen::Vector3d& command) const {
        const Eigen::Vector3d lo(-cfg.max_backward_velocity, -cfg.max_velocity.y(), -cfg.max_velocity.z());
        return command.cwiseMax(lo).cwiseMin(cfg.max_velocity);
    }

    double ModelledWalkMPC::envelope_violation(const Eigen::Vector3d& command) const {
        double violation = -std::numeric_limits<double>::infinity();
        for (const auto& row : cfg.envelope) {
            violation = std::max(violation, row.normal.dot(command) - row.bound);
        }
        return violation;
    }

    void ModelledWalkMPC::set_previous_command(const Eigen::Vector3d& command) {
        u_prev = command.allFinite() ? clip_to_limits(command) : Eigen::Vector3d::Zero();
        a_guess.clear();
    }

    void ModelledWalkMPC::fill_shared_parameters(const std::vector<Eigen::Vector2d>& obstacles) {
        for (auto& p : params) {
            p[MODELLED_MPC_WALK_PATH_P_W_POSITION]      = cfg.w_position;
            p[MODELLED_MPC_WALK_PATH_P_W_HEADING]       = cfg.w_heading;
            p[MODELLED_MPC_WALK_PATH_P_W_FACE]          = cfg.w_face;
            p[MODELLED_MPC_WALK_PATH_P_HUBER_DELTA]     = cfg.huber_delta;
            p[MODELLED_MPC_WALK_PATH_P_OBSTACLE_RADIUS] = cfg.obstacle_radius;
            for (int i = 0; i < 3; ++i) {
                p[MODELLED_MPC_WALK_PATH_P_W_EFFORT + i]       = cfg.w_effort[i];
                p[MODELLED_MPC_WALK_PATH_P_W_RATE + i]         = cfg.w_rate[i];
                p[MODELLED_MPC_WALK_PATH_P_KINK_SMOOTHING + i] = cfg.kink_smoothing[i];
            }
            for (int j = 0; j < MAX_OBSTACLES; ++j) {
                const bool used                                   = j < int(obstacles.size());
                p[MODELLED_MPC_WALK_PATH_P_OBSTACLES + 2 * j]     = used ? obstacles[j].x() : 0.0;
                p[MODELLED_MPC_WALK_PATH_P_OBSTACLES + 2 * j + 1] = used ? obstacles[j].y() : 0.0;
                p[MODELLED_MPC_WALK_PATH_P_ACTIVE + j]            = used ? 1.0 : 0.0;
            }
            for (int n = 0; n < MODELLED_MPC_WALK_PATH_PATHS; ++n) {
                const HammersteinPath& path = cfg.model.paths[n];
                double* q                   = &p[MODELLED_MPC_WALK_PATH_P_MODEL + n * MODELLED_MPC_WALK_PATH_PATH_SIZE];
                q[MODELLED_MPC_WALK_PATH_PATH_GAIN]   = path.gain;
                q[MODELLED_MPC_WALK_PATH_PATH_POLE]   = path.pole;
                q[MODELLED_MPC_WALK_PATH_PATH_LINEAR] = path.linear;
                q[MODELLED_MPC_WALK_PATH_PATH_OFFSET] = path.offset;
                std::copy(path.coef.begin(), path.coef.end(), q + MODELLED_MPC_WALK_PATH_PATH_COEF);
                std::copy(path.translation.begin(),
                          path.translation.end(),
                          q + MODELLED_MPC_WALK_PATH_PATH_TRANSLATION);
            }
        }
    }

    Solution ModelledWalkMPC::solve(const Eigen::Vector3d& target, const std::vector<Eigen::Vector2d>& all_obstacles) {
        Solution solution{};

        const bool finite =
            target.allFinite() && std::ranges::all_of(all_obstacles, [](const auto& o) { return o.allFinite(); });
        if (!finite || !lags.allFinite()) {
            // Bad input: report it and start the next solve cold
            a_guess.clear();
            if (!lags.allFinite()) {
                lags.setZero();
            }
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

        // Roll the guess out from the current state, through the solver's (smoothed) model
        std::vector<State> s_guess(N + 1);
        s_guess[0] << 0.0, 0.0, 0.0, lags, u_prev;
        for (int k = 0; k < N; ++k) {
            const Eigen::Vector3d u  = s_guess[k].tail<3>() + a_guess[k] * DT;
            Eigen::Vector3d pose     = s_guess[k].head<3>();
            HammersteinModel::Lags z = s_guess[k].segment<9>(MODELLED_MPC_WALK_PATH_S_LAG);
            for (int m = 0; m < MODELLED_MPC_WALK_PATH_SUBSTEPS; ++m) {
                pose = move(pose, HammersteinModel::delivered(z), MODEL_TS);
                z    = cfg.model.step(z, u, cfg.kink_smoothing);
            }
            s_guess[k + 1] << pose, z, u;
        }

        // Per-stage parameters: the heading fade-in and the travel bearing come from the guess, not the decision
        // variables. Were the fade-in a function of the optimised path, walking away from the target would make
        // heading errors cheaper, and the robot would swing out and back to turn on the spot.
        fill_shared_parameters(obstacles);
        for (int k = 0; k <= N; ++k) {
            const Eigen::Vector2d d                = target.head<2>() - s_guess[k].head<2>();
            auto& p                                = params[k];
            p[MODELLED_MPC_WALK_PATH_P_TARGET]     = target.x();
            p[MODELLED_MPC_WALK_PATH_P_TARGET + 1] = target.y();
            p[MODELLED_MPC_WALK_PATH_P_TARGET + 2] = target.z();
            p[MODELLED_MPC_WALK_PATH_P_GATE] = std::exp(-d.squaredNorm() / (cfg.heading_radius * cfg.heading_radius));
            p[MODELLED_MPC_WALK_PATH_P_BEARING] = d.norm() > 1e-3 ? std::atan2(d.y(), d.x()) : target.z();
            modelled_mpc_walk_path_acados_update_params(capsule, k, p.data(), MODELLED_MPC_WALK_PATH_NP);
        }

        ocp_nlp_config* nlp_config = modelled_mpc_walk_path_acados_get_nlp_config(capsule);
        ocp_nlp_dims* nlp_dims     = modelled_mpc_walk_path_acados_get_nlp_dims(capsule);
        ocp_nlp_in* nlp_in         = modelled_mpc_walk_path_acados_get_nlp_in(capsule);
        ocp_nlp_out* nlp_out       = modelled_mpc_walk_path_acados_get_nlp_out(capsule);
        ocp_nlp_solver* nlp_solver = modelled_mpc_walk_path_acados_get_nlp_solver(capsule);

        // Pin the first state to the current one
        State x0 = s_guess[0];
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0.data());
        for (int k = 0; k <= N; ++k) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", s_guess[k].data());
            if (k < N) {
                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", a_guess[k].data());
            }
        }

        const auto start    = std::chrono::steady_clock::now();
        solution.status     = modelled_mpc_walk_path_acados_solve(capsule);
        solution.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        ocp_nlp_get(nlp_solver, "sqp_iter", &solution.iterations);

        std::vector<State> s(N + 1);
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
        solution.velocities.reserve(N + 1);
        solution.commands.reserve(N);
        for (int k = 0; k <= N; ++k) {
            solution.states.emplace_back(s[k].head<3>());
            solution.velocities.emplace_back(
                HammersteinModel::delivered(s[k].segment<9>(MODELLED_MPC_WALK_PATH_S_LAG)));
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
        solution.envelope_violation = envelope_violation(solution.command);
        a_guess                     = a;
        u_prev                      = solution.command;
        return solution;
    }

}  // namespace module::planning::modelled_walk_mpc
