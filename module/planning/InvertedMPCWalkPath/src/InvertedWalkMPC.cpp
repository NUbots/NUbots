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
#include "InvertedWalkMPC.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "Problem.hpp"
#include "ocp/AcadosOcp.hpp"
#include "ocp/OcpFunctions.hpp"

namespace module::planning::inverted_walk_mpc {

    const int InvertedWalkMPC::N             = Problem::N;
    const double InvertedWalkMPC::DT         = Problem::DT;
    const int InvertedWalkMPC::MAX_OBSTACLES = Problem::MAX_OBSTACLES;
    const int InvertedWalkMPC::ENVELOPE_ROWS = Problem::ENVELOPE_ROWS;

    namespace {

        using P = Problem;

        using Vector6d = Eigen::Matrix<double, 6, 1>;

        /// @brief Wraps an angle to [-pi, pi]
        double wrap(const double angle) {
            return std::remainder(angle, 2.0 * M_PI);
        }

        /// @brief The problem's structure: velocity limits on the velocity part of the state, the envelope's rows on
        /// it, and the obstacle clearances
        ocp::Layout layout() {
            ocp::Layout layout{};
            layout.N              = P::N;
            layout.dt             = P::DT;
            layout.nx             = P::NX;
            layout.nu             = P::NU;
            layout.np             = P::NP;
            layout.bounded_states = {3, 4, 5};
            layout.ng             = P::ENVELOPE_ROWS;
            layout.nh             = P::NH;
            return layout;
        }

    }  // namespace

    std::vector<EnvelopeRow> envelope_from_yaml(const YAML::Node& node) {
        std::vector<EnvelopeRow> rows{};
        for (const auto& r : node) {
            const auto v = r.as<std::vector<double>>();
            if (v.size() != 4) {
                throw std::runtime_error("InvertedMPCWalkPath: an envelope row is [n_vx, n_vy, n_wz, d]");
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

    InvertedWalkMPC::InvertedWalkMPC(const Config& config)
        : ocp(std::make_unique<ocp::AcadosOcp>(layout(), ocp::ProblemFunctions<Problem>::functions())) {
        params.assign(N + 1, std::vector<double>(P::NP, 0.0));
        configure(config);
        reset();
    }

    InvertedWalkMPC::~InvertedWalkMPC() = default;

    void InvertedWalkMPC::configure(const Config& config) {
        if (int(config.envelope.size()) > ENVELOPE_ROWS) {
            throw std::runtime_error("InvertedMPCWalkPath: the envelope has " + std::to_string(config.envelope.size())
                                     + " rows, but the problem has " + std::to_string(ENVELOPE_ROWS));
        }
        cfg                        = config;
        ocp_nlp_config* nlp_config = ocp->config;
        ocp_nlp_dims* nlp_dims     = ocp->dims;
        ocp_nlp_in* nlp_in         = ocp->in;
        ocp_nlp_out* nlp_out       = ocp->out;

        // Velocity limits on the velocity part of the state, at stages 1..N (stage 0 is pinned to the current state)
        Eigen::Vector3d lbx = cfg.lower_velocity;
        Eigen::Vector3d ubx = cfg.upper_velocity;
        for (int k = 1; k <= N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lbx", lbx.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "ubx", ubx.data());
        }

        // The capability envelope on the velocity part of the state, at stages 1..N; unused rows, and every row at
        // stage 0 (pinned to the current state, which may be outside after a fallback), are switched off
        constexpr double big = 1e9;
        constexpr int nx     = P::NX;
        std::vector<double> C(ENVELOPE_ROWS * nx, 0.0);  // column major
        std::vector<double> lg(ENVELOPE_ROWS, -big);
        std::vector<double> ug(ENVELOPE_ROWS, big);
        std::vector<double> ug_off(ENVELOPE_ROWS, big);
        for (int r = 0; r < int(cfg.envelope.size()); ++r) {
            for (int i = 0; i < 3; ++i) {
                C[(3 + i) * ENVELOPE_ROWS + r] = cfg.envelope[r].normal[i];
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

        ocp->set_max_iterations(std::max(1, cfg.max_iterations));

        // A new configuration can make the old warm start infeasible
        a_guess.clear();
        v_prev = clip_to_limits(v_prev);
    }

    void InvertedWalkMPC::reset() {
        v_prev = Eigen::Vector3d::Zero();
        a_guess.clear();
        // Zero the iterates and the QP solver's memory, keeping the limits and parameters
        ocp->reset();
    }

    Eigen::Vector3d InvertedWalkMPC::clip_to_limits(const Eigen::Vector3d& velocity) const {
        return velocity.cwiseMax(cfg.lower_velocity).cwiseMin(cfg.upper_velocity);
    }

    double InvertedWalkMPC::envelope_violation(const Eigen::Vector3d& velocity) const {
        double violation = -std::numeric_limits<double>::infinity();
        for (const auto& row : cfg.envelope) {
            violation = std::max(violation, row.normal.dot(velocity) - row.bound);
        }
        return violation;
    }

    void InvertedWalkMPC::set_previous_velocity(const Eigen::Vector3d& velocity) {
        v_prev = velocity.allFinite() ? clip_to_limits(velocity) : Eigen::Vector3d::Zero();
        a_guess.clear();
    }

    void InvertedWalkMPC::fill_shared_parameters(const std::vector<Eigen::Vector2d>& obstacles) {
        for (auto& p : params) {
            p[P::P_W_POSITION]      = cfg.w_position;
            p[P::P_W_HEADING]       = cfg.w_heading;
            p[P::P_W_FACE]          = cfg.w_face;
            p[P::P_HUBER_DELTA]     = cfg.huber_delta;
            p[P::P_OBSTACLE_RADIUS] = cfg.obstacle_radius;
            for (int i = 0; i < 3; ++i) {
                p[P::P_W_EFFORT + i] = cfg.w_effort[i];
                p[P::P_W_RATE + i]   = cfg.w_rate[i];
            }
            for (int j = 0; j < MAX_OBSTACLES; ++j) {
                const bool used               = j < int(obstacles.size());
                p[P::P_OBSTACLES + 2 * j]     = used ? obstacles[j].x() : 0.0;
                p[P::P_OBSTACLES + 2 * j + 1] = used ? obstacles[j].y() : 0.0;
                p[P::P_ACTIVE + j]            = used ? 1.0 : 0.0;
            }
        }
    }

    Solution InvertedWalkMPC::solve(const Eigen::Vector3d& target, const std::vector<Eigen::Vector2d>& all_obstacles) {
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
            a_guess[0].z()        = sign * 0.3 / DT;
        }

        // Roll the guess out from the current state
        std::vector<Vector6d> s_guess(N + 1);
        s_guess[0] << 0.0, 0.0, 0.0, v_prev;
        for (int k = 0; k < N; ++k) {
            s_guess[k + 1] = P::dynamics<double>(s_guess[k], a_guess[k], nullptr);
        }

        // Per-stage parameters: the heading fade-in and the travel bearing come from the guess, not the decision
        // variables. Were the fade-in a function of the optimised path, walking away from the target would make
        // heading errors cheaper, and the robot would swing out and back to turn on the spot.
        fill_shared_parameters(obstacles);
        for (int k = 0; k <= N; ++k) {
            const Eigen::Vector2d d = target.head<2>() - s_guess[k].head<2>();
            auto& p                 = params[k];
            p[P::P_TARGET]          = target.x();
            p[P::P_TARGET + 1]      = target.y();
            p[P::P_TARGET + 2]      = target.z();
            p[P::P_GATE]            = std::exp(-d.squaredNorm() / (cfg.heading_radius * cfg.heading_radius));
            p[P::P_BEARING]         = d.norm() > 1e-3 ? std::atan2(d.y(), d.x()) : target.z();
            ocp->set_parameters(k, p.data());
        }

        ocp_nlp_config* nlp_config = ocp->config;
        ocp_nlp_dims* nlp_dims     = ocp->dims;
        ocp_nlp_in* nlp_in         = ocp->in;
        ocp_nlp_out* nlp_out       = ocp->out;

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
        solution.status     = ocp->solve();
        solution.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        solution.iterations = ocp->iterations();

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
        solution.velocities.reserve(N);
        for (int k = 0; k <= N; ++k) {
            solution.states.emplace_back(s[k].head<3>());
            if (k > 0) {
                solution.velocities.emplace_back(s[k].tail<3>());
            }
        }

        if (!solution.success) {
            // Don't warm start from a failed solve; the caller decides what to send and reports it back
            a_guess.clear();
            return solution;
        }

        // The velocity planned for the first step, clipped so it is exactly within the limits despite the solver's
        // tolerance
        solution.velocity = clip_to_limits(s[1].tail<3>());
        a_guess           = a;
        v_prev            = solution.velocity;
        return solution;
    }

}  // namespace module::planning::inverted_walk_mpc
