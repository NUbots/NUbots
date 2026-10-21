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
#include "AcadosOcp.hpp"

#include <numeric>
#include <stdexcept>

#include "acados_c/external_function_interface.h"

namespace utility::ocp {

    namespace {

        /// @brief Bound used for constraints that are switched off
        constexpr double BIG = 1e9;

        /// @brief acados's default linear slack penalty, until the caller sets its own
        constexpr double DEFAULT_SLACK_PENALTY = 1000.0;

        /// @brief Wraps one of our functions in acados's wrapper for CasADi-style functions
        void wrap(external_function_external_param_casadi& wrapper, const ExternalFunction& f) {
            wrapper                     = {};
            wrapper.casadi_fun          = f.fun;
            wrapper.casadi_work         = f.work;
            wrapper.casadi_sparsity_in  = f.sparsity_in;
            wrapper.casadi_sparsity_out = f.sparsity_out;
            wrapper.casadi_n_in         = f.n_in;
            wrapper.casadi_n_out        = f.n_out;
            external_function_opts opts;
            external_function_opts_set_to_default(&opts);
            opts.external_workspace = true;
            external_function_external_param_casadi_create(&wrapper, &opts);
        }

        std::vector<int> iota(const int n) {
            std::vector<int> v(n);
            std::iota(v.begin(), v.end(), 0);
            return v;
        }

    }  // namespace

    AcadosOcp::AcadosOcp(const Layout& layout_, const Functions& functions) : layout(layout_) {
        const int N  = layout.N;
        const int nx = layout.nx;
        const int nb = int(layout.bounded_states.size());
        const int ng = layout.ng;
        const int nh = layout.nh;

        // The plan: SQP with an exact Hessian, HPIPM, mirrored regularisation and merit backtracking (without which
        // SQP stalls on near-target approaches)
        plan                                       = ocp_nlp_plan_create(N);
        plan->nlp_solver                           = SQP;
        plan->ocp_qp_solver_plan.qp_solver         = PARTIAL_CONDENSING_HPIPM;
        plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
        plan->regularization                       = MIRROR;
        plan->globalization                        = MERIT_BACKTRACKING;
        for (int k = 0; k <= N; ++k) {
            plan->nlp_cost[k]        = EXTERNAL;
            plan->nlp_constraints[k] = BGH;
        }
        for (int k = 0; k < N; ++k) {
            plan->nlp_dynamics[k]               = DISCRETE_MODEL;
            plan->sim_solver_plan[k].sim_solver = INVALID_SIM_SOLVER;
        }
        config = ocp_nlp_config_create(*plan);

        // Dimensions. Stage 0 bounds the whole state (pinned to the current one, as equalities) and has no nonlinear
        // constraints; stage N has no input.
        dims = ocp_nlp_dims_create(config);
        std::vector<int> v_nx(N + 1, nx), v_nu(N + 1, layout.nu), v_nz(N + 1, 0), v_np(N + 1, layout.np);
        std::vector<int> v_ns(N + 1, ng + nh);
        v_nu[N] = 0;
        v_ns[0] = ng;
        ocp_nlp_dims_set_opt_vars(config, dims, "nx", v_nx.data());
        ocp_nlp_dims_set_opt_vars(config, dims, "nu", v_nu.data());
        ocp_nlp_dims_set_opt_vars(config, dims, "nz", v_nz.data());
        ocp_nlp_dims_set_opt_vars(config, dims, "ns", v_ns.data());
        ocp_nlp_dims_set_opt_vars(config, dims, "np", v_np.data());
        ocp_nlp_dims_set_global(config, dims, "np_global", 0);
        ocp_nlp_dims_set_global(config, dims, "n_global_data", 0);
        for (int k = 0; k <= N; ++k) {
            int nbx   = k == 0 ? nx : nb;
            int nbxe  = k == 0 ? nx : 0;
            int nh_k  = k == 0 ? 0 : nh;
            int zero  = 0;
            int ng_k  = ng;
            int nsg_k = ng;
            int nsh_k = nh_k;
            ocp_nlp_dims_set_constraints(config, dims, k, "nbx", &nbx);
            ocp_nlp_dims_set_constraints(config, dims, k, "nbu", &zero);
            ocp_nlp_dims_set_constraints(config, dims, k, "nsbx", &zero);
            ocp_nlp_dims_set_constraints(config, dims, k, "nsbu", &zero);
            ocp_nlp_dims_set_constraints(config, dims, k, "ng", &ng_k);
            ocp_nlp_dims_set_constraints(config, dims, k, "nsg", &nsg_k);
            ocp_nlp_dims_set_constraints(config, dims, k, "nbxe", &nbxe);
            ocp_nlp_dims_set_constraints(config, dims, k, "nh", &nh_k);
            ocp_nlp_dims_set_constraints(config, dims, k, "nsh", &nsh_k);
        }

        // Solver options, as acados_template sets them for this plan
        opts           = ocp_nlp_solver_opts_create(config, dims);
        const auto set = [&](const char* field, auto value) { ocp_nlp_solver_opts_set(config, opts, field, &value); };
        set("exact_hess", 1);
        set("exact_hess_dyn", 1);
        set("exact_hess_cost", 1);
        set("exact_hess_constr", 1);
        set("fixed_hess", 0);
        set("globalization_alpha_min", 0.05);
        set("globalization_alpha_reduction", 0.7);
        set("globalization_line_search_use_sufficient_descent", 0);
        set("globalization_use_SOC", 0);
        set("globalization_eps_sufficient_descent", 1e-4);
        set("globalization_full_step_dual", 0);
        set("levenberg_marquardt", 0.0);
        set("qp_cond_N", N);
        set("reg_epsilon", 1e-4);
        set("reg_max_cond_block", 1e7);
        set("reg_min_epsilon", 1e-8);
        set("reg_adaptive_eps", false);
        set("tol_min_step_norm", 0.0);
        ocp_nlp_solver_opts_set(config, opts, "qp_hpipm_mode", const_cast<char*>("BALANCE"));
        set("qp_t0_init", 2);
        set("tol_stat", 1e-6);
        set("tol_eq", 1e-6);
        set("tol_ineq", 1e-6);
        set("tol_comp", 1e-6);
        set("max_iter", 10);
        set("qp_iter_max", 50);
        set("print_level", 0);
        set("qp_cond_ric_alg", 1);
        set("qp_ric_alg", 1);
        for (int k = 0; k <= N; ++k) {
            int numerical_hessian = 0;
            ocp_nlp_solver_opts_set_at_stage(config, opts, k, "cost_numerical_hessian", &numerical_hessian);
        }

        out = ocp_nlp_out_create(config, dims);
        ocp_nlp_out_set_values_to_zero(config, dims, out);
        in = ocp_nlp_in_create(config, dims);

        // The functions: the dynamics at stages 0..N-1, the stage cost at 0..N-1, the nonlinear constraints at
        // 1..N-1, and the terminal cost and constraints at N
        wrappers.resize(3 * (N + N + (N - 1) + 2));
        auto next           = wrappers.begin();
        const auto bind_set = [&](const FunctionSet& set) {
            auto* first = &*next;
            wrap(*next++, set.value);
            wrap(*next++, set.jacobian);
            wrap(*next++, set.hessian);
            return first;
        };
        for (int k = 0; k < N; ++k) {
            auto* f = bind_set(functions.dynamics);
            ocp_nlp_dynamics_model_set_external_param_fun(config, dims, in, k, "disc_dyn_fun", &f[0]);
            ocp_nlp_dynamics_model_set_external_param_fun(config, dims, in, k, "disc_dyn_fun_jac", &f[1]);
            ocp_nlp_dynamics_model_set_external_param_fun(config, dims, in, k, "disc_dyn_fun_jac_hess", &f[2]);
        }
        for (int k = 0; k <= N; ++k) {
            auto* f = bind_set(k < N ? functions.cost : functions.terminal_cost);
            ocp_nlp_cost_model_set_external_param_fun(config, dims, in, k, "ext_cost_fun", &f[0]);
            ocp_nlp_cost_model_set_external_param_fun(config, dims, in, k, "ext_cost_fun_jac", &f[1]);
            ocp_nlp_cost_model_set_external_param_fun(config, dims, in, k, "ext_cost_fun_jac_hess", &f[2]);
        }
        for (int k = 1; k <= N; ++k) {
            auto* f = bind_set(k < N ? functions.constraint : functions.terminal_constraint);
            ocp_nlp_constraints_model_set_external_param_fun(config, dims, in, k, "nl_constr_h_fun", &f[0]);
            ocp_nlp_constraints_model_set_external_param_fun(config, dims, in, k, "nl_constr_h_fun_jac", &f[1]);
            ocp_nlp_constraints_model_set_external_param_fun(config, dims, in, k, "nl_constr_h_fun_jac_hess", &f[2]);
        }

        // Step lengths, and the cost scaling that goes with them (the terminal cost isn't scaled)
        for (int k = 0; k <= N; ++k) {
            double scaling = k < N ? layout.dt : 1.0;
            if (k < N) {
                ocp_nlp_in_set(config, dims, in, k, "Ts", &layout.dt);
            }
            ocp_nlp_cost_model_set(config, dims, in, k, "scaling", &scaling);
        }

        // Slacks: an L1 penalty on the upper side and a unit L2 one, no penalty on the lower side (only the upper side
        // of the soft constraints is used)
        for (int k = 0; k <= N; ++k) {
            const int ns = v_ns[k];
            if (ns == 0) {
                continue;  // acados refuses the null pointers empty vectors give
            }
            std::vector<double> zeros(ns, 0.0), ones(ns, 1.0), z(ns, DEFAULT_SLACK_PENALTY);
            ocp_nlp_cost_model_set(config, dims, in, k, "Zl", zeros.data());
            ocp_nlp_cost_model_set(config, dims, in, k, "Zu", ones.data());
            ocp_nlp_cost_model_set(config, dims, in, k, "zl", zeros.data());
            ocp_nlp_cost_model_set(config, dims, in, k, "zu", z.data());
        }

        // Constraints, with placeholder bounds that the caller replaces
        std::vector<int> idxbx0 = iota(nx);
        std::vector<double> x0(nx, 0.0);
        ocp_nlp_constraints_model_set(config, dims, in, out, 0, "idxbx", idxbx0.data());
        ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", x0.data());
        ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", x0.data());
        ocp_nlp_constraints_model_set(config, dims, in, out, 0, "idxbxe", idxbx0.data());

        std::vector<int> idxbx = layout.bounded_states;
        std::vector<double> lbx(nb, -BIG), ubx(nb, BIG);
        std::vector<double> C(ng * nx, 0.0), D(ng * layout.nu, 0.0), lg(ng, -BIG), ug(ng, BIG);
        std::vector<int> idxsg = iota(ng);
        std::vector<double> lh(nh, -BIG), uh(nh, 0.0);
        std::vector<int> idxsh = iota(nh);
        for (int k = 0; k <= N; ++k) {
            // acados refuses null pointers, which empty vectors give, so constraints a stage doesn't have are skipped
            if (k > 0 && nb > 0) {
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "idxbx", idxbx.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "lbx", lbx.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "ubx", ubx.data());
            }
            if (k > 0 && nh > 0) {
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "lh", lh.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "uh", uh.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "idxsh", idxsh.data());
            }
            if (ng > 0) {
                if (k < N) {
                    ocp_nlp_constraints_model_set(config, dims, in, out, k, "D", D.data());
                }
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "C", C.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "lg", lg.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "ug", ug.data());
                ocp_nlp_constraints_model_set(config, dims, in, out, k, "idxsg", idxsg.data());
            }
        }

        std::vector<double> p(layout.np, 0.0);
        for (int k = 0; k <= N; ++k) {
            set_parameters(k, p.data());
        }

        solver = ocp_nlp_solver_create(config, dims, opts, in);
        if (solver == nullptr || ocp_nlp_precompute(solver, in, out) != ACADOS_SUCCESS) {
            release();
            throw std::runtime_error("AcadosOcp: creating the acados solver failed");
        }
    }

    AcadosOcp::~AcadosOcp() {
        release();
    }

    void AcadosOcp::release() {
        if (solver != nullptr) {
            ocp_nlp_solver_destroy(solver);
        }
        if (in != nullptr) {
            ocp_nlp_in_destroy(in);
        }
        if (out != nullptr) {
            ocp_nlp_out_destroy(out);
        }
        if (opts != nullptr) {
            ocp_nlp_solver_opts_destroy(opts);
        }
        if (dims != nullptr) {
            ocp_nlp_dims_destroy(dims);
        }
        if (config != nullptr) {
            ocp_nlp_config_destroy(config);
        }
        if (plan != nullptr) {
            ocp_nlp_plan_destroy(plan);
        }
        for (auto& w : wrappers) {
            external_function_external_param_casadi_free(&w);
        }
        wrappers.clear();
        solver = nullptr;
        in     = nullptr;
        out    = nullptr;
        opts   = nullptr;
        dims   = nullptr;
        config = nullptr;
        plan   = nullptr;
    }

    void AcadosOcp::set_parameters(const int stage, const double* p) {
        ocp_nlp_in_set(config, dims, in, stage, "parameter_values", const_cast<double*>(p));
    }

    void AcadosOcp::set_max_iterations(int max_iterations) {
        ocp_nlp_solver_opts_set(config, opts, "max_iter", &max_iterations);
    }

    void AcadosOcp::reset() {
        ocp_nlp_out_set_values_to_zero(config, dims, out);
        ocp_nlp_solver_reset_qp_memory(solver, in, out);
    }

    int AcadosOcp::solve() {
        return ocp_nlp_solve(solver, in, out);
    }

    int AcadosOcp::iterations() const {
        int iterations = 0;
        ocp_nlp_get(solver, "sqp_iter", &iterations);
        return iterations;
    }

}  // namespace utility::ocp
