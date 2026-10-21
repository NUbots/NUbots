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
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_ACADOSOCP_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_ACADOSOCP_HPP

#include <vector>

#include "OcpFunctions.hpp"
#include "acados/utils/external_function_generic.h"
#include "acados_c/ocp_nlp_interface.h"

namespace module::planning::inverted_walk_mpc::ocp {

    /// @brief The structure of the problem: what is fixed once the solver is created. Every number in it (bounds,
    /// constraint matrices, slack penalties, parameters) is set afterwards through the acados handles.
    struct Layout {
        /// @brief Steps in the horizon
        int N = 0;
        /// @brief Step length (s). The stage costs are scaled by it, the terminal cost isn't.
        double dt = 0.0;
        /// @brief State, input and per-stage parameter sizes
        int nx = 0;
        int nu = 0;
        int np = 0;
        /// @brief The states with bounds at stages 1..N. Stage 0 bounds the whole state, to pin it to the current one.
        std::vector<int> bounded_states{};
        /// @brief Soft linear constraints on the state, lg ≤ C·x ≤ ug, at every stage
        int ng = 0;
        /// @brief Soft nonlinear constraints, h(x, u) ≤ uh, at stages 1..N
        int nh = 0;
    };

    /// @brief An acados SQP solver (exact Hessian, HPIPM for the QPs, merit backtracking) for a problem with discrete
    /// dynamics, external costs, soft linear constraints on the state and soft nonlinear constraints, built from
    /// C++ functions rather than generated code. The slacks of a stage are ordered as acados orders them: the linear
    /// constraints', then the nonlinear constraints'.
    class AcadosOcp {
    public:
        AcadosOcp(const Layout& layout, const Functions& functions);
        ~AcadosOcp();
        AcadosOcp(const AcadosOcp&)            = delete;
        AcadosOcp& operator=(const AcadosOcp&) = delete;
        AcadosOcp(AcadosOcp&&)                 = delete;
        AcadosOcp& operator=(AcadosOcp&&)      = delete;

        /// @brief Sets the parameter vector of a stage
        void set_parameters(int stage, const double* p);
        /// @brief Sets the SQP iteration cap
        void set_max_iterations(int max_iterations);
        /// @brief Zeroes the iterates and the QP solver's memory, keeping every number set
        void reset();
        /// @brief Solves from the current iterates; returns acados's status
        int solve();
        /// @brief SQP iterations the last solve used
        [[nodiscard]] int iterations() const;

        // The acados handles, for setting bounds, constraints, slack penalties and iterates, and reading the solution
        ocp_nlp_config* config = nullptr;
        ocp_nlp_dims* dims     = nullptr;
        ocp_nlp_in* in         = nullptr;
        ocp_nlp_out* out       = nullptr;
        ocp_nlp_solver* solver = nullptr;
        void* opts             = nullptr;

    private:
        void release();

        Layout layout{};
        ocp_nlp_plan_t* plan = nullptr;
        /// @brief acados's wrappers of the functions, one per stage that uses it (each holds its stage's parameters).
        /// Allocated once: acados keeps pointers to them.
        std::vector<external_function_external_param_casadi> wrappers{};
    };

}  // namespace module::planning::inverted_walk_mpc::ocp

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_ACADOSOCP_HPP
