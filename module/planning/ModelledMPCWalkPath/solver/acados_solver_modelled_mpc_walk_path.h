/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_modelled_mpc_walk_path_H_
#define ACADOS_SOLVER_modelled_mpc_walk_path_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define MODELLED_MPC_WALK_PATH_NX     15
#define MODELLED_MPC_WALK_PATH_NZ     0
#define MODELLED_MPC_WALK_PATH_NU     3
#define MODELLED_MPC_WALK_PATH_NP     175
#define MODELLED_MPC_WALK_PATH_NP_GLOBAL     0
#define MODELLED_MPC_WALK_PATH_NBX    3
#define MODELLED_MPC_WALK_PATH_NBX0   15
#define MODELLED_MPC_WALK_PATH_NBU    3
#define MODELLED_MPC_WALK_PATH_NSBX   0
#define MODELLED_MPC_WALK_PATH_NSBU   0
#define MODELLED_MPC_WALK_PATH_NSH    4
#define MODELLED_MPC_WALK_PATH_NSH0   0
#define MODELLED_MPC_WALK_PATH_NSG    24
#define MODELLED_MPC_WALK_PATH_NSPHI  0
#define MODELLED_MPC_WALK_PATH_NSHN   4
#define MODELLED_MPC_WALK_PATH_NSGN   24
#define MODELLED_MPC_WALK_PATH_NSPHIN 0
#define MODELLED_MPC_WALK_PATH_NSPHI0 0
#define MODELLED_MPC_WALK_PATH_NSBXN  0
#define MODELLED_MPC_WALK_PATH_NS     28
#define MODELLED_MPC_WALK_PATH_NS0    24
#define MODELLED_MPC_WALK_PATH_NSN    28
#define MODELLED_MPC_WALK_PATH_NG     24
#define MODELLED_MPC_WALK_PATH_NBXN   3
#define MODELLED_MPC_WALK_PATH_NGN    24
#define MODELLED_MPC_WALK_PATH_NY0    0
#define MODELLED_MPC_WALK_PATH_NY     0
#define MODELLED_MPC_WALK_PATH_NYN    0
#define MODELLED_MPC_WALK_PATH_N      25
#define MODELLED_MPC_WALK_PATH_NH     4
#define MODELLED_MPC_WALK_PATH_NHN    4
#define MODELLED_MPC_WALK_PATH_NH0    0
#define MODELLED_MPC_WALK_PATH_NPHI0  0
#define MODELLED_MPC_WALK_PATH_NPHI   0
#define MODELLED_MPC_WALK_PATH_NPHIN  0
#define MODELLED_MPC_WALK_PATH_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct modelled_mpc_walk_path_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *discr_dyn_phi_fun;
    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt;



    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt_hess;


    // cost

    external_function_external_param_casadi *ext_cost_fun;
    external_function_external_param_casadi *ext_cost_fun_jac;
    external_function_external_param_casadi *ext_cost_fun_jac_hess;






    external_function_external_param_casadi ext_cost_0_fun;
    external_function_external_param_casadi ext_cost_0_fun_jac;
    external_function_external_param_casadi ext_cost_0_fun_jac_hess;





    external_function_external_param_casadi ext_cost_e_fun;
    external_function_external_param_casadi ext_cost_e_fun_jac;
    external_function_external_param_casadi ext_cost_e_fun_jac_hess;




    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;
    external_function_external_param_casadi *nl_constr_h_fun_jac_hess;









    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;
    external_function_external_param_casadi nl_constr_h_e_fun_jac_hess;




} modelled_mpc_walk_path_solver_capsule;

ACADOS_SYMBOL_EXPORT modelled_mpc_walk_path_solver_capsule * modelled_mpc_walk_path_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_free_capsule(modelled_mpc_walk_path_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_create(modelled_mpc_walk_path_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_reset(modelled_mpc_walk_path_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar);

/**
 * Generic version of modelled_mpc_walk_path_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_create_with_discretization(modelled_mpc_walk_path_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_update_time_steps(modelled_mpc_walk_path_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_update_qp_solver_cond_N(modelled_mpc_walk_path_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_update_params(modelled_mpc_walk_path_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_update_params_sparse(modelled_mpc_walk_path_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_set_p_global_and_precompute_dependencies(modelled_mpc_walk_path_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_solve(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_setup_qp_matrices_and_factorize(modelled_mpc_walk_path_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_free(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void modelled_mpc_walk_path_acados_print_stats(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int modelled_mpc_walk_path_acados_custom_update(modelled_mpc_walk_path_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *modelled_mpc_walk_path_acados_get_nlp_in(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *modelled_mpc_walk_path_acados_get_nlp_out(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *modelled_mpc_walk_path_acados_get_sens_out(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *modelled_mpc_walk_path_acados_get_nlp_solver(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *modelled_mpc_walk_path_acados_get_nlp_config(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *modelled_mpc_walk_path_acados_get_nlp_opts(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *modelled_mpc_walk_path_acados_get_nlp_dims(modelled_mpc_walk_path_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *modelled_mpc_walk_path_acados_get_nlp_plan(modelled_mpc_walk_path_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_modelled_mpc_walk_path_H_
