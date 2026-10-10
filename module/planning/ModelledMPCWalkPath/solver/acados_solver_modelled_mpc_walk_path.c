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

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h> // memcpy
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "modelled_mpc_walk_path_model/modelled_mpc_walk_path_model.h"


#include "modelled_mpc_walk_path_constraints/modelled_mpc_walk_path_constraints.h"
#include "modelled_mpc_walk_path_cost/modelled_mpc_walk_path_cost.h"



#include "acados_solver_modelled_mpc_walk_path.h"

#define NX     MODELLED_MPC_WALK_PATH_NX
#define NZ     MODELLED_MPC_WALK_PATH_NZ
#define NU     MODELLED_MPC_WALK_PATH_NU
#define NP     MODELLED_MPC_WALK_PATH_NP
#define NP_GLOBAL     MODELLED_MPC_WALK_PATH_NP_GLOBAL
#define NY0    MODELLED_MPC_WALK_PATH_NY0
#define NY     MODELLED_MPC_WALK_PATH_NY
#define NYN    MODELLED_MPC_WALK_PATH_NYN

#define NBX    MODELLED_MPC_WALK_PATH_NBX
#define NBX0   MODELLED_MPC_WALK_PATH_NBX0
#define NBU    MODELLED_MPC_WALK_PATH_NBU
#define NG     MODELLED_MPC_WALK_PATH_NG
#define NBXN   MODELLED_MPC_WALK_PATH_NBXN
#define NGN    MODELLED_MPC_WALK_PATH_NGN

#define NH     MODELLED_MPC_WALK_PATH_NH
#define NHN    MODELLED_MPC_WALK_PATH_NHN
#define NH0    MODELLED_MPC_WALK_PATH_NH0
#define NPHI   MODELLED_MPC_WALK_PATH_NPHI
#define NPHIN  MODELLED_MPC_WALK_PATH_NPHIN
#define NPHI0  MODELLED_MPC_WALK_PATH_NPHI0
#define NR     MODELLED_MPC_WALK_PATH_NR

#define NS     MODELLED_MPC_WALK_PATH_NS
#define NS0    MODELLED_MPC_WALK_PATH_NS0
#define NSN    MODELLED_MPC_WALK_PATH_NSN

#define NSBX   MODELLED_MPC_WALK_PATH_NSBX
#define NSBU   MODELLED_MPC_WALK_PATH_NSBU
#define NSH0   MODELLED_MPC_WALK_PATH_NSH0
#define NSH    MODELLED_MPC_WALK_PATH_NSH
#define NSHN   MODELLED_MPC_WALK_PATH_NSHN
#define NSG    MODELLED_MPC_WALK_PATH_NSG
#define NSPHI0 MODELLED_MPC_WALK_PATH_NSPHI0
#define NSPHI  MODELLED_MPC_WALK_PATH_NSPHI
#define NSPHIN MODELLED_MPC_WALK_PATH_NSPHIN
#define NSGN   MODELLED_MPC_WALK_PATH_NSGN
#define NSBXN  MODELLED_MPC_WALK_PATH_NSBXN





// ** solver data **

modelled_mpc_walk_path_solver_capsule * modelled_mpc_walk_path_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(modelled_mpc_walk_path_solver_capsule));
    modelled_mpc_walk_path_solver_capsule *capsule = (modelled_mpc_walk_path_solver_capsule *) capsule_mem;

    return capsule;
}


int modelled_mpc_walk_path_acados_free_capsule(modelled_mpc_walk_path_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int modelled_mpc_walk_path_acados_create(modelled_mpc_walk_path_solver_capsule* capsule)
{
    int N_shooting_intervals = MODELLED_MPC_WALK_PATH_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return modelled_mpc_walk_path_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int modelled_mpc_walk_path_acados_update_time_steps(modelled_mpc_walk_path_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "modelled_mpc_walk_path_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 1
 */
void modelled_mpc_walk_path_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = MIRROR;

    nlp_solver_plan->globalization = MERIT_BACKTRACKING;
}


static ocp_nlp_dims* modelled_mpc_walk_path_acados_create_setup_dimensions(modelled_mpc_walk_path_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 15;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 3
 */
void modelled_mpc_walk_path_acados_create_setup_functions(modelled_mpc_walk_path_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], modelled_mpc_walk_path_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], modelled_mpc_walk_path_constr_h_fun);
        }
        capsule->nl_constr_h_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac_hess[i], modelled_mpc_walk_path_constr_h_fun_jac_uxt_zt_hess);
        }
    
        // external cost
        MAP_CASADI_FNC(ext_cost_0_fun, modelled_mpc_walk_path_cost_ext_cost_0_fun);
        MAP_CASADI_FNC(ext_cost_0_fun_jac, modelled_mpc_walk_path_cost_ext_cost_0_fun_jac);
        MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, modelled_mpc_walk_path_cost_ext_cost_0_fun_jac_hess);



    
        // discrete dynamics
        capsule->discr_dyn_phi_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun[i], modelled_mpc_walk_path_dyn_disc_phi_fun);
        }

        capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], modelled_mpc_walk_path_dyn_disc_phi_fun_jac);
        }

    

    

    
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt_hess[i], modelled_mpc_walk_path_dyn_disc_phi_fun_jac_hess);
        }
        // external cost
        capsule->ext_cost_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun[i], modelled_mpc_walk_path_cost_ext_cost_fun);
        }

        capsule->ext_cost_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun_jac[i], modelled_mpc_walk_path_cost_ext_cost_fun_jac);
        }

        capsule->ext_cost_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], modelled_mpc_walk_path_cost_ext_cost_fun_jac_hess);
        }

        


        


        
    } // N > 0
    MAP_CASADI_FNC(nl_constr_h_e_fun_jac, modelled_mpc_walk_path_constr_h_e_fun_jac_uxt_zt);
    MAP_CASADI_FNC(nl_constr_h_e_fun, modelled_mpc_walk_path_constr_h_e_fun);
    MAP_CASADI_FNC(nl_constr_h_e_fun_jac_hess, modelled_mpc_walk_path_constr_h_e_fun_jac_uxt_zt_hess);
    
    
    
    
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, modelled_mpc_walk_path_cost_ext_cost_e_fun);

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, modelled_mpc_walk_path_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, modelled_mpc_walk_path_cost_ext_cost_e_fun_jac_hess);

    // external cost - jacobian wrt params
    

    

    

#undef MAP_CASADI_FNC
}


/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 5
 */
void modelled_mpc_walk_path_acados_create_set_default_parameters(modelled_mpc_walk_path_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;

    // initialize parameters to initial value
    
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        modelled_mpc_walk_path_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 5
 */
void modelled_mpc_walk_path_acados_create_setup_nlp_in_numerical_values(modelled_mpc_walk_path_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        modelled_mpc_walk_path_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.1;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.1;
        cost_scaling[1] = 0.1;
        cost_scaling[2] = 0.1;
        cost_scaling[3] = 0.1;
        cost_scaling[4] = 0.1;
        cost_scaling[5] = 0.1;
        cost_scaling[6] = 0.1;
        cost_scaling[7] = 0.1;
        cost_scaling[8] = 0.1;
        cost_scaling[9] = 0.1;
        cost_scaling[10] = 0.1;
        cost_scaling[11] = 0.1;
        cost_scaling[12] = 0.1;
        cost_scaling[13] = 0.1;
        cost_scaling[14] = 0.1;
        cost_scaling[15] = 0.1;
        cost_scaling[16] = 0.1;
        cost_scaling[17] = 0.1;
        cost_scaling[18] = 0.1;
        cost_scaling[19] = 0.1;
        cost_scaling[20] = 0.1;
        cost_scaling[21] = 0.1;
        cost_scaling[22] = 0.1;
        cost_scaling[23] = 0.1;
        cost_scaling[24] = 0.1;
        cost_scaling[25] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Cost ****/



    // slacks initial
    double* zlu0_mem = calloc(4*NS0, sizeof(double));
    double* Zl_0 = zlu0_mem+NS0*0;
    double* Zu_0 = zlu0_mem+NS0*1;
    double* zl_0 = zlu0_mem+NS0*2;
    double* zu_0 = zlu0_mem+NS0*3;

    // change only the non-zero elements:
    Zu_0[0] = 1;
    Zu_0[1] = 1;
    Zu_0[2] = 1;
    Zu_0[3] = 1;
    Zu_0[4] = 1;
    Zu_0[5] = 1;
    Zu_0[6] = 1;
    Zu_0[7] = 1;
    Zu_0[8] = 1;
    Zu_0[9] = 1;
    Zu_0[10] = 1;
    Zu_0[11] = 1;
    Zu_0[12] = 1;
    Zu_0[13] = 1;
    Zu_0[14] = 1;
    Zu_0[15] = 1;
    Zu_0[16] = 1;
    Zu_0[17] = 1;
    Zu_0[18] = 1;
    Zu_0[19] = 1;
    Zu_0[20] = 1;
    Zu_0[21] = 1;
    Zu_0[22] = 1;
    Zu_0[23] = 1;
    zu_0[0] = 1000;
    zu_0[1] = 1000;
    zu_0[2] = 1000;
    zu_0[3] = 1000;
    zu_0[4] = 1000;
    zu_0[5] = 1000;
    zu_0[6] = 1000;
    zu_0[7] = 1000;
    zu_0[8] = 1000;
    zu_0[9] = 1000;
    zu_0[10] = 1000;
    zu_0[11] = 1000;
    zu_0[12] = 1000;
    zu_0[13] = 1000;
    zu_0[14] = 1000;
    zu_0[15] = 1000;
    zu_0[16] = 1000;
    zu_0[17] = 1000;
    zu_0[18] = 1000;
    zu_0[19] = 1000;
    zu_0[20] = 1000;
    zu_0[21] = 1000;
    zu_0[22] = 1000;
    zu_0[23] = 1000;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Zl", Zl_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Zu", Zu_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "zl", zl_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "zu", zu_0);
    free(zlu0_mem);
    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:
    Zu[0] = 1;
    Zu[1] = 1;
    Zu[2] = 1;
    Zu[3] = 1;
    Zu[4] = 1;
    Zu[5] = 1;
    Zu[6] = 1;
    Zu[7] = 1;
    Zu[8] = 1;
    Zu[9] = 1;
    Zu[10] = 1;
    Zu[11] = 1;
    Zu[12] = 1;
    Zu[13] = 1;
    Zu[14] = 1;
    Zu[15] = 1;
    Zu[16] = 1;
    Zu[17] = 1;
    Zu[18] = 1;
    Zu[19] = 1;
    Zu[20] = 1;
    Zu[21] = 1;
    Zu[22] = 1;
    Zu[23] = 1;
    Zu[24] = 1;
    Zu[25] = 1;
    Zu[26] = 1;
    Zu[27] = 1;
    zu[0] = 1000;
    zu[1] = 1000;
    zu[2] = 1000;
    zu[3] = 1000;
    zu[4] = 1000;
    zu[5] = 1000;
    zu[6] = 1000;
    zu[7] = 1000;
    zu[8] = 1000;
    zu[9] = 1000;
    zu[10] = 1000;
    zu[11] = 1000;
    zu[12] = 1000;
    zu[13] = 1000;
    zu[14] = 1000;
    zu[15] = 1000;
    zu[16] = 1000;
    zu[17] = 1000;
    zu[18] = 1000;
    zu[19] = 1000;
    zu[20] = 1000;
    zu[21] = 1000;
    zu[22] = 1000;
    zu[23] = 1000;
    zu[24] = 1000;
    zu[25] = 1000;
    zu[26] = 1000;
    zu[27] = 1000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);


    // slacks terminal
    double* zluemem = calloc(4*NSN, sizeof(double));
    double* Zl_e = zluemem+NSN*0;
    double* Zu_e = zluemem+NSN*1;
    double* zl_e = zluemem+NSN*2;
    double* zu_e = zluemem+NSN*3;

    // change only the non-zero elements:
    Zu_e[0] = 1;
    Zu_e[1] = 1;
    Zu_e[2] = 1;
    Zu_e[3] = 1;
    Zu_e[4] = 1;
    Zu_e[5] = 1;
    Zu_e[6] = 1;
    Zu_e[7] = 1;
    Zu_e[8] = 1;
    Zu_e[9] = 1;
    Zu_e[10] = 1;
    Zu_e[11] = 1;
    Zu_e[12] = 1;
    Zu_e[13] = 1;
    Zu_e[14] = 1;
    Zu_e[15] = 1;
    Zu_e[16] = 1;
    Zu_e[17] = 1;
    Zu_e[18] = 1;
    Zu_e[19] = 1;
    Zu_e[20] = 1;
    Zu_e[21] = 1;
    Zu_e[22] = 1;
    Zu_e[23] = 1;
    Zu_e[24] = 1;
    Zu_e[25] = 1;
    Zu_e[26] = 1;
    Zu_e[27] = 1;
    zu_e[0] = 1000;
    zu_e[1] = 1000;
    zu_e[2] = 1000;
    zu_e[3] = 1000;
    zu_e[4] = 1000;
    zu_e[5] = 1000;
    zu_e[6] = 1000;
    zu_e[7] = 1000;
    zu_e[8] = 1000;
    zu_e[9] = 1000;
    zu_e[10] = 1000;
    zu_e[11] = 1000;
    zu_e[12] = 1000;
    zu_e[13] = 1000;
    zu_e[14] = 1000;
    zu_e[15] = 1000;
    zu_e[16] = 1000;
    zu_e[17] = 1000;
    zu_e[18] = 1000;
    zu_e[19] = 1000;
    zu_e[20] = 1000;
    zu_e[21] = 1000;
    zu_e[22] = 1000;
    zu_e[23] = 1000;
    zu_e[24] = 1000;
    zu_e[25] = 1000;
    zu_e[26] = 1000;
    zu_e[27] = 1000;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Zl", Zl_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Zu", Zu_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "zl", zl_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "zu", zu_e);
    free(zluemem);

    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(15 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);












    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -1;
    ubu[0] = 1;
    lbu[1] = -1;
    ubu[1] = 1;
    lbu[2] = -1;
    ubu[2] = 1;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);


    // set up general constraints for stage 0 to N-1
    double* D = calloc(NG*NU, sizeof(double));
    double* C = calloc(NG*NX, sizeof(double));
    double* lug = calloc(2*NG, sizeof(double));
    double* lg = lug;
    double* ug = lug + NG;
    C[0+NG * 12] = 1;
    C[1+NG * 13] = 1;
    C[2+NG * 14] = 1;
    C[3+NG * 12] = 1;
    C[4+NG * 13] = 1;
    C[5+NG * 14] = 1;
    C[6+NG * 12] = 1;
    C[7+NG * 13] = 1;
    C[8+NG * 14] = 1;
    C[9+NG * 12] = 1;
    C[10+NG * 13] = 1;
    C[11+NG * 14] = 1;
    C[12+NG * 12] = 1;
    C[13+NG * 13] = 1;
    C[14+NG * 14] = 1;
    C[15+NG * 12] = 1;
    C[16+NG * 13] = 1;
    C[17+NG * 14] = 1;
    C[18+NG * 12] = 1;
    C[19+NG * 13] = 1;
    C[20+NG * 14] = 1;
    C[21+NG * 12] = 1;
    C[22+NG * 13] = 1;
    C[23+NG * 14] = 1;
    lg[0] = -1000000000;
    lg[1] = -1000000000;
    lg[2] = -1000000000;
    lg[3] = -1000000000;
    lg[4] = -1000000000;
    lg[5] = -1000000000;
    lg[6] = -1000000000;
    lg[7] = -1000000000;
    lg[8] = -1000000000;
    lg[9] = -1000000000;
    lg[10] = -1000000000;
    lg[11] = -1000000000;
    lg[12] = -1000000000;
    lg[13] = -1000000000;
    lg[14] = -1000000000;
    lg[15] = -1000000000;
    lg[16] = -1000000000;
    lg[17] = -1000000000;
    lg[18] = -1000000000;
    lg[19] = -1000000000;
    lg[20] = -1000000000;
    lg[21] = -1000000000;
    lg[22] = -1000000000;
    lg[23] = -1000000000;
    ug[0] = 1000000000;
    ug[1] = 1000000000;
    ug[2] = 1000000000;
    ug[3] = 1000000000;
    ug[4] = 1000000000;
    ug[5] = 1000000000;
    ug[6] = 1000000000;
    ug[7] = 1000000000;
    ug[8] = 1000000000;
    ug[9] = 1000000000;
    ug[10] = 1000000000;
    ug[11] = 1000000000;
    ug[12] = 1000000000;
    ug[13] = 1000000000;
    ug[14] = 1000000000;
    ug[15] = 1000000000;
    ug[16] = 1000000000;
    ug[17] = 1000000000;
    ug[18] = 1000000000;
    ug[19] = 1000000000;
    ug[20] = 1000000000;
    ug[21] = 1000000000;
    ug[22] = 1000000000;
    ug[23] = 1000000000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "D", D);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "C", C);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lg", lg);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ug", ug);
    }
    free(D);
    free(C);
    free(lug);


    // set up soft bounds for general linear constraints
    int* idxsg = malloc(NSG * sizeof(int));
    idxsg[0] = 0;
    idxsg[1] = 1;
    idxsg[2] = 2;
    idxsg[3] = 3;
    idxsg[4] = 4;
    idxsg[5] = 5;
    idxsg[6] = 6;
    idxsg[7] = 7;
    idxsg[8] = 8;
    idxsg[9] = 9;
    idxsg[10] = 10;
    idxsg[11] = 11;
    idxsg[12] = 12;
    idxsg[13] = 13;
    idxsg[14] = 14;
    idxsg[15] = 15;
    idxsg[16] = 16;
    idxsg[17] = 17;
    idxsg[18] = 18;
    idxsg[19] = 19;
    idxsg[20] = 20;
    idxsg[21] = 21;
    idxsg[22] = 22;
    idxsg[23] = 23;
    double* lusg = calloc(2*NSG, sizeof(double));
    double* lsg = lusg;
    double* usg = lusg + NSG;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxsg", idxsg);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lsg", lsg);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "usg", usg);
    }
    free(idxsg);
    free(lusg);


    /* Path constraints */

    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 12;
    idxbx[1] = 13;
    idxbx[2] = 14;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = -1;
    ubx[0] = 1;
    lbx[1] = -1;
    ubx[1] = 1;
    lbx[2] = -1;
    ubx[2] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);


    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    lh[0] = -1000000000;
    lh[1] = -1000000000;
    lh[2] = -1000000000;
    lh[3] = -1000000000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
    }
    free(luh);








    // set up soft bounds for nonlinear constraints
    int* idxsh = malloc(NSH * sizeof(int));
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    double* lush = calloc(2*NSH, sizeof(double));
    double* lsh = lush;
    double* ush = lush + NSH;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ush", ush);
    }
    free(idxsh);
    free(lush);



    /* terminal constraints */

    // set up bounds for last stage
    // x
    int* idxbx_e = malloc(NBXN * sizeof(int));
    idxbx_e[0] = 12;
    idxbx_e[1] = 13;
    idxbx_e[2] = 14;
    double* lubx_e = calloc(2*NBXN, sizeof(double));
    double* lbx_e = lubx_e;
    double* ubx_e = lubx_e + NBXN;
    lbx_e[0] = -1;
    ubx_e[0] = 1;
    lbx_e[1] = -1;
    ubx_e[1] = 1;
    lbx_e[2] = -1;
    ubx_e[2] = 1;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ubx", ubx_e);
    free(idxbx_e);
    free(lubx_e);


    // set up general constraints for last stage
    double* C_e = calloc(NGN*NX, sizeof(double));
    double* lug_e = calloc(2*NGN, sizeof(double));
    double* lg_e = lug_e;
    double* ug_e = lug_e + NGN;
    C_e[0+NGN * 12] = 1;
    C_e[1+NGN * 13] = 1;
    C_e[2+NGN * 14] = 1;
    C_e[3+NGN * 12] = 1;
    C_e[4+NGN * 13] = 1;
    C_e[5+NGN * 14] = 1;
    C_e[6+NGN * 12] = 1;
    C_e[7+NGN * 13] = 1;
    C_e[8+NGN * 14] = 1;
    C_e[9+NGN * 12] = 1;
    C_e[10+NGN * 13] = 1;
    C_e[11+NGN * 14] = 1;
    C_e[12+NGN * 12] = 1;
    C_e[13+NGN * 13] = 1;
    C_e[14+NGN * 14] = 1;
    C_e[15+NGN * 12] = 1;
    C_e[16+NGN * 13] = 1;
    C_e[17+NGN * 14] = 1;
    C_e[18+NGN * 12] = 1;
    C_e[19+NGN * 13] = 1;
    C_e[20+NGN * 14] = 1;
    C_e[21+NGN * 12] = 1;
    C_e[22+NGN * 13] = 1;
    C_e[23+NGN * 14] = 1;
    lg_e[0] = -1000000000;
    ug_e[0] = 1000000000;
    lg_e[1] = -1000000000;
    ug_e[1] = 1000000000;
    lg_e[2] = -1000000000;
    ug_e[2] = 1000000000;
    lg_e[3] = -1000000000;
    ug_e[3] = 1000000000;
    lg_e[4] = -1000000000;
    ug_e[4] = 1000000000;
    lg_e[5] = -1000000000;
    ug_e[5] = 1000000000;
    lg_e[6] = -1000000000;
    ug_e[6] = 1000000000;
    lg_e[7] = -1000000000;
    ug_e[7] = 1000000000;
    lg_e[8] = -1000000000;
    ug_e[8] = 1000000000;
    lg_e[9] = -1000000000;
    ug_e[9] = 1000000000;
    lg_e[10] = -1000000000;
    ug_e[10] = 1000000000;
    lg_e[11] = -1000000000;
    ug_e[11] = 1000000000;
    lg_e[12] = -1000000000;
    ug_e[12] = 1000000000;
    lg_e[13] = -1000000000;
    ug_e[13] = 1000000000;
    lg_e[14] = -1000000000;
    ug_e[14] = 1000000000;
    lg_e[15] = -1000000000;
    ug_e[15] = 1000000000;
    lg_e[16] = -1000000000;
    ug_e[16] = 1000000000;
    lg_e[17] = -1000000000;
    ug_e[17] = 1000000000;
    lg_e[18] = -1000000000;
    ug_e[18] = 1000000000;
    lg_e[19] = -1000000000;
    ug_e[19] = 1000000000;
    lg_e[20] = -1000000000;
    ug_e[20] = 1000000000;
    lg_e[21] = -1000000000;
    ug_e[21] = 1000000000;
    lg_e[22] = -1000000000;
    ug_e[22] = 1000000000;
    lg_e[23] = -1000000000;
    ug_e[23] = 1000000000;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "C", C_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lg", lg_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ug", ug_e);
    free(C_e);
    free(lug_e);


    // set up nonlinear constraints for last stage
    double* luh_e = calloc(2*NHN, sizeof(double));
    double* lh_e = luh_e;
    double* uh_e = luh_e + NHN;
    lh_e[0] = -1000000000;
    lh_e[1] = -1000000000;
    lh_e[2] = -1000000000;
    lh_e[3] = -1000000000;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "uh", uh_e);
    free(luh_e);



    /* terminal soft constraints */






    // set up soft bounds for general linear constraints
    int* idxsg_e = calloc(NSGN, sizeof(int));
    idxsg_e[0] = 0;
    idxsg_e[1] = 1;
    idxsg_e[2] = 2;
    idxsg_e[3] = 3;
    idxsg_e[4] = 4;
    idxsg_e[5] = 5;
    idxsg_e[6] = 6;
    idxsg_e[7] = 7;
    idxsg_e[8] = 8;
    idxsg_e[9] = 9;
    idxsg_e[10] = 10;
    idxsg_e[11] = 11;
    idxsg_e[12] = 12;
    idxsg_e[13] = 13;
    idxsg_e[14] = 14;
    idxsg_e[15] = 15;
    idxsg_e[16] = 16;
    idxsg_e[17] = 17;
    idxsg_e[18] = 18;
    idxsg_e[19] = 19;
    idxsg_e[20] = 20;
    idxsg_e[21] = 21;
    idxsg_e[22] = 22;
    idxsg_e[23] = 23;
    double* lusg_e = calloc(2*NSGN, sizeof(double));
    double* lsg_e = lusg_e;
    double* usg_e = lusg_e + NSGN;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "idxsg", idxsg_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lsg", lsg_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "usg", usg_e);
    free(idxsg_e);
    free(lusg_e);


    // set up soft bounds for nonlinear constraints
    int* idxsh_e = malloc(NSHN * sizeof(int));
    idxsh_e[0] = 0;
    idxsh_e[1] = 1;
    idxsh_e[2] = 2;
    idxsh_e[3] = 3;
    double* lush_e = calloc(2*NSHN, sizeof(double));
    double* lsh_e = lush_e;
    double* ush_e = lush_e + NSHN;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "idxsh", idxsh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lsh", lsh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ush", ush_e);
    free(idxsh_e);
    free(lush_e);





}

// this function only sets external functions, numerical values are set in modelled_mpc_walk_path_acados_create_setup_nlp_in_numerical_values
void modelled_mpc_walk_path_acados_create_setup_nlp_in(modelled_mpc_walk_path_solver_capsule* capsule, const int N)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
        
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac_hess",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    
    
    
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
        
        
        
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);
    
    
    

    /**** Constraints ****/

    // bounds for initial stage



    // set up nonlinear constraints for stage 1 to N-1
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i,
                                      "nl_constr_h_fun_jac_hess", &capsule->nl_constr_h_fun_jac_hess[i-1]);
        
        
      
        
    }



    /* terminal constraints */

    // set up nonlinear constraints for last stage

    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac", &capsule->nl_constr_h_e_fun_jac);
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun", &capsule->nl_constr_h_e_fun);
    
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac_hess",
                                  &capsule->nl_constr_h_e_fun_jac_hess);
    
    
  
    }


static void modelled_mpc_walk_path_acados_create_set_opts(modelled_mpc_walk_path_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);

    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    double globalization_alpha_min = 0.05;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_min", &globalization_alpha_min);

    double globalization_alpha_reduction = 0.7;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_reduction", &globalization_alpha_reduction);



    int globalization_line_search_use_sufficient_descent = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_line_search_use_sufficient_descent", &globalization_line_search_use_sufficient_descent);

    int globalization_use_SOC = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_use_SOC", &globalization_use_SOC);

    double globalization_eps_sufficient_descent = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_eps_sufficient_descent", &globalization_eps_sufficient_descent);

    int with_solution_sens_wrt_params_forw = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_forw", &with_solution_sens_wrt_params_forw);

    int with_solution_sens_wrt_params_adj = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_adj", &with_solution_sens_wrt_params_adj);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 25;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);
    double reg_epsilon = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_epsilon", &reg_epsilon);
    double reg_max_cond_block = 10000000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_max_cond_block", &reg_max_cond_block);

    double reg_min_epsilon = 0.00000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_min_epsilon", &reg_min_epsilon);

    bool reg_adaptive_eps = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_adaptive_eps", &reg_adaptive_eps);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    // NLP QP tol strategy
    ocp_nlp_qp_tol_strategy_t nlp_qp_tol_strategy = FIXED_QP_TOL;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_strategy", &nlp_qp_tol_strategy);

    double nlp_qp_tol_reduction_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_reduction_factor", &nlp_qp_tol_reduction_factor);

    double nlp_qp_tol_safety_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_safety_factor", &nlp_qp_tol_safety_factor);

    double nlp_qp_tol_min_stat = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_stat", &nlp_qp_tol_min_stat);

    double nlp_qp_tol_min_eq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_eq", &nlp_qp_tol_min_eq);

    double nlp_qp_tol_min_ineq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_ineq", &nlp_qp_tol_min_ineq);

    double nlp_qp_tol_min_comp = 0.00000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_comp", &nlp_qp_tol_min_comp);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}


/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 7
 */
void modelled_mpc_walk_path_acados_set_nlp_out(modelled_mpc_walk_path_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for modelled_mpc_walk_path_acados_create: step 9
 */
int modelled_mpc_walk_path_acados_create_precompute(modelled_mpc_walk_path_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int modelled_mpc_walk_path_acados_create_with_discretization(modelled_mpc_walk_path_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != MODELLED_MPC_WALK_PATH_N && !new_time_steps) {
        fprintf(stderr, "modelled_mpc_walk_path_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, MODELLED_MPC_WALK_PATH_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    modelled_mpc_walk_path_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = modelled_mpc_walk_path_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    modelled_mpc_walk_path_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    modelled_mpc_walk_path_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    modelled_mpc_walk_path_acados_create_setup_functions(capsule);
    modelled_mpc_walk_path_acados_create_setup_nlp_in(capsule, N);
    modelled_mpc_walk_path_acados_create_setup_nlp_in_numerical_values(capsule, N, new_time_steps);
    modelled_mpc_walk_path_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = modelled_mpc_walk_path_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int modelled_mpc_walk_path_acados_update_qp_solver_cond_N(modelled_mpc_walk_path_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from modelled_mpc_walk_path_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = modelled_mpc_walk_path_acados_create_precompute(capsule);
    return status;
}


int modelled_mpc_walk_path_acados_reset(modelled_mpc_walk_path_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar)
{
    // set initialization to all zeros
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    // sets primal and dual iterates to zero
    ocp_nlp_out_set_values_to_zero(nlp_config, nlp_dims, nlp_out);

    // reset integrator memory
    ocp_nlp_solver_reset_integrator_memory(nlp_solver, nlp_in, nlp_out);
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    if (reset_numerical_values)
    {
        // reset parameters to initial values
        modelled_mpc_walk_path_acados_create_set_default_parameters(capsule);

        // reset numerical values in nlp_in
        modelled_mpc_walk_path_acados_create_setup_nlp_in_numerical_values(capsule, N, NULL);
    }

    if (reset_solver_options)
    {
        // reset solver options to initial values
        modelled_mpc_walk_path_acados_create_set_opts(capsule);
    }

    if (reset_x_to_x0_bar)
    {double* buffer = calloc(NX, sizeof(double));
        ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
        for (int i=0; i<N+1; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        }
        free(buffer);
    }
    return 0;
}




int modelled_mpc_walk_path_acados_update_params(modelled_mpc_walk_path_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 175;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int modelled_mpc_walk_path_acados_update_params_sparse(modelled_mpc_walk_path_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int modelled_mpc_walk_path_acados_set_p_global_and_precompute_dependencies(modelled_mpc_walk_path_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, modelled_mpc_walk_path_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int modelled_mpc_walk_path_acados_solve(modelled_mpc_walk_path_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int modelled_mpc_walk_path_acados_setup_qp_matrices_and_factorize(modelled_mpc_walk_path_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int modelled_mpc_walk_path_acados_free(modelled_mpc_walk_path_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        

        
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
  
  
  
    free(capsule->discr_dyn_phi_fun_jac_ut_xt_hess);

    // cost
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    
    
    
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
        
        
        
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);
    
    
    

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac_hess[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    free(capsule->nl_constr_h_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun_jac);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun_jac_hess);



    return 0;
}


void modelled_mpc_walk_path_acados_print_stats(modelled_mpc_walk_path_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[176];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int modelled_mpc_walk_path_acados_custom_update(modelled_mpc_walk_path_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *modelled_mpc_walk_path_acados_get_nlp_in(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *modelled_mpc_walk_path_acados_get_nlp_out(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *modelled_mpc_walk_path_acados_get_sens_out(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *modelled_mpc_walk_path_acados_get_nlp_solver(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *modelled_mpc_walk_path_acados_get_nlp_config(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_config; }
void *modelled_mpc_walk_path_acados_get_nlp_opts(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *modelled_mpc_walk_path_acados_get_nlp_dims(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *modelled_mpc_walk_path_acados_get_nlp_plan(modelled_mpc_walk_path_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
