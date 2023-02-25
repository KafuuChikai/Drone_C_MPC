#include "drone_test.h"

int main()
{
    // create a capsule according to the pre-defined model
    drone_solver_capsule *acados_ocp_capsule = drone_acados_create_capsule();

    // optimizer
    status = drone_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("drone_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    sim_solver_capsule *sim_capsule = drone_acados_sim_solver_create_capsule();
    status = drone_acados_sim_create(sim_capsule);
    sim_config *drone_sim_config = drone_acados_get_sim_config(sim_capsule);
    void *drone_sim_dims = drone_acados_get_sim_dims(sim_capsule);
    sim_in *drone_sim_in = drone_acados_get_sim_in(sim_capsule);
    sim_out *drone_sim_out = drone_acados_get_sim_out(sim_capsule);

    if (status)
    {
        printf("acados_create() simulator returned status %d. Exiting.\n", status);
        exit(1);
    }

    // some important structure of ocp
    ocp_nlp_config *nlp_config = drone_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = drone_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = drone_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = drone_acados_get_nlp_out(acados_ocp_capsule);
    //    ocp_nlp_solver *nlp_solver = mobile_robot_acados_get_nlp_solver(acados_ocp_capsule);
    //    void *nlp_opts = mobile_robot_acados_get_nlp_opts(acados_ocp_capsule);

    //T = *nlp_dims->T;
    N = nlp_dims->N;
    nx = *nlp_dims->nx;
    nu = *nlp_dims->nu;
    printf("time horizion is 1s, n nodes are %d with state %d and input %d \n", N, nx, nu);

    Eigen::MatrixXd simX((N + 1), nx);
    Eigen::MatrixXd simU(N, nu);
    Eigen::VectorXd time_record(N);

    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = -5.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 1.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;

    x_target[0] = 1.0;
    x_target[1] = 1.0;
    x_target[2] = -5.0;
    x_target[3] = 0.0;
    x_target[4] = 0.0;
    x_target[5] = 0.0;
    x_target[6] = 1.0;
    x_target[7] = 0.0;
    x_target[8] = 0.0;
    x_target[9] = 0.0;
    x_target[10] = 0.0;
    x_target[11] = 0.0;

    x_state[0] = 1.0;
    x_state[1] = 1.0;
    x_state[2] = -5.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;
    x_state[5] = 0.0;
    x_state[6] = 1.0;
    x_state[7] = 0.0;
    x_state[8] = 0.0;
    x_state[9] = 0.0;
    x_state[10] = 0.0;
    x_state[11] = 0.0;
    x_state[12] = 0.0;
    x_state[13] = 0.0;
    x_state[14] = 0.0;
    x_state[15] = 0.0;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_target);

    for (int i = 0; i < nx; i++)
        simX(0, i) = x_current[i];

    for (int i = 0; i < N; i++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", x_state);

    // closed loop simulation
    for (int ii = 0; ii < N; ii++)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

        status = drone_acados_solve(acados_ocp_capsule);
        //        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        //        min_time = MIN(elapsed_time, min_time);

        // get the optimized control input
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
        for (int i = 0; i < nu; i++)
            simU(ii, i) = u_current[i];

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        time_record(ii) = elapsed_time_ms;

        // simulation
        sim_in_set(drone_sim_config, drone_sim_dims,
                   drone_sim_in, "u", u_current);
        sim_in_set(drone_sim_config, drone_sim_dims,
                   drone_sim_in, "x", x_current);

        status = drone_acados_sim_solve(sim_capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(drone_sim_config, drone_sim_dims,
                    drone_sim_out, "x", x_current);

        for (int i = 0; i < nx; i++)
            simX(ii + 1, i) = x_current[i];
    }

    // print results
    for (int i = 0; i < N + 1; i++)
    {
        printf("Final position index %d %f, %f, %f \n", i, simX(i, 0), simX(i, 1), simX(i, 2));
        // printf("Final velocity index %d %f, %f, %f \n", i, simX(i, 3), simX(i, 4), simX(i, 5));
        // printf("Final attitude index %d %f, %f, %f, %f \n", i, simX(i, 6), simX(i, 7), simX(i, 8), simX(i, 9));
        // printf("Final control index %d %f, %f, %f, %f \n", i, simU(i, 0), simU(i, 1), simU(i, 2), simU(i, 3));
    }

    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());
    return status;
}