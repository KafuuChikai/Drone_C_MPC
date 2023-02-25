#ifndef Drone_C_MPC_drone_test_H
#define Drone_C_MPC_drone_test_H

#include <chrono>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"

#include "acados_sim_solver_drone.h"
#include "acados_solver_drone.h"

int status; // acados operation state

int idxbx0[3];

double min_time = 1e12;
double elapsed_time;

double x_target[12];
double x_state[16];
double x_current[13];
double u_current[4];
int N;
int nx;
int nu;

#endif // Drone_C_MPC_drone_test_H