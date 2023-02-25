#ifndef Drone_C_MPC_drone_point_H
#define Drone_C_MPC_drone_point_H

#include <chrono>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"

#include "acados_sim_solver_drone_simple.h"
#include "acados_solver_drone_simple.h"

int status; // acados operation state

double min_time = 1e12;
double elapsed_time;

double x_target[3];
double x_state[7];
double x_current[10];
double u_current[4];
int N;
int nx;
int nu;

#endif // Drone_C_MPC_drone_point_H