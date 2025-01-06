cmake_minimum_required(VERSION 3.1)
project(Drone_C_MPC LANGUAGES C CXX)

# for macOS
set(CMAKE_MACOSX_RPATH 1)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")

# include Eigen3
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})

# include some more paths
set(ENV{ACADOS_SOURCE_DIR} "$ENV{HOME}/acados")
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
# include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados_c)
# include_directories($ENV{ACADOS_SOURCE_DIR}/include/qpOASES_e)
include_directories(python/c_generated_code)

# load acados libs
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

# simple model
file(GLOB drone_simple_ocp_solver
        python/c_generated_code/acados_solver_drone_simple.c
        )
file(GLOB drone_simple_casadi_fun
        python/c_generated_code/drone_simple_model/*.c)
file(GLOB drone_simple_sim_solver
        python/c_generated_code/acados_sim_solver_drone_simple.c
        )

# full model
file(GLOB drone_ocp_solver
        python/c_generated_code/acados_solver_drone.c
        )
file(GLOB drone_casadi_fun
        python/c_generated_code/drone_model/*.c)
file(GLOB drone_sim_solver
        python/c_generated_code/acados_sim_solver_drone.c
        )

# Define the example
## these two are only basic tests
#add_executable(main_sim_mobile_robot ../python/c_generated_code/main_sim_mobile_robot.c
#        ${casadi_fun}
#        ${sim_solver}
#        )
#target_link_libraries(main_sim_mobile_robot acados hpipm blasfeo)
# # only
#add_executable(main_mobile_robot ../python/c_generated_code/main_mobile_robot.c
#        ${casadi_fun}
#        ${sim_solver}
#        ${ocp_solver})
#target_link_libraries(main_mobile_robot acados hpipm blasfeo)
#add_test(main_mobile_robot main_mobile_robot)

# create libraries
# simple model
add_library(drone_simple_ocp_shared_lib SHARED ${drone_simple_ocp_solver} ${drone_simple_casadi_fun} )
target_link_libraries(drone_simple_ocp_shared_lib acados hpipm blasfeo)

add_library(drone_simple_sim_shared_lib SHARED ${drone_simple_sim_solver} ${drone_simple_casadi_fun})
target_link_libraries(drone_simple_sim_shared_lib acados hpipm blasfeo)

# full model
add_library(drone_ocp_shared_lib SHARED ${drone_ocp_solver} ${drone_casadi_fun} )
target_link_libraries(drone_ocp_shared_lib acados hpipm blasfeo)

add_library(drone_sim_shared_lib SHARED ${drone_sim_solver} ${drone_casadi_fun})
target_link_libraries(drone_sim_shared_lib acados hpipm blasfeo)

# add exe
add_executable(drone_point src/drone_point.cpp )
target_link_libraries(drone_point drone_simple_ocp_shared_lib drone_simple_sim_shared_lib)

add_executable(drone_test src/drone_test.cpp )
target_link_libraries(drone_test drone_ocp_shared_lib drone_sim_shared_lib)
