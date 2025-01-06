# Drone_C_MPC

This project implements a drone control system based on Model Predictive Control (MPC) using C++. This repository uses C code generated from the [Drone_Python_MPC](https://github.com/KafuuChikai/Drone_Python_MPC) repository.

## 1. Installation

1. Install **acados** and its dependencies, refer to the [acados official documentation](https://docs.acados.org/installation/index.html).

2. Clone this repository:

   ```bash
   git clone https://github.com/KafuuChikai/Drone_C_MPC.git
   cd Drone_Python_MPC
   git submodule update --init --recursive
   ```

## 2. Usage

1. Generate C code from Python:

   ```bash
   python drone_opt_track.py
   python drone_opt_simple_track.py
   ```

2. Build the project:

   ```bash
    mkdir build
    cd build
    cmake ..
    make
   ```

3. After building the project, you can run the executables:

    ```bash
    ./drone_point
    ./drone_test
    ```