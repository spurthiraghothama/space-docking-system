# Spacecraft Docking System: 3D Cube Collision & Dynamics Simulation

A real-time 3D rigid-body docking and post-contact pose estimation simulation built in C++17 with OpenGL, GLFW, GLAD, and GLM.

## Features
- **3D Visualization:** Real-time 60 Hz OpenGL rendering of Chaser and Target spacecraft cubes.
- **Orientation Kinematics:** 4D unit quaternion attitude tracking avoiding Euler gimbal lock, with conversion to Euler Roll-Pitch-Yaw angles.
- **Interactive Scenarios:**
  - `[1]` **Head-on collision**: Symmetrical straight-line rebound, zero net torque.
  - `[2]` **Off-center collision**: Asymmetric impact producing net torque, lever-arm rotation, and spin.
  - `[R]` **Reset**: Re-initialize current scenario.
  - `[SPACE]` **Pause / Resume**: Toggle simulation state.
- **Contact Reconstruction:** Multi-axis force and torque distribution across a 6-cell hexagonal load-cell array, contact point estimation, and rigid-body acceleration propagation.

## Prerequisites
- CMake 3.20+
- C++17 compliant compiler (GCC, Clang, or MSVC)

## Build Instructions

```bash
cmake -B build_win -G "MinGW Makefiles"
cmake --build build_win --target sim
```

## Running the Simulation

- **Windows:** `.\sim.exe` (or `.\build_win\sim.exe`)


