# Digital Twin for Augmented VDE Test Facility

Real-time Hardware-in-the-Loop (HIL) digital twin platform for space docking simulation.

## Architecture Overview
- **Dynamics Engine Software (Server):** Real-time physics engine loop executed at 1 ms interval.
- **Visualization Software (Client):** Decoupled rendering loop running at 60 Hz display refresh rate.
- **External Interfaces:** Network socket connections to Stewart Platform Control, DAQ/Sensors, and NGC Module.

## Project Structure
```text
VDE/
├── docs/             # Design and architecture documentation
├── include/          # Public header files
├── src/              # Implementation source code
│   ├── dynamics_engine/ # Real-time dynamics and kinematics engine
│   ├── visualization/   # GUI and 3D rendering engine
│   ├── network/         # Client-Server socket network layer
│   └── utils/           # CSV and Error logging modules
├── tests/            # Unit, integration, and system tests
├── config/           # Setup and run parameters
└── logs/             # Output directory for execution logs
```

## Build Instructions
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
