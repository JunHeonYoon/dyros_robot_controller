# Examples Overview

The `examples` directory contains C++ and Python examples that use MuJoCo simulation.

## Install MuJoCo

```bash
python3 -m pip install mujoco
```

The C++ examples require MuJoCo headers and libraries to be discoverable by CMake.

## C++ Example Directory

```text
examples/C++
```

Build:

```bash
cd examples/C++
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

Run:

```bash
./dyros_robot_controller_example fr3
./dyros_robot_controller_example xls
./dyros_robot_controller_example fr3_xls
```

## Python Example Directory

```text
examples/python
```

Run:

```bash
python3 dyros_robot_controller_example.py --robot_name fr3
python3 dyros_robot_controller_example.py --robot_name xls
python3 dyros_robot_controller_example.py --robot_name fr3_xls
```

## Control Loop Pattern

All examples follow the same shape:

1. Initialize `RobotData` and `RobotController`.
2. Update model state from simulator state.
3. Compute the desired command.
4. Send the command back to the simulator.
