# Franka FR3 Example

![FR3 simulation](../assets/images/fr3.gif)

The FR3 example demonstrates manipulator control through the `Manipulator::RobotController` interface.

## Run

```bash
cd examples/python
python3 dyros_robot_controller_example.py --robot_name fr3
```

```bash
cd examples/C++/build
./dyros_robot_controller_example fr3
```

## Modes

| Key | Mode | Description |
| --- | --- | --- |
| `1` | Home | Move to a predefined joint-space configuration |
| `2` | QPIK | Move the end-effector with QP inverse kinematics |
| `3` | Gravity Compensation | Apply gravity compensation torques |
| `4` | Gravity Compensation W QPID | Use QPID in the gravity compensation workflow |

## Files

| File | Purpose |
| --- | --- |
| `examples/C++/src/fr3_controller.cpp` | C++ controller implementation |
| `examples/C++/include/fr3_controller.hpp` | C++ controller declaration |
| `examples/python/fr3_controller.py` | Python controller implementation |
| `examples/robots/fr3` | FR3 model assets |
