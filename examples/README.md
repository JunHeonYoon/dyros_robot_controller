# Dyros Robot Controller Examples (C++ & Python)

This directory contains minimal examples demonstrating how to use **Dyros Robot Controller**, either directly in **C++** or through its **Python bindings**.
Some examples require the installation of the **MuJoCo** simulator.

Installing **MuJoCo**:
```bash
git clone https://github.com/deepmind/mujoco.git
cd mujoco
mkdir build && cd build
cmake ..
cmake --build .
cmake --install .
```

Installing **MuJoCo-py**:
```bash
pip3 install mujoco
```

## Build C++ examples
```bash
cd C++
mkdir build && cd build
cmake ..
make -j10
```

## Run the Examples
The example codes include two representative robot types:
- 7-DoF torque control Manipulator: [Franka FR3](https://franka.de/franka-research-3)
- Mecanum Wheel based Mobile Robot: [Summit XL Steel](https://robotnik.eu/products/mobile-robots/rb-kairos-2/)

Run Python Example:
```bash
cd python
python3 dyros_robot_controller_example.py --robot_name fr3 # or xls
```

Run C++ Example:
```bash
cd C++/build
./dyros_robot_controller_example fr3 # or xls
```
## Brief Code Explanation

All controller examples — **Franka FR3** ([C++](C++/src/fr3_controller.cpp) / [Python](python/fr3_controller.py)) and **Summit XLS** ([C++](C++/src/xls_controller.cpp) / [Python](python/xls_controller.py)) — follow the same three basic execution stages:

### 1. Initialization
Executed once at the beginning of the simulation or robot startup.  
- Initializes all control-related variables.  
- Loads **`robot_data`** and **`robot_controller`** objects provided by `dyros_robot_controller`.  
- Prepares necessary kinematic and dynamic parameters for subsequent computations.

### 2. Model Update (`updateModel()` in C++, `update_model()` in Python)
Executed **every control iteration** (e.g., 1 kHz loop).  
- Retrieves the current time (`current_time`), joint position (`qpos`), and velocity (`qvel`) from the simulator interface.  
- Calls `robot_data->UpdateState()` / `robot_data.update_state()` to refresh all internal states (Jacobian, transformation matrices, etc.).  
- Updates the robot’s internal model to reflect the most recent configuration.

### 3. Control Computation (`compute()` in C++ and Python)
Executed after each model update step.  
- Computes the desired control input based on the updated robot state.  
- Uses `robot_controller` to generate appropriate control commands (joint torques or velocities).  
- Sends the computed command back to the robot for execution.

---

## FR3 Controller Example

In the **FR3** simulation example, three predefined motions are available.  
Each motion can be activated by pressing the corresponding key number in the terminal window.

| Key | Motion | Description |
|-----|---------|-------------|
| `1` | **Home** | Moves the manipulator to a predefined joint-space home configuration. |
| `2` | **QP-IK** | Uses a QP-based inverse kinematics solver to move the end-effector to a target pose in task space. |
| `3` | **Gravity Compensation** | Applies joint torques to compensate for gravity, keeping the arm passive but balanced. |

---

## XLS Controller Example

In the **Summit XLS** mobile robot example, two basic motion modes are implemented.  
Each mode can be selected from the terminal using the assigned key number.

| Key | Motion | Description |
|-----|---------|-------------|
| `1` | **Stop** | Immediately stops the mobile base. |
| `2` | **Base Velocity Tracking** | Enables velocity tracking mode; the base velocity is computed from keyboard input. |

### Keyboard Control Mapping
| Key | Action | Effect |
|------|---------|--------|
| ↑ / ↓ | Forward / Backward | +vx / -vx |
| ← / → | Left / Right strafe | +vy / -vy |
| `b` / `v` | Rotate CCW / CW | +ω / -ω |

---



