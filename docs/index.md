# Dyros Robot Controller

**Dyros Robot Controller** is a ROS 2 C++ library for mobile robots, manipulators, and mobile manipulators.
It provides a unified controller API backed by [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) for kinematics/dynamics and [OSQP](https://osqp.org/) for quadratic-programming solvers, along with Python bindings and MuJoCo simulation examples.

<div class="grid cards" markdown>

-   :material-robot-arm: **Manipulator control**

    Joint-space PD, CLIK, OSF, QP-based inverse kinematics and inverse dynamics, and hierarchical QP — all with CBF safety constraints.

    [→ Joint-space control](concepts/joint-space-control.md) · [→ Task-space control](concepts/task-space-control.md) · [→ QP / HQP](concepts/qp-and-hqp.md)

-   :material-car: **Mobile robot control**

    Kinematic support for differential-drive, mecanum-wheel, and caster-wheel mobile bases.

-   :material-robot: **Mobile manipulator control**

    Whole-body QP / HQP interfaces that split optimized velocity or torque commands into a mobile-base vector and a manipulator vector.

-   :material-code-braces: **Python and C++ examples**

    Minimal MuJoCo simulation loops for the Franka FR3, Summit XLS, and FR3-on-XLS robots.

</div>

<div style="display:flex; gap:1rem; justify-content:center; flex-wrap:wrap; margin: 1.5rem 0;">

<figure style="flex:1; min-width:200px; text-align:center; margin:0;">
  <img src="assets/images/fr3.gif" alt="Franka FR3" style="width:100%;">
  <figcaption>Franka FR3 — manipulator</figcaption>
</figure>

<figure style="flex:1; min-width:200px; text-align:center; margin:0;">
  <img src="assets/images/xls.gif" alt="Summit XLS" style="width:100%;">
  <figcaption>Summit XLS — mobile base</figcaption>
</figure>

<figure style="flex:1; min-width:200px; text-align:center; margin:0;">
  <img src="assets/images/fr3_xls.gif" alt="FR3 on Summit XLS" style="width:100%;">
  <figcaption>FR3 on Summit XLS — mobile manipulator</figcaption>
</figure>

</div>

## Package Layout

| Path | Purpose |
| --- | --- |
| `include/dyros_robot_controller/` | Public C++ headers |
| `src/` | C++ controller, solver, and binding implementations |
| `drc/` | Python wrapper package |
| `examples/C++` | C++ MuJoCo examples |
| `examples/python` | Python MuJoCo examples |
| `examples/robots` | Example robot URDF/SRDF models and assets |

## Controller Classes

| C++ class | Python module | Purpose |
| --- | --- | --- |
| `drc::Manipulator::RobotData` | `drc.manipulator.RobotData` | State storage and kinematics/dynamics (manipulator) |
| `drc::Manipulator::RobotController` | `drc.manipulator.RobotController` | Control computations (manipulator) |
| `drc::Mobile::RobotData` | `drc.mobile.RobotData` | State storage (mobile base) |
| `drc::Mobile::RobotController` | `drc.mobile.RobotController` | Control computations (mobile base) |
| `drc::MobileManipulator::RobotData` | `drc.mobile_manipulator.RobotData` | State storage (whole-body) |
| `drc::MobileManipulator::RobotController` | `drc.mobile_manipulator.RobotController` | Control computations (whole-body) |

## Quick Start

```python
from drc.manipulator import RobotData, RobotController
from drc.type_define import TaskSpaceData
import numpy as np

robot_data = RobotData(dt=0.001, urdf_path="path/to/robot.urdf")
robot_ctrl = RobotController(robot_data)

# Set QP-IK weights
dof = robot_data.get_dof()
robot_ctrl.set_QPIK_gain(
    w_tracking   = np.array([100]*6, dtype=float),
    w_vel_damping = np.ones(dof) * 0.01,
    w_acc_damping = np.ones(dof) * 0.1,
)

# Control loop
while True:
    robot_data.update_state(q, qdot)                          # update from simulator

    task = TaskSpaceData()
    task.x_desired         = T_desired                        # target SE(3) pose
    task.xdot_desired      = np.zeros(6)
    task.x_init            = T_init
    task.xdot_init         = np.zeros(6)
    task.control_start_time = t_start
    task.current_time       = t_now

    ok, qdot_cmd = robot_ctrl.QPIK_cubic({"end_effector": task}, duration=3.0)
```

See [Installation](getting-started/installation.md) for build instructions and [Examples](examples/overview.md) for complete simulation loops.
