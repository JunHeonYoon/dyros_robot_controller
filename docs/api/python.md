# Python API

The Python package is installed as `drc` via `ament_python_install_package`. After sourcing the workspace:

```python
import drc
from drc.manipulator        import RobotData, RobotController
from drc.mobile             import RobotData as MobileData, RobotController as MobileController
from drc.mobile_manipulator import RobotData as MomaData,   RobotController as MomaController
from drc.type_define        import TaskSpaceData
```

The Python API mirrors the C++ API. Method names use `snake_case` equivalents of the C++ `CamelCase` names.

---

## Modules

| Module | Contents |
| --- | --- |
| `drc.manipulator` | `RobotData`, `RobotController` for single-arm robots |
| `drc.mobile` | `RobotData`, `RobotController` for mobile bases |
| `drc.mobile_manipulator` | `RobotData`, `RobotController` for whole-body mobile manipulators |
| `drc.type_define` | `TaskSpaceData`, `KinematicParam`, `JointIndex`, `ActuatorIndex` |

---

## Manipulator

### Initialization

```python
import numpy as np
from drc.manipulator import RobotData, RobotController

robot_data = RobotData(
    dt=0.001,
    urdf_path="path/to/robot.urdf",
    srdf_path="path/to/robot.srdf",   # optional; pass "" to skip
    packages_path="path/to/packages"  # optional
)
robot_ctrl = RobotController(robot_data)

dof = robot_data.get_dof()
```

### State update

```python
robot_data.update_state(q, qdot)   # numpy arrays of shape (dof,)
```

### Gain setters

All C++ gain setters have Python equivalents with the same semantics.  
See [C++ Gain Setters](cpp.md#gain-setters) for the mathematical meaning of each gain.

```python
# Joint-space PD gains  (default Kp=400, Kv=40 per joint)
robot_ctrl.set_joint_gain(Kp, Kv)         # numpy arrays (dof,)
robot_ctrl.set_joint_Kp_gain(Kp)
robot_ctrl.set_joint_Kv_gain(Kv)

# IK proportional gain  (used by CLIKStep, QPIKStep, HQPIKStep)
robot_ctrl.set_IK_gain({"end_effector": np.ones(6) * 5.0})
robot_ctrl.set_IK_gain(np.ones(6) * 5.0)   # same gain for all links

# ID proportional + derivative gains  (used by OSFStep, QPIDStep, HQPIDStep)
robot_ctrl.set_ID_gain({"end_effector": np.ones(6) * 100}, {"end_effector": np.ones(6) * 20})
robot_ctrl.set_ID_gain(np.ones(6) * 100, np.ones(6) * 20)

# QPIK weights
robot_ctrl.set_QPIK_gain(
    w_tracking    = np.array([100.0]*6),
    w_vel_damping = np.ones(dof) * 0.01,
    w_acc_damping = np.ones(dof) * 0.1,
)

# QPID weights
robot_ctrl.set_QPID_gain(
    w_tracking    = np.array([100.0]*6),
    w_vel_damping = np.ones(dof) * 0.01,
    w_acc_damping = np.ones(dof) * 0.1,
)

# HQPIK / HQPID weights
robot_ctrl.set_HQPIK_gain(w_tracking, w_vel_damping, w_acc_damping)
robot_ctrl.set_HQPID_gain(w_tracking, w_vel_damping, w_acc_damping)
```

### Joint-space control

```python
# Cubic reference trajectory
q_ref    = robot_ctrl.move_joint_position_cubic(
    q_target, qdot_target, q_init, qdot_init, current_time, init_time, duration)
qdot_ref = robot_ctrl.move_joint_velocity_cubic(...)  # same args

# PD torque from desired position/velocity: τ = M(Kp*eq + Kv*eqdot) + g
tau = robot_ctrl.move_joint_torque_step(q_target, qdot_target, use_mass=True)

# Torque from desired acceleration: τ = M*qddot_d + g
tau = robot_ctrl.move_joint_torque_step(qddot_target, use_mass=True)

# Cubic trajectory + PD torque
tau = robot_ctrl.move_joint_torque_cubic(
    q_target, qdot_target, q_init, qdot_init, current_time, init_time, duration)
```

### Task-space — CLIK

```python
from drc.type_define import TaskSpaceData

task = TaskSpaceData()
task.set_zero()

# CLIK: requires xdot_desired
task.xdot_desired = np.zeros(6)
ok, qdot = robot_ctrl.CLIK({"end_effector": task})
ok, qdot = robot_ctrl.CLIK({"end_effector": task}, null_qdot=np.zeros(dof))

# CLIKStep: requires x_desired, xdot_desired
task.x_desired    = T_desired        # 4x4 homogeneous matrix or Eigen.Affine3d
task.xdot_desired = np.zeros(6)
ok, qdot = robot_ctrl.CLIK_step({"end_effector": task})

# CLIKCubic: requires x_init, xdot_init, x_desired, xdot_desired,
#             control_start_time, current_time
task.set_init()                      # snapshot current state into *_init fields
task.x_desired         = T_desired
task.xdot_desired      = np.zeros(6)
task.current_time      = t_now
ok, qdot = robot_ctrl.CLIK_cubic({"end_effector": task}, duration=3.0)
```

### Task-space — OSF

```python
# OSF: requires xddot_desired
task.xddot_desired = np.zeros(6)
ok, tau = robot_ctrl.OSF({"end_effector": task})
ok, tau = robot_ctrl.OSF({"end_effector": task}, null_torque=np.zeros(dof))

# OSFStep: requires x_desired, xdot_desired
ok, tau = robot_ctrl.OSF_step({"end_effector": task})

# OSFCubic: requires cubic fields (same as CLIKCubic)
ok, tau = robot_ctrl.OSF_cubic({"end_effector": task}, duration=3.0)
```

### QP inverse kinematics

```python
# QPIK (velocity-level QP with CBF safety constraints)
ok, qdot = robot_ctrl.QPIK({"end_effector": task})

# QPIKStep: adds IK gain feedback before solving QP
ok, qdot = robot_ctrl.QPIK_step({"end_effector": task})

# QPIKCubic: cubic trajectory + IK gain + QP solver
ok, qdot = robot_ctrl.QPIK_cubic({"end_effector": task}, duration=3.0)
```

Returns `(False, zeros)` if the QP solver fails.

### QP inverse dynamics

```python
ok, tau = robot_ctrl.QPID({"end_effector": task})
ok, tau = robot_ctrl.QPID_step({"end_effector": task})
ok, tau = robot_ctrl.QPID_cubic({"end_effector": task}, duration=3.0)
```

On solver failure `tau` is set to gravity compensation `g(q)`.

### Hierarchical QP

```python
hierarchy = [
    {"panda_hand":  high_priority_task},   # level 0: highest priority
    {"panda_link4": lower_priority_task},  # level 1
]

# HQPIKStep
ok, qdot = robot_ctrl.HQPIK_step(hierarchy)

# HQPIKCubic
ok, qdot = robot_ctrl.HQPIK_cubic(hierarchy, duration=3.0)

# HQPIDStep / HQPIDCubic
ok, tau = robot_ctrl.HQPID_step(hierarchy)
ok, tau = robot_ctrl.HQPID_cubic(hierarchy, duration=3.0)
```

---

## Mobile Base

```python
from drc.type_define import KinematicParam, DriveType
from drc.mobile      import RobotData

param = KinematicParam()
param.type         = DriveType.Differential
param.wheel_radius = 0.1
param.base_width   = 0.5

mobile_data = RobotData(dt=0.001, param=param)
mobile_data.update_state(wheel_pos, wheel_vel)  # shape (wheel_num,)
```

See [C++ Mobile hard-coded assumptions](cpp.md#drive-type-requirements-and-hard-coded-assumptions) for wheel ordering and joint-count constraints — the same rules apply to Python.

---

## Mobile Manipulator

### Initialization

```python
from drc.type_define        import KinematicParam, DriveType, JointIndex, ActuatorIndex
from drc.mobile_manipulator import RobotData, RobotController

# Mobile-base kinematics (differential drive example)
param = KinematicParam()
param.type         = DriveType.Differential
param.wheel_radius = 0.1
param.base_width   = 0.5

# Joint index: describes where each group starts in the Pinocchio joint vector
joint_idx               = JointIndex()
joint_idx.virtual_start = 0   # x, y, yaw (always 3 joints)
joint_idx.mani_start    = 3   # first arm joint
joint_idx.mobi_start    = 10  # first wheel joint

# Actuator index: describes where each group starts in the actuator command vector
actuator_idx            = ActuatorIndex()
actuator_idx.mani_start = 0   # arm torques/velocities start here
actuator_idx.mobi_start = 7   # wheel commands start here

moma_data = RobotData(
    dt=0.001,
    mobile_param=param,
    joint_idx=joint_idx,
    actuator_idx=actuator_idx,
    urdf_path="path/to/robot.urdf",
    srdf_path="path/to/robot.srdf",   # optional; "" to skip
    packages_path="path/to/packages"  # optional
)
```

### DOF attributes

After construction `RobotData` exposes integer attributes for convenience:

```python
mani_dof = moma_data.mani_dof   # number of arm joints
mobi_dof = moma_data.mobi_dof   # number of wheel joints
act_dof  = moma_data.get_actuator_dof()  # mani_dof + mobi_dof
```

!!! warning "No `.mani` / `.mobi` / `.moma` sub-object proxies in Python"
    The C++ API exposes `data.mani`, `data.mobi`, and `data.moma` as sub-object references that
    give scoped access to arm-only or mobile-only kinematics.
    **These proxy references are not available in the Python bindings.**
    Use the flat methods directly on `moma_data` instead:

    ```python
    # C++ sub-object pattern (NOT available in Python):
    #   J_arm = moma_data.mani.compute_jacobian("panda_hand")   ← does not exist

    # Python equivalent — use the full MobileManipulator methods:
    J_full = moma_data.compute_jacobian_actuated("panda_hand")  # (6 × act_dof)
    J_mobi = moma_data.compute_mobile_fk_jacobian()             # mobile Jacobian
    ```

### State update

```python
moma_data.update_state(
    q_virtual,    # shape (3,)       — [x, y, yaw]
    q_mobile,     # shape (mobi_dof,) — wheel positions
    q_mani,       # shape (mani_dof,) — arm joint positions
    qdot_virtual, # shape (3,)
    qdot_mobile,  # shape (mobi_dof,)
    qdot_mani,    # shape (mani_dof,)
)
```

### Gain setup

Gains are set directly on `RobotController` using the same methods as the Manipulator API.  
`w_tracking`, `w_vel_damping`, `w_acc_damping` sizes must match `act_dof` for the joint-damping weights and 6 (or `{link: (6,)}`) for tracking weights:

```python
moma_ctrl = RobotController(moma_data)
act_dof   = moma_data.get_actuator_dof()

# Arm PD gains (size = mani_dof)
moma_ctrl.set_manipulator_joint_gain(Kp_arm, Kv_arm)

# IK proportional gain (for CLIK / QPIK Step variants)
moma_ctrl.set_IK_gain({"panda_hand": np.ones(6) * 5.0})

# Whole-body QP-IK weights
moma_ctrl.set_QPIK_gain(
    w_tracking    = np.array([100.0] * 6),
    w_vel_damping = np.ones(act_dof) * 0.01,
    w_acc_damping = np.ones(act_dof) * 0.1,
)

# Whole-body QP-ID weights
moma_ctrl.set_QPID_gain(
    w_tracking    = np.array([100.0] * 6),
    w_vel_damping = np.ones(act_dof) * 0.01,
    w_acc_damping = np.ones(act_dof) * 0.1,
)
```

### Control methods

All task-space methods accept a `dict[str, TaskSpaceData]` mapping link names to desired states.

#### CLIK / OSF

```python
ok, qdot_mobile, qdot_mani = moma_ctrl.CLIK({"panda_hand": task})
ok, qdot_mobile, qdot_mani = moma_ctrl.CLIK_step({"panda_hand": task})
ok, qdot_mobile, qdot_mani = moma_ctrl.CLIK_cubic({"panda_hand": task}, duration=3.0)

ok, tau_mobile, tau_mani   = moma_ctrl.OSF({"panda_hand": task})
ok, tau_mobile, tau_mani   = moma_ctrl.OSF_step({"panda_hand": task})
ok, tau_mobile, tau_mani   = moma_ctrl.OSF_cubic({"panda_hand": task}, duration=3.0)
```

#### QP Inverse Kinematics

```python
ok, qdot_mobile, qdot_mani = moma_ctrl.QPIK({"panda_hand": task})
ok, qdot_mobile, qdot_mani = moma_ctrl.QPIK_step({"panda_hand": task})
ok, qdot_mobile, qdot_mani = moma_ctrl.QPIK_cubic({"panda_hand": task}, duration=3.0)
```

#### QP Inverse Dynamics

```python
ok, tau_mobile, tau_mani   = moma_ctrl.QPID({"panda_hand": task})
ok, tau_mobile, tau_mani   = moma_ctrl.QPID_step({"panda_hand": task})
ok, tau_mobile, tau_mani   = moma_ctrl.QPID_cubic({"panda_hand": task}, duration=3.0)
```

#### Hierarchical QP

```python
hierarchy = [
    {"panda_hand":  high_priority_task},   # level 0
    {"panda_link4": lower_priority_task},  # level 1
]
ok, qdot_mobile, qdot_mani = moma_ctrl.HQPIK_step(hierarchy)
ok, qdot_mobile, qdot_mani = moma_ctrl.HQPIK_cubic(hierarchy, duration=3.0)
ok, tau_mobile,  tau_mani  = moma_ctrl.HQPID_step(hierarchy)
ok, tau_mobile,  tau_mani  = moma_ctrl.HQPID_cubic(hierarchy, duration=3.0)
```

#### Joint-space (arm only)

```python
# Cubic reference trajectory for the arm
q_ref    = moma_ctrl.move_manipulator_joint_position_cubic(
    q_target, qdot_target, q_init, qdot_init, current_time, init_time, duration)

# PD torque for the arm: τ = M_arm(Kp*eq + Kv*eqdot) + g_arm
tau_arm  = moma_ctrl.move_manipulator_joint_torque_step(q_target, qdot_target, use_mass=True)
tau_arm  = moma_ctrl.move_manipulator_joint_torque_cubic(
    q_target, qdot_target, q_init, qdot_init, current_time, init_time, duration)
```

#### Mobile-base velocity command

```python
cmd_vel  = np.array([vx, vy, omega])   # desired base twist
wheel_vel = moma_ctrl.compute_mobile_wheel_vel(cmd_vel)  # → shape (mobi_dof,)
# or, combined: computes wheel_vel and returns it
moma_ctrl.mobile_velocity_command(cmd_vel, wheel_vel)
```

All hard-coded constraints documented in [C++ Mobile Manipulator](cpp.md#mobile-manipulator) apply equally here.

---

## `TaskSpaceData`

```python
from drc.type_define import TaskSpaceData
import numpy as np

task = TaskSpaceData()
task.set_zero()                    # reset to identity / zeros

# Fields (all writable from Python)
task.x                  # 4×4 numpy array — current pose
task.xdot               # (6,) — current velocity
task.x_desired          # 4×4 — target pose
task.xdot_desired       # (6,) — target velocity
task.xddot_desired      # (6,) — target acceleration
task.x_init             # 4×4 — initial pose (set by set_init())
task.xdot_init          # (6,) — initial velocity
task.control_start_time # float — trajectory start time
task.current_time       # float — must be updated each cycle

task.set_init()         # snapshots current → init fields
task.set_desired()      # snapshots current → desired fields
```

`set_init()` must be called once at the start of each motion segment before using any `*Cubic` variant.
