# C++ API

```cpp
// Manipulator
#include <dyros_robot_controller/manipulator/robot_data.h>
#include <dyros_robot_controller/manipulator/robot_controller.h>

// Mobile base
#include <dyros_robot_controller/mobile/robot_data.h>
#include <dyros_robot_controller/mobile/robot_controller.h>

// Mobile manipulator
#include <dyros_robot_controller/mobile_manipulator/robot_data.h>
#include <dyros_robot_controller/mobile_manipulator/robot_controller.h>
```

---

## Manipulator

### `RobotData` construction

```cpp
auto robot_data = std::make_shared<drc::Manipulator::RobotData>(
    dt,           // double  — control loop period [s]
    urdf_path,    // std::string — path to URDF file
    srdf_path,    // std::string — path to SRDF (optional; "" to skip)
    packages_path // std::string — path for mesh file resolution (optional)
);
```

#### State update

Call once per control cycle before any controller method:

```cpp
robot_data->updateState(q, qdot);  // q, qdot: Eigen::VectorXd of size dof
```

---

### `RobotController` construction

```cpp
auto robot_ctrl = std::make_shared<drc::Manipulator::RobotController>(robot_data);
```

---

### Gain Setters {#gain-setters}

#### Joint-space gains

| Method | Parameters | Role |
| --- | --- | --- |
| `setJointGain(Kp, Kv)` | `VectorXd Kp, Kv` | Sets both PD gains at once |
| `setJointKpGain(Kp)` | `VectorXd Kp` | Proportional gain only |
| `setJointKvGain(Kv)` | `VectorXd Kv` | Derivative gain only |

These gains appear in the PD torque law — see [Joint-Space Control](../concepts/joint-space-control.md#pd-torque-control).

**Defaults:** \(K_p = 400\,I_n\), \(K_v = 40\,I_n\).

#### IK (CLIK) gains

| Method | Parameters | Role |
| --- | --- | --- |
| `setIKGain(link_Kp)` | `map<string, Vector6d>` | Per-link proportional gain \(K_p\) |
| `setIKGain(Kp)` | `Vector6d` | Same \(K_p\) for every link |

These gains feed the CLIK/QPIK **Step** error term: \(\dot{x}^d_{\text{cmd}} = K_p\,e_x + \dot{x}^d\).  
See [Task-Space Control — CLIK](../concepts/task-space-control.md#clik).

#### ID (OSF / QPID) gains

| Method | Parameters | Role |
| --- | --- | --- |
| `setIDGain(link_Kp, link_Kv)` | `map<string, Vector6d>` | Per-link PD gains |
| `setIDGain(Kp, Kv)` | `Vector6d Kp, Kv` | Same gains for every link |
| `setIDKpGain(...)` | `map` or `Vector6d` | Proportional gain only |
| `setIDKvGain(...)` | `map` or `Vector6d` | Derivative gain only |

These gains feed the OSF/QPID **Step** error term: \(\ddot{x}^d_{\text{cmd}} = K_p\,e_x + K_v\,e_{\dot{x}} + \ddot{x}^d\).  
See [Task-Space Control — OSF](../concepts/task-space-control.md#osf).

#### QP-IK weights

| Method | Parameters | Role |
| --- | --- | --- |
| `setQPIKGain(w_tracking, w_vel, w_acc)` | `Vector6d`, `VectorXd`, `VectorXd` | All weights at once |
| `setQPIKTrackingGain(w_tracking)` | `Vector6d` or `map` | Task tracking weight \(W_1\) |
| `setQPIKJointVelGain(w_vel)` | `VectorXd` | Velocity damping weight \(W_2\) |
| `setQPIKJointAccGain(w_acc)` | `VectorXd` | Acceleration damping weight \(W_3\) |

See [QP-IK objective](../concepts/qp-and-hqp.md#qpik-objective) for the full cost formulation.

#### QP-ID weights

| Method | Parameters | Role |
| --- | --- | --- |
| `setQPIDGain(w_tracking, w_vel, w_acc)` | `Vector6d`, `VectorXd`, `VectorXd` | All weights at once |
| `setQPIDTrackingGain(w_tracking)` | `Vector6d` or `map` | Task tracking weight \(W_1\) |
| `setQPIDJointVelGain(w_vel)` | `VectorXd` | Velocity damping weight \(W_2\) |
| `setQPIDJointAccGain(w_acc)` | `VectorXd` | Acceleration damping weight \(W_3\) |

See [QP-ID objective](../concepts/qp-and-hqp.md#qpid-objective) for the full cost formulation.

#### HQP-IK / HQP-ID weights

Same structure as QP weights but prefixed `setHQPIK*` / `setHQPID*`.
The weights apply **uniformly across all priority levels**.

---

### Joint-Space Control

For detailed mathematics see [Joint-Space Control](../concepts/joint-space-control.md).

#### `moveJointPositionCubic` / `moveJointVelocityCubic`

```cpp
VectorXd q_ref    = robot_ctrl->moveJointPositionCubic(
    q_target, qdot_target, q_init, qdot_init, current_time, init_time, duration);
VectorXd qdot_ref = robot_ctrl->moveJointVelocityCubic(...same args...);
```

Returns the cubic polynomial reference at `current_time`.
[→ Cubic polynomial formula](../concepts/joint-space-control.md#cubic-polynomial-trajectory)

#### `moveJointTorqueStep`

```cpp
// Overload 1: PD control from desired position/velocity
// τ = M(q)(Kp*(q_d − q) + Kv*(qdot_d − qdot)) + g(q)
VectorXd tau = robot_ctrl->moveJointTorqueStep(q_target, qdot_target, use_mass=true);

// Overload 2: from desired acceleration
// τ = M(q)*qddot_d + g(q)
VectorXd tau = robot_ctrl->moveJointTorqueStep(qddot_target, use_mass=true);
```

With `use_mass = false` the mass matrix is replaced by the identity (feedforward + gravity only).

#### `moveJointTorqueCubic`

Generates a cubic joint trajectory then calls `moveJointTorqueStep`.

---

### Task-Space Control — CLIK

For detailed mathematics see [Task-Space Control — CLIK](../concepts/task-space-control.md#clik).

#### `CLIK`

Solves \(\dot{q} = J^+\dot{x}^d + (I-J^+J)\dot{q}_{\text{null}}\) directly.

```cpp
// link_task_data must include xdot_desired
robot_ctrl->CLIK(link_task_data, opt_qdot);
robot_ctrl->CLIK(link_task_data, opt_qdot, null_qdot);  // with null-space motion
```

#### `CLIKStep`

Adds IK proportional error feedback before solving CLIK.
`x_desired` and `xdot_desired` must be set in `TaskSpaceData`.

```cpp
robot_ctrl->CLIKStep(link_task_data, opt_qdot);
```

#### `CLIKCubic`

Generates cubic trajectory then calls `CLIKStep`.
`x_init`, `xdot_init`, `x_desired`, `xdot_desired`, `control_start_time`, `current_time` must be set.

```cpp
robot_ctrl->CLIKCubic(link_task_data, duration, opt_qdot);
```

---

### Task-Space Control — OSF

For detailed mathematics see [Task-Space Control — OSF](../concepts/task-space-control.md#osf).

#### `OSF`

Computes torques via operational space formulation: \(\tau = J^T\Lambda\ddot{x}^d + N^T\tau_{\text{null}} + g\).
`xddot_desired` must be set.

```cpp
robot_ctrl->OSF(link_task_data, opt_torque);
robot_ctrl->OSF(link_task_data, opt_torque, null_torque);
```

#### `OSFStep` / `OSFCubic`

Same pattern as CLIKStep / CLIKCubic but at torque level.

---

### QP Inverse Kinematics

For detailed mathematics see [QPIK](../concepts/qp-and-hqp.md#qpik-objective).

!!! warning "Self-collision: use primitive `<collision>` geometries, not meshes"
    Every call to `QPIK`, `QPID`, and all HQP variants invokes Pinocchio + FCL
    `computeMinimumDistance()` to evaluate self-collision CBF constraints.
    When `<collision>` elements in the URDF reference **mesh files** (STL / OBJ / DAE),
    FCL must run GJK/EPA over every triangle — easily **several milliseconds per link pair**,
    which breaks any sub-millisecond control loop.

    **Always replace collision meshes with primitives:**

    | Primitive | URDF snippet |
    |---|---|
    | Sphere | `<geometry><sphere radius="0.05"/></geometry>` |
    | Cylinder | `<geometry><cylinder radius="0.04" length="0.3"/></geometry>` |
    | Box | `<geometry><box size="0.1 0.1 0.2"/></geometry>` |
    | Capsule | `<geometry><capsule radius="0.04" length="0.3"/></geometry>` |

    With all-primitive collision models a 7-DOF robot completes the full pairwise check in **< 0.05 ms** — two orders of magnitude faster than mesh-based queries.

    The `srdf_path` argument to `RobotData` can additionally list collision pairs to **disable** (e.g. adjacent links), reducing the number of checked pairs further.

#### `QPIK`

```cpp
bool ok = robot_ctrl->QPIK(link_task_data, opt_qdot);
bool ok = robot_ctrl->QPIK(link_task_data, opt_qdot, time_verbose);  // string timing output
```

Returns `false` if the QP solver fails; `opt_qdot` is zeroed on failure.

#### `QPIKStep` / `QPIKCubic`

Same variant pattern as CLIK. `QPIKStep` adds IK proportional error feedback.

---

### QP Inverse Dynamics

For detailed mathematics see [QPID](../concepts/qp-and-hqp.md#qpid-objective).

#### `QPID` / `QPIDStep` / `QPIDCubic`

Same pattern as QPIK but at torque level.
On QP failure `opt_torque` is set to gravity compensation (`g(q)`).

---

### Hierarchical QP

For detailed mathematics see [HQP](../concepts/qp-and-hqp.md#hierarchical-qp).

#### `HQPIK` / `HQPIKStep` / `HQPIKCubic`

```cpp
std::vector<std::map<std::string, drc::TaskSpaceData>> hierarchy;
hierarchy.push_back({{ "panda_hand",  ee_task }});    // level 0: highest priority
hierarchy.push_back({{ "panda_link4", elbow_task }}); // level 1: lower priority

Eigen::VectorXd qdot_desired(dof);
robot_ctrl->HQPIKStep(hierarchy, qdot_desired);
```

The hierarchy structure is rebuilt automatically when the number of tasks per level changes.

#### `HQPID` / `HQPIDStep` / `HQPIDCubic`

Same pattern as HQPIK but at torque level.

---

## Mobile Base

### `RobotData` construction

```cpp
drc::Mobile::KinematicParam param;
param.type         = drc::Mobile::DriveType::Differential;
param.wheel_radius = 0.1;
param.base_width   = 0.5;

auto mobile_data = std::make_shared<drc::Mobile::RobotData>(dt, param);
```

#### State update

```cpp
// wheel_pos, wheel_vel: Eigen::VectorXd of size wheel_num
mobile_data->updateState(wheel_pos, wheel_vel);
```

### Drive-type requirements and hard-coded assumptions

!!! warning "Differential drive — wheel count is always 2"
    `DriveType::Differential` **hard-codes `wheel_num_ = 2`**.
    The joint/wheel vector is expected in the order **`[left_wheel, right_wheel]`**.
    The Jacobian is:
    \[
    J_{\text{diff}} =
    \begin{bmatrix}
    r/2 & r/2 \\
    0   & 0   \\
    -r/L & r/L
    \end{bmatrix}
    \]
    where \(r\) = `wheel_radius` and \(L\) = `base_width`.
    A positive wheel velocity on the left corresponds to the wheel spinning forward.

!!! warning "Mecanum drive — roller-angle vector length determines wheel count"
    `DriveType::Mecanum` sets `wheel_num_` equal to `roller_angles.size()`.
    `roller_angles`, `base2wheel_positions`, and `base2wheel_angles` **must all have the same length**;
    an assertion fires at construction time if they differ.
    Wheel ordering must match the order these three vectors are filled.

!!! warning "Caster drive — each caster module occupies 2 joints"
    `DriveType::Caster` sets `wheel_num_ = base2wheel_positions.size() * 2`.
    Each caster unit contributes one steering joint and one drive joint.
    The joint order in the input vector must follow the caster-module order given in `base2wheel_positions`.

### `KinematicParam` field summary

| Field | Type | Required for |
| --- | --- | --- |
| `type` | `DriveType` | all |
| `wheel_radius` | `double` | all |
| `base_width` | `double` | `Differential` |
| `roller_angles` | `vector<double>` | `Mecanum` |
| `base2wheel_positions` | `vector<Vector2d>` | `Mecanum`, `Caster` |
| `base2wheel_angles` | `vector<double>` | `Mecanum`, `Caster` |
| `wheel_offset` | `double` | `Caster` |
| `max_lin_speed` | `double` | optional (default 2.0 m/s) |
| `max_ang_speed` | `double` | optional (default 2.0 rad/s) |

---

## Mobile Manipulator

### `RobotData` construction

```cpp
drc::MobileManipulator::JointIndex joint_idx;
joint_idx.virtual_start = 0;   // index of first virtual (floating-base) joint
joint_idx.mani_start    = 3;   // index of first manipulator joint
joint_idx.mobi_start    = 10;  // index of first wheel joint

drc::MobileManipulator::ActuatorIndex actuator_idx;
actuator_idx.mani_start = 0;   // index of manipulator in the actuator vector
actuator_idx.mobi_start = 7;   // index of wheels in the actuator vector

auto moma_data = std::make_shared<drc::MobileManipulator::RobotData>(
    dt, mobile_param, joint_idx, actuator_idx,
    urdf_path, srdf_path, packages_path);
```

#### State update

```cpp
moma_data->updateState(q_virtual, q_mobile, q_mani,
                        qdot_virtual, qdot_mobile, qdot_mani);
```

### Sub-object accessors

After construction, three named sub-references provide scoped access:

| Reference | Type | Scope |
| --- | --- | --- |
| `moma_data->moma` | `MobileManipulator::RobotData&` | Whole-body (world frame, full DOF) |
| `moma_data->mani` | `Manipulator::RobotData&` | Arm-only quantities in the mobile-base frame |
| `moma_data->mobi` | `Mobile::RobotData&` | Mobile-base quantities only |

Use `mani` when passing a manipulator-only proxy to a standalone `Manipulator::RobotController`.

### URDF structure requirements

!!! warning "URDF joint ordering must match `JointIndex`"
    The URDF joint ordering as parsed by Pinocchio **must** exactly match the indices given in `JointIndex`.
    The recommended URDF ordering is:

    ```
    [virtual_joint_x, virtual_joint_y, virtual_joint_yaw]   ← indices 0-2
    [arm_joint_1, ..., arm_joint_n]                          ← indices 3 to 3+n-1
    [wheel_joint_1, ..., wheel_joint_m]                      ← indices 3+n to end
    ```

    If `joint_idx.virtual_start`, `joint_idx.mani_start`, or `joint_idx.mobi_start` do not match the Pinocchio joint order, dynamics and Jacobians will be incorrect and silently corrupted.

!!! warning "Virtual DOF is hard-coded to 3"
    The floating base is assumed to have exactly **3 virtual joints** (\(x\), \(y\), \(\text{yaw}\)) regardless of the robot.
    This is hard-coded as `virtual_dof_ = 3` and cannot be changed without modifying the source.
    Full 6-DOF floating-base robots (e.g., aerial manipulators) are not supported by this class.

!!! warning "Manipulator DOF is inferred, not explicit"
    `mani_dof_` is computed as:
    ```
    mani_dof_ = total_dof − virtual_dof_(3) − wheel_num_
    ```
    If the URDF contains additional passive joints, gripper joints, or auxiliary joints beyond the arm and wheels, `mani_dof_` will be incorrect. 
    There is a `TODO` in the source tracking this limitation — always verify `moma_data->getManipulatorDof()` against your expected arm DOF.

!!! warning "Manipulator Jacobians and dynamics are expressed in the mobile-base frame"
    When using `moma_data->mani.*` accessors, all poses, Jacobians, and dynamic quantities are
    **re-expressed in the current mobile-base frame**, not the world frame. This is intentional for
    arm-only controllers, but note that the base-frame changes every timestep as the platform moves.

### QP / HQP outputs for mobile manipulators

`QPIK` and `QPID` methods on `MobileManipulator::RobotController` return two separate vectors:

```cpp
Eigen::VectorXd opt_qdot_mobile;
Eigen::VectorXd opt_qdot_mani;
bool ok = robot_ctrl->QPIKCubic(link_task_data, duration,
                                 opt_qdot_mobile, opt_qdot_mani);
```

The whole-body Jacobian is formed internally and the solution is split at the mobile/manipulator DoF boundary defined by `JointIndex`.

---

## Utility: `TaskSpaceData`

```cpp
#include <dyros_robot_controller/type_define.h>

drc::TaskSpaceData task;
task.setZero();                         // reset all fields

// At the start of a motion segment:
task.x      = robot_data->getPose("end_effector");  // current pose
task.xdot   = robot_data->getVelocity("end_effector");
task.current_time = t_now;
task.setInit();                         // snapshot into *_init fields

// Set target:
task.x_desired         = T_target;
task.xdot_desired      = Vector6d::Zero();
task.xddot_desired     = Vector6d::Zero();
task.current_time      = t_now;        // updated each cycle
```

`setInit()` copies `x`, `xdot`, `xddot`, and `current_time` into `x_init`, `xdot_init`, `xddot_init`, and `control_start_time` respectively — call it once at trajectory start.
