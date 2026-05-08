# Controller Architecture

Dyros Robot Controller separates **robot state** from **controller computation** into two distinct classes.

## Class Hierarchy

```
RobotData          ← owns model, state, and all derived kinematics/dynamics
    │
    └─► RobotController   ← consumes RobotData; computes control commands
            │
            ├─ QPIK / QPID         (single-priority QP solvers)
            └─ HQPIK / HQPID       (hierarchical QP solvers)
```

## RobotData

`RobotData` wraps a [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) model and caches every quantity that the controller needs at each timestep.

| Quantity | Method | Symbol |
| --- | --- | --- |
| Joint positions / velocities | `getJointPosition()` / `getJointVelocity()` | \(q,\;\dot{q}\) |
| Link SE(3) pose | `getPose(link_name)` | \(T \in SE(3)\) |
| Link Jacobian | `getJacobian(link_name)` | \(J \in \mathbb{R}^{6 \times n}\) |
| Link Jacobian time-derivative | `getJacobianTimeVariation(link_name)` | \(\dot{J}\) |
| Mass matrix / inverse | `getMassMatrix()` / `getMassMatrixInv()` | \(M,\;M^{-1}\) |
| Coriolis vector | `getCoriolis()` | \(c\) |
| Gravity vector | `getGravity()` | \(g\) |
| Nonlinear effects | `getNonlinearEffects()` | \(\text{nle} = c + g\) |
| Minimum self-collision distance | `getMinDistance(...)` | \(d_{\min}\) |
| Manipulability + gradient | `getManipulability(...)` | \(w,\;\nabla_q w\) |

Call `updateState(q, qdot)` once per control cycle to refresh all cached quantities before calling any controller method.

## RobotController

`RobotController` consumes a shared `RobotData` pointer and exposes the following controller groups:

| Group | Methods | Details |
| --- | --- | --- |
| Joint-space | `moveJoint*` | [Joint-Space Control](joint-space-control.md) |
| CLIK | `CLIK*`, `CLIKStep*`, `CLIKCubic*` | [Task-Space Control → CLIK](task-space-control.md#clik) |
| OSF | `OSF*`, `OSFStep*`, `OSFCubic*` | [Task-Space Control → OSF](task-space-control.md#osf) |
| QP-IK | `QPIK*`, `QPIKStep*`, `QPIKCubic*` | [QP and HQP → QPIK](qp-and-hqp.md#qpik-objective) |
| QP-ID | `QPID*`, `QPIDStep*`, `QPIDCubic*` | [QP and HQP → QPID](qp-and-hqp.md#qpid-objective) |
| HQP-IK | `HQPIK*`, `HQPIKStep*`, `HQPIKCubic*` | [QP and HQP → HQP](qp-and-hqp.md#hierarchical-qp) |
| HQP-ID | `HQPID*`, `HQPIDStep*`, `HQPIDCubic*` | [QP and HQP → HQP](qp-and-hqp.md#hierarchical-qp) |

### Method naming convention

Each controller has three variants:

| Suffix | Input | Description |
| --- | --- | --- |
| (none) | `xdot_desired` / `xddot_desired` | Desired velocity or acceleration fed directly to the solver. |
| `Step` | `x_desired`, `xdot_desired` | Computes a proportional + feedforward reference from pose error, then calls the bare solver. |
| `Cubic` | `x_init`, `xdot_init`, `x_desired`, `xdot_desired`, `control_start_time`, `current_time` | Generates a cubic spline reference, then calls `Step`. |

## Task Data Flow

Task-space methods accept a `std::map<std::string, TaskSpaceData>` keyed by link name.
`TaskSpaceData` bundles all fields needed by any variant:

```cpp
struct TaskSpaceData {
    Affine3d  x_desired;           // target SE(3) pose
    Vector6d  xdot_desired;        // target 6D velocity
    Vector6d  xddot_desired;       // target 6D acceleration
    Affine3d  x_init;              // initial SE(3) pose  (Cubic variants)
    Vector6d  xdot_init;           // initial 6D velocity (Cubic variants)
    double    control_start_time;  // trajectory start time (Cubic variants)
    double    current_time;        // current simulation time (Cubic variants)
};
```

Unused fields are ignored silently.

## Mobile Manipulator Outputs

Mobile manipulator QP / HQP methods return two separate command vectors:

```cpp
Eigen::VectorXd opt_qdot_mobile;       // mobile-base DoF
Eigen::VectorXd opt_qdot_manipulator;  // manipulator DoF
```

The whole-body Jacobian is stacked internally and the solution is split at the mobile / manipulator DoF boundary.
