# QP and HQP Control

QP-based controllers formulate the control problem as a **Quadratic Program** so that task tracking, joint constraints, singularity avoidance, and self-collision avoidance can all be handled simultaneously.

---

## QPIK — QP Inverse Kinematics {#qpik-objective}

QPIK computes joint velocities \(\dot{q}\) from desired task-space velocities.

### Objective function

\[
\min_{\dot{q},\,s} \quad
\underbrace{\sum_i \left\| \dot{x}^d_i - J_i\dot{q} \right\|^2_{W_{1,i}}}_{\text{task tracking}}
+ \underbrace{\left\| \dot{q} \right\|^2_{W_2}}_{\text{velocity damping}}
+ \underbrace{\left\| \frac{\dot{q} - \dot{q}_{\text{now}}}{\Delta t} \right\|^2_{W_3}}_{\text{accel. damping}}
+ 1000\,\mathbf{1}^T s
\]

Rewritten in standard QP form \(\min \tfrac{1}{2}z^TPz + q^Tz\) with \(z = [\dot{q};\, s]\):

\[
P = \begin{bmatrix}
2\sum_i J_i^T W_{1,i} J_i + 2W_2 + \dfrac{2}{\Delta t^2}W_3 & 0 \\
0 & 0
\end{bmatrix}
\]

\[
q = \begin{bmatrix}
-2\sum_i J_i^T W_{1,i}\dot{x}^d_i - \dfrac{2}{\Delta t^2}W_3\dot{q}_{\text{now}} \\
1000
\end{bmatrix}
\]

### Weight parameters

| Parameter | API setter | Role |
| --- | --- | --- |
| \(W_{1,i} = \text{diag}(w_{\text{tracking},i})\) | `setQPIKTrackingGain` | Per-axis task velocity tracking penalty |
| \(W_2 = \text{diag}(w_{\text{vel}})\) | `setQPIKJointVelGain` | Joint velocity magnitude penalty (regularisation) |
| \(W_3 = \text{diag}(w_{\text{acc}})\) | `setQPIKJointAccGain` | Joint acceleration penalty (smoothness) |

!!! tip "Tuning guidance"
    - Increase \(W_{1,i}\) to track the task more aggressively.
    - Increase \(W_2\) to keep joints slow near limits.
    - Increase \(W_3\) to smooth out velocity commands between steps.
    - All three together form a weighted regulariser; if only one task is given and no constraints are active, the solution degenerates to a damped pseudoinverse.

### Bounds

\[
\dot{q}_{\min} \leq \dot{q} \leq \dot{q}_{\max}, \qquad s \geq 0
\]

Velocity limits are read from the URDF via Pinocchio.

### CBF Inequality Constraints

Safety constraints are enforced using **Control Barrier Functions (CBF)**.

#### 1. Joint angle limits (1st-order CBF)

Define \(h_{\min}(q) = q - q_{\min} \geq 0\) and \(h_{\max}(q) = q_{\max} - q \geq 0\).
The 1st-order CBF conditions with soft-slack \(s \geq 0\) yield:

\[
\begin{bmatrix} I & I \\ -I & I \end{bmatrix}
\begin{bmatrix} \dot{q} \\ s \end{bmatrix}
\geq
\begin{bmatrix} -\alpha(q - q_{\min}) \\ -\alpha(q_{\max} - q) \end{bmatrix}
\]

#### 2. Singularity avoidance (1st-order CBF)

Define \(h_{\text{sing}}(q) = w(q) - w_{\min} \geq 0\) where \(w(q) = \sqrt{\det(JJ^T)}\) is the Yoshikawa manipulability index.

\[
\nabla_q w^T\dot{q} + s \geq -\alpha\bigl(w(q) - w_{\min}\bigr)
\]

The gradient \(\nabla_q w\) is computed analytically by Pinocchio.

#### 3. Self-collision avoidance (1st-order CBF)

Define \(h_{\text{col}}(q) = d_{\min}(q) - d_{\min}^{\text{safe}} \geq 0\) where \(d_{\min}\) is the minimum pairwise link distance.

\[
\nabla_q d_{\min}^T\dot{q} + s \geq -\alpha\bigl(d_{\min}(q) - d_{\min}^{\text{safe}}\bigr)
\]

!!! note "Exponential filter on collision gradient"
    The self-collision gradient \(\nabla_q d_{\min}\) is filtered with a first-order exponential filter (\(\alpha_{\text{filter}} = 0.2\) hard-coded) to smooth discontinuous jumps when the closest collision pair changes between control steps.

!!! warning "Use primitive collision geometries — mesh shapes cause real-time deadline misses"
    Self-collision distance \(d_{\min}\) is computed every control step via **Pinocchio + FCL** `computeMinimumDistance()`.
    When the URDF `<collision>` elements reference **mesh files** (STL / OBJ / DAE), FCL must perform full GJK/EPA on every triangle — this can take **several milliseconds per link pair**, easily exceeding a 1 ms control budget.

    **Strongly recommended**: replace every `<collision>` mesh with a **primitive shape** in the URDF/SRDF:

    | FCL primitive | URDF tag |
    |---|---|
    | Sphere | `<sphere radius="..."/>` |
    | Cylinder | `<cylinder radius="..." length="..."/>` |
    | Box | `<box size="... ... ..."/>` |
    | Capsule | `<capsule radius="..." length="..."/>` *(vendor extension)* |

    Primitive-vs-primitive distance queries run in **O(1)** closed form. A robot with 7 links and all-primitive collision models typically completes the full pairwise check in **< 0.05 ms**.

---

## QPID — QP Inverse Dynamics {#qpid-objective}

QPID computes joint accelerations \(\ddot{q}\) and torques \(\tau\) from desired task-space accelerations.

### Objective function

\[
\min_{\ddot{q},\,\tau,\,s} \quad
\underbrace{\sum_i \left\| \ddot{x}^d_i - J_i\ddot{q} - \dot{J}_i\dot{q} \right\|^2_{W_{1,i}}}_{\text{task tracking}}
+ \underbrace{\left\| \ddot{q} \right\|^2_{W_2}}_{\text{accel. damping}}
+ \underbrace{\left\| \Delta t\,\ddot{q} + \dot{q} \right\|^2_{W_3}}_{\text{velocity damping}}
+ 100\,\mathbf{1}^T s
\]

In standard QP form \(z = [\ddot{q};\, \tau;\, s]\):

\[
P = \begin{bmatrix}
2\sum_i J_i^T W_{1,i} J_i + 2W_2 + 2\Delta t^2 W_3 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\]

\[
q = \begin{bmatrix}
-2\sum_i J_i^T W_{1,i}(\ddot{x}^d_i - \dot{J}_i\dot{q}) + 2\Delta t\, W_3\dot{q} \\
0 \\
100
\end{bmatrix}
\]

### Equality constraint — robot dynamics

\[
M(q)\,\ddot{q} + \text{nle}(q,\dot{q}) = \tau
\quad\Leftrightarrow\quad
\begin{bmatrix} M & -I & 0 \end{bmatrix}
\begin{bmatrix} \ddot{q} \\ \tau \\ s \end{bmatrix}
= -\text{nle}
\]

where \(\text{nle} = c(q,\dot{q}) + g(q)\).

### Bounds

\[
\tau_{\min} \leq \tau \leq \tau_{\max}, \qquad s \geq 0
\]

Torque limits are read from the URDF.

### CBF Inequality Constraints

#### 1. Joint angle limits (2nd-order CBF)

\[
\begin{bmatrix} I & 0 & I \\ -I & 0 & I \end{bmatrix}
\begin{bmatrix} \ddot{q} \\ \tau \\ s \end{bmatrix}
\geq
\begin{bmatrix} -2\alpha\dot{q} - \alpha^2(q - q_{\min}) \\ 2\alpha\dot{q} - \alpha^2(q_{\max} - q) \end{bmatrix}
\]

#### 2. Joint velocity limits (1st-order CBF)

\[
\begin{bmatrix} I & 0 & I \\ -I & 0 & I \end{bmatrix}
\begin{bmatrix} \ddot{q} \\ \tau \\ s \end{bmatrix}
\geq
\begin{bmatrix} -\alpha(\dot{q} - \dot{q}_{\min}) \\ -\alpha(\dot{q}_{\max} - \dot{q}) \end{bmatrix}
\]

#### 3. Self-collision avoidance (2nd-order CBF)

Let \(\dot{h} = \nabla_q d_{\min}^T \dot{q}\) and \(\ddot{h} = (\dot{\nabla}_q d_{\min})^T\dot{q} + \nabla_q d_{\min}^T\ddot{q}\).

\[
\nabla_q d_{\min}^T\ddot{q} + s
\geq -(\dot{\nabla}_q d_{\min})^T\dot{q}
    - 2\alpha\,\nabla_q d_{\min}^T\dot{q}
    - \alpha^2(d_{\min} - d_{\min}^{\text{safe}})
\]

!!! note "Singularity CBF in QPID"
    A singularity CBF row is allocated but **currently inactive** in the QPID implementation.

!!! warning "Use primitive collision geometries — mesh shapes cause real-time deadline misses"
    The same Pinocchio + FCL `computeMinimumDistance()` call used in QPIK is also performed every step in QPID.
    See the [identical warning in the QPIK section](#qpik-cbf-constraints) — the recommendation to replace `<collision>` meshes with primitive shapes applies equally here.

---

## Hierarchical QP (HQPIK / HQPID) {#hierarchical-qp}

HQP solves a sequence of QP problems, one per priority level. Higher-priority task results are enforced as **strict equality constraints** for lower-priority solvers.

### Algorithm

For each priority level \(k = 0, 1, \ldots, K-1\):

**1. Cost function** (same structure as QPIK/QPID):

\[
\min_{\dot{q},\,s^{(k)}} \quad
\sum_i \left\| \dot{x}^d_{k,i} - J_{k,i}\dot{q} \right\|^2_{W_{1,k,i}}
+ \left\| \dot{q} \right\|^2_{W_2}
+ \left\| \frac{\dot{q} - \dot{q}_{\text{prev}}}{\Delta t} \right\|^2_{W_3}
\]

**2. Equality constraint** (preservation of all higher-priority tasks):

\[
J_{\text{eq}}^{(k)} \dot{q} = v_{\text{eq}}^{(k)}
\]

where \(J_{\text{eq}}^{(k)}\) stacks the Jacobians of levels \(0,\ldots,k-1\) and
\(v_{\text{eq}}^{(k)} = J_{\text{eq}}^{(k)} \dot{q}_{k-1}^*\) uses the optimal velocity from the previous level.

**3. Shared inequality constraints** (joint limits, singularity, self-collision) are computed once per cycle and applied identically to all levels.

### Input format

```cpp
// C++
std::vector<std::map<std::string, drc::TaskSpaceData>> hierarchy;
hierarchy.push_back({{ "panda_hand",  high_priority_task }});  // level 0 (highest)
hierarchy.push_back({{ "panda_link3", lower_priority_task }});  // level 1

Eigen::VectorXd qdot_desired(dof);
robot_controller.HQPIKStep(hierarchy, qdot_desired);
```

The hierarchy is rebuilt automatically when the number of tasks per level changes.

---

## Choosing the Right Controller

| Controller | Level | Input required | CBF constraints | When to use |
| --- | --- | --- | --- | --- |
| `CLIK*` | velocity | \(\dot{x}^d\) | none | Simple tracking, no joint-safety requirement |
| `OSF*` | torque | \(\ddot{x}^d\) | none | Dynamics-aware torque control without constraints |
| `QPIK*` | velocity | \(\dot{x}^d\) | ✓ | Task tracking with safety constraints |
| `QPID*` | torque | \(\ddot{x}^d\) | ✓ | Torque-level tracking with full constraint set |
| `HQPIK*` | velocity | hierarchy of \(\dot{x}^d\) | ✓ | Hard task priorities at velocity level |
| `HQPID*` | torque | hierarchy of \(\ddot{x}^d\) | ✓ | Hard task priorities at torque level |
