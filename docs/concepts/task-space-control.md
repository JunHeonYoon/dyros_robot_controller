# Task-Space Control

Task-space (operational-space) controllers command a link's Cartesian pose or velocity rather than individual joint angles.
Two classical formulations are provided: **CLIK** (velocity-level) and **OSF** (torque-level).

## Background: Jacobian and Task-Space Error

The geometric Jacobian \(J_i(q) \in \mathbb{R}^{6 \times n}\) maps joint velocities to the 6D spatial velocity of link \(i\):

\[
\dot{x}_i = J_i(q)\,\dot{q}, \qquad \dot{x}_i = \begin{bmatrix} v_i \\ \omega_i \end{bmatrix}
\]

For the **Step** and **Cubic** variants the controller needs a **pose error** \(e_x \in \mathbb{R}^6\):

\[
e_x = \begin{bmatrix} p_d - p \\ \frac{1}{2}(R_d r_1 \times r_1 + R_d r_2 \times r_2 + R_d r_3 \times r_3) \end{bmatrix}
\]

where \(p\) is the current position, \(R\) is the current rotation matrix with column vectors \(r_k\), and \(R_d\) is the desired rotation matrix.
This is the angle-axis–based orientation error that vanishes when \(R = R_d\).

---

## CLIK — Closed-Loop Inverse Kinematics {#clik}

CLIK maps task-space velocity commands to joint velocity commands using the Moore-Penrose pseudoinverse.

### Core formula (`CLIK`)

Given desired task velocity \(\dot{x}^d_i\) for each link \(i\), the stacked Jacobian \(J \in \mathbb{R}^{6k \times n}\) is formed from all \(k\) controlled links and the joint velocity is:

\[
\boxed{
\dot{q} = J^+\,\dot{x}^d + (I - J^+ J)\,\dot{q}_{\text{null}}
}
\]

| Term | Description |
| --- | --- |
| \(J^+\) | Pseudoinverse computed with Complete Orthogonal Decomposition (COD); robust near singularities |
| \((I - J^+ J)\) | Null-space projector; maps \(\dot{q}_{\text{null}}\) into the null space of \(J\) without disturbing task tracking |
| \(\dot{q}_{\text{null}}\) | Optional secondary joint-velocity objective (e.g. joint-limit centering, posture control) |

### With pose feedback (`CLIKStep`)

Adds a proportional error term to close the kinematic loop and correct position drift:

\[
\dot{x}^d_{\text{cmd},i} = K_{p,i}\,e_{x,i} + \dot{x}^d_i
\]

\[
\dot{q} = J^+\,\dot{x}^d_{\text{cmd}} + (I - J^+ J)\,\dot{q}_{\text{null}}
\]

**Gain** \(K_p \in \mathbb{R}^6\) is set with [`setIKGain`](../api/cpp.md#gain-setters).
Higher values converge faster but may amplify noise; a typical value is in the range \(1\)–\(10\) for position and orientation channels.

### With cubic trajectory (`CLIKCubic`)

First generates a cubic polynomial reference trajectory (see [Cubic polynomial](joint-space-control.md#cubic-polynomial-trajectory)):

\[
x_d(t),\; \dot{x}_d(t) \;\leftarrow\; \text{cubic\_spline}(x_{\text{init}},\, \dot{x}_{\text{init}},\, x_{\text{target}},\, \dot{x}_{\text{target}},\, T)
\]

then passes the result through `CLIKStep`.

### Singularity and CLIK

At a kinematic singularity \(J\) becomes rank-deficient and \(J^+\) amplifies noise.
For singularity-robust IK with hard joint limits and collision avoidance, prefer [QPIK](qp-and-hqp.md#qpik-objective) instead.

---

## OSF — Operational Space Formulation {#osf}

OSF operates at the torque level and accounts for robot dynamics.
It is the natural torque-level complement to CLIK.

### Task-space mass matrix

The key quantity is the **task-space (operational-space) inertia matrix**:

\[
\Lambda = \bigl(J\,M^{-1}J^T\bigr)^{-1} \in \mathbb{R}^{6k \times 6k}
\]

and the **dynamically consistent pseudoinverse** of \(J^T\):

\[
J^{T\dagger} = \Lambda\,J\,M^{-1} \in \mathbb{R}^{6k \times n}
\]

### Core formula (`OSF`)

Given desired task-space acceleration \(\ddot{x}^d\) and an optional null-space torque \(\tau_{\text{null}}\):

\[
\boxed{
\tau = J^T \Lambda\,\ddot{x}^d + N^T\,\tau_{\text{null}} + g(q)
}
\]

where the null-space projector is:

\[
N = I_n - J^T J^{T\dagger}
\]

| Term | Description |
| --- | --- |
| \(J^T \Lambda\,\ddot{x}^d\) | Task-space force mapped to joint torques |
| \(N^T\,\tau_{\text{null}}\) | Null-space torque (e.g., joint damping or posture control) |
| \(g(q)\) | Gravity compensation — added automatically |

### With pose feedback (`OSFStep`)

Computes the task-space acceleration command from position and velocity errors:

\[
\ddot{x}^d_{\text{cmd},i} = K_{p,i}\,e_{x,i} + K_{v,i}\,e_{\dot{x},i} + \ddot{x}^d_i
\]

where the velocity error is \(e_{\dot{x},i} = \dot{x}^d_i - \dot{x}_i\).

**Gains** \(K_p, K_v \in \mathbb{R}^6\) are set with [`setIDGain`](../api/cpp.md#gain-setters).

### With cubic trajectory (`OSFCubic`)

Generates a cubic reference trajectory then passes through `OSFStep`, identical in structure to `CLIKCubic`.

---

## Choosing CLIK vs OSF vs QP

| Criterion | CLIK | OSF | QPIK / QPID |
| --- | --- | --- | --- |
| Command type | joint velocity | joint torque | joint velocity / torque |
| Dynamics-aware | no | yes | yes (QPID) |
| Joint limit enforcement | none | none | CBF constraints |
| Singularity avoidance | pseudoinverse only | pseudoinverse only | CBF constraint |
| Self-collision avoidance | none | none | CBF constraint |
| Multiple tasks | stacked Jacobian | stacked Jacobian | weighted or hierarchical |
| Computational cost | low | low | medium (QP solver) |

For applications where safety constraints matter, use [QPIK or QPID](qp-and-hqp.md).
