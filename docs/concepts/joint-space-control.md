# Joint-Space Control

Joint-space controllers operate entirely in the \(n\)-dimensional joint configuration space and are the simplest way to drive a manipulator to a desired configuration.

## Robot Dynamics

All joint-space torque controllers are grounded in the manipulator equation of motion:

\[
M(q)\,\ddot{q} + c(q,\dot{q}) + g(q) = \tau
\]

| Symbol | Dimension | Description |
| --- | --- | --- |
| \(q,\,\dot{q},\,\ddot{q}\) | \(\mathbb{R}^n\) | Joint position, velocity, acceleration |
| \(M(q)\) | \(\mathbb{R}^{n\times n}\) | Symmetric positive-definite mass matrix |
| \(c(q,\dot{q})\) | \(\mathbb{R}^n\) | Coriolis and centrifugal vector |
| \(g(q)\) | \(\mathbb{R}^n\) | Gravitational torque vector |
| \(\tau\) | \(\mathbb{R}^n\) | Actuator torque vector |

## PD Torque Control

### `moveJointTorqueStep(q_target, qdot_target)`

Computes the desired joint acceleration via a PD law and converts it to torque:

\[
\ddot{q}_d = K_p\,(q_d - q) + K_v\,(\dot{q}_d - \dot{q})
\]

\[
\tau = M(q)\,\ddot{q}_d + g(q)
\]

With `use_mass = false`, the mass matrix is replaced by the identity:

\[
\tau = \ddot{q}_d + g(q)
\]

**Gains** are set with [`setJointGain(Kp, Kv)`](../api/cpp.md#gain-setters).
- \(K_p \in \mathbb{R}^n\) acts as a diagonal spring: larger values pull the joint to \(q_d\) faster.
- \(K_v \in \mathbb{R}^n\) acts as a diagonal damper: larger values reduce overshoot and oscillation.

Default initial values: \(K_p = 400\,I_n\), \(K_v = 40\,I_n\).

!!! tip "Critical damping"
    Critical damping is achieved when \(K_v = 2\sqrt{K_p}\) element-wise (assuming unit inertia).
    For the defaults that gives \(K_v = 2\sqrt{400} = 40\), which matches the built-in defaults.

### `moveJointTorqueStep(qddot_target)`

Feeds a pre-computed desired acceleration directly:

\[
\tau = M(q)\,\ddot{q}_d + g(q)
\]

Use this when you already have a desired acceleration from another controller (e.g., task-space OSF or QPID output).

## Cubic Polynomial Trajectory

### `moveJointPositionCubic` / `moveJointVelocityCubic` / `moveJointTorqueCubic`

A cubic polynomial satisfies four boundary conditions and produces smooth, bounded velocity profiles.
For a single joint, the trajectory is:

\[
q(t) = a_0 + a_1\,s + a_2\,s^2 + a_3\,s^3, \qquad s = t - t_0 \in [0,\, T]
\]

The four coefficients are uniquely determined by:

\[
q(0) = q_{\text{init}},\quad
\dot{q}(0) = \dot{q}_{\text{init}},\quad
q(T) = q_{\text{target}},\quad
\dot{q}(T) = \dot{q}_{\text{target}}
\]

Solving gives:

\[
\begin{aligned}
a_0 &= q_{\text{init}} \\
a_1 &= \dot{q}_{\text{init}} \\
a_2 &= \frac{3(q_{\text{target}}-q_{\text{init}})}{T^2} - \frac{2\dot{q}_{\text{init}}+\dot{q}_{\text{target}}}{T} \\
a_3 &= \frac{-2(q_{\text{target}}-q_{\text{init}})}{T^3} + \frac{\dot{q}_{\text{init}}+\dot{q}_{\text{target}}}{T^2}
\end{aligned}
\]

The velocity profile \(\dot{q}(t)\) is the analytic derivative, and
`moveJointTorqueCubic` chains the cubic reference through `moveJointTorqueStep`.

## Gravity Compensation

Pure gravity compensation (gravity torques only, no acceleration term) is a special case:

\[
\tau = g(q)
\]

obtained by calling `moveJointTorqueStep(qddot_target = 0)`.

## API Summary

| Method | Output | Description |
| --- | --- | --- |
| `moveJointPositionCubic(...)` | \(q_d(t)\) | Cubic reference joint position |
| `moveJointVelocityCubic(...)` | \(\dot{q}_d(t)\) | Cubic reference joint velocity |
| `moveJointTorqueStep(q_d, qdot_d)` | \(\tau\) | PD torque from current pose error |
| `moveJointTorqueStep(qddot_d)` | \(\tau\) | Torque from desired acceleration |
| `moveJointTorqueCubic(...)` | \(\tau\) | Cubic trajectory + PD torque |
