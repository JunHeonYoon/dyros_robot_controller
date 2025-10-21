# Dyros Robot Controller

**Dyros Robot Controller** is a versatile ROS-based control package that implements control algorithms for various types of robots, including mobile robots, manipulators, and mobile manipulators.  
It is designed to work seamlessly in both simulation and real-robot environments, enabling rapid development and testing of advanced control strategies.
Documentation of the **Dyros Robot Controller** is available [here](https://www.notion.so/dyros_robot_controller-28c454d594108063817bc141f5668e0f#28c454d5941080999042f30fb446355a).

---

## Key Features
| ClearPath Husky                     | Summit XLS                      |DYROS PCV                    |
| ----------------------------------- | ------------------------------- |---------------------------- |
| ![husky_model](imgs/husky.gif)      | ![xls_model](imgs/xls.gif)      |![pcv_model](imgs/pcv.gif)   |

| Franka FR3                     | Universal Robots 5e             |
| ------------------------------ | ------------------------------- |
| ![fr3_model](imgs/fr3.gif)     | ![xls_model](imgs/ur5.gif)      | 

| Husky FR3                      | XLS FR3                         |
| ------------------------------ | ------------------------------- |
| Comming Soon                   | Comming Soon                    |

- **Mobile Robots**
  - Supports differential drive, mecanum, and caster wheel configurations  
  - Provides kinematics-based Forward/Inverse Kinematics  

- **Manipulators**
  - Joint-space position, velocity, and torque control  
  - Task-space control using Jacobian-based CLIK (Closed-loop Inverse Kinematics)  
  - Dynamic control using OSF (Operational Space Formulation)  
  - QP-based controllers that handle constraints (joint limits, velocity, collision avoidance, etc.)  

- **Mobile Manipulators**
  - Whole-body torque-based controller  

- **Simulation and Real-World Integration**
  - Validated in simulation with:  
    - Mobile robots: ClearPath Husky, Summit XL, 4-wheel powered caster vehicle  
    - Manipulators: Franka FR3, UR5  
    - Mobile manipulators: Husky-FR3, XLS-FR3  
  - Real robot integration in progress  

---

## Dependencies

- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)  
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) – for kinematics and dynamics computation  
- [OSQP](https://osqp.org/) and [OSQP-Eigen](https://github.com/robotology/osqp-eigen) – for fast Quadratic Programming solvers  

---
## Installation

```bash
cd catkin_ws
git clone https://github.com/YoungWook0533/dyros_robot_controller.git src
catkin build
source devel/setup.bash
```

## Applications

`dyros_robot_controller` serves as a foundation for research and development in robot control, supporting:  

- Mobile robot kinematic control  
- Advanced manipulator control strategies  
- Whole-body control for mobile manipulators

- ## Examples

In the [examples](https://github.com/YoungWook0533/dyros_robot_controller/tree/main/examples) directory, we provide some basic examples of using dyros robot controller in Python and C++.
It enables fast prototyping of novel controllers and facilitates seamless integration between simulation and real-world robotics platforms.  

---
