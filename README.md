# DYROS Robot Controller Package

**dyros_robot_controller** is a versatile ROS2-based control package that implements control algorithms for various types of robots, including mobile robots, manipulators, and mobile manipulators.  
It is designed to work seamlessly in both simulation and real-robot environments, enabling rapid development and testing of advanced control strategies.

---

## Key Features

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

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)  
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html) – for kinematics and dynamics computation  
- [OSQP](https://osqp.org/) and [OSQP-Eigen](https://github.com/robotology/osqp-eigen) – for fast Quadratic Programming solvers  

---


 # Installation
 ```bash
cd ros2_ws
git clone https://github.com/JunHeonYoon/dyros_robot_controller.git src
colcon build --symlink-install
source install/setup.bash
```

---
## Applications

`dyros_robot_controller` serves as a foundation for research and development in robot control, supporting:  

- Mobile robot kinematic control  
- Advanced manipulator control strategies  
- Whole-body control for mobile manipulators  

It enables fast prototyping of novel controllers and facilitates seamless integration between simulation and real-world robotics platforms.  

---



