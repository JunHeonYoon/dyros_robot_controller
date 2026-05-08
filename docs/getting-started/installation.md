# Installation

## System Requirements

The package is built as a ROS 2 package and expects a ROS 2 workspace layout.

Required dependencies:

- ROS 2 Humble
- Eigen3
- Pinocchio
- OSQP and OSQP-Eigen
- eigenpy
- Boost.Python

## Clone

Clone the repository into the `src` directory of a ROS 2 workspace.

```bash
cd ~/ros2_ws/src
git clone https://github.com/JunHeonYoon/dyros_robot_controller.git
```

If the package is part of a larger workspace, keep it under that workspace's `src` tree.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select dyros_robot_controller
source install/setup.bash
```

## Python Bindings

The Python package is installed as `drc` through `ament_python_install_package(drc)`. After sourcing the workspace, the package can be imported from Python.

```python
import drc
from drc.manipulator import RobotController
```

## Documentation Dependencies

Install the documentation dependencies from the repository root.

```bash
python3 -m pip install -r docs/requirements.txt
```
