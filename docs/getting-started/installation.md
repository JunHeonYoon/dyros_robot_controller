# Installation

## System Requirements

Required dependencies:

- Eigen3
- Pinocchio
- OSQP and OSQP-Eigen
- eigenpy
- Boost.Python

## Clone

```bash
git clone https://github.com/JunHeonYoon/dyros_robot_controller.git
```

## Build

```bash
cd dyros_robot_controller
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
make
make install
```

## Environment Variables

Add to `~/.bashrc`:

```bash
export dyros_robot_controller_DIR="<custom-folder>"
export LD_LIBRARY_PATH="$dyros_robot_controller_DIR/lib:$LD_LIBRARY_PATH"
```

## Python Bindings

After setting the environment variables, the `drc` package can be imported from Python:

```python
import drc
from drc.manipulator import RobotController
```

## Documentation Dependencies

Install the documentation dependencies from the repository root.

```bash
python3 -m pip install -r docs/requirements.txt
```
