# Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
#
# Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
# Seoul National University, under a research agreement with ETRI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp
from .robot_data import RobotData


class RobotController(drc_cpp.MobileRobotController):
    """
    A Python wrapper for the C++ RobotController::Mobile::MobileBase class.
    
    This class computes wheel velocities based on desired base velocity
    using inverse kinematics Jacobians. Supports Differential, Mecanum, and Caster drive types.
    """
    def __init__(self, robot_data: RobotData):
        """
        Constructor.

        Parameters:
            robot_data : (DataMobileBase) An instance of the Python MobileBase wrapper which contains the robot's kinematic model.

        Raises:
            TypeError: If the robot_data is not an instance of the Python MobileBase wrapper.
        """
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python MobileBase wrapper")
        self._robot_data = robot_data
        self._dt = float(self._robot_data.get_dt())
        super().__init__(self._robot_data)

    def compute_wheel_vel(self, base_vel: np.ndarray) -> np.ndarray:
        """
        Compute wheel velocities from desired base velocity using inverse kinematics.

        Parameters:
            base_vel : (np.ndarray) Desired base velocity [vx, vy, wz], size = 3.
        
        Returns:
            (np.ndarray) Computed wheel velocities [rad/s], size = number of wheels.
        """
        base_vel = base_vel.reshape(-1)
        assert base_vel.size == 3, f"Size of {base_vel.size} is not equal to 3."
        return super().computeWheelVel(base_vel)

    def compute_IK_jacobian(self) -> np.ndarray:
        """
        Compute inverse kinematics Jacobian (maps base velocity to wheel velocity).
        
        Returns:
            (np.ndarray) Jacobian matrix of size [wheel_num x 3].
        """
        return super().computeIKJacobian()

    def velocity_command(self, desired_base_vel: np.ndarray) -> np.ndarray:
        """
        Generate velocity command with optional constraints (e.g., saturation).
        
        Parameters:
            desired_base_vel : (np.ndarray) Desired base velocity [vx, vy, wz], size = 3.

        Returns:
            (np.ndarray) Wheel velocity command (possibly saturated), size = number of wheels.
        """
        desired_base_vel = desired_base_vel.reshape(-1)
        assert desired_base_vel.size == 3, f"Size of {desired_base_vel.size} is not equal to 3."
        return super().VelocityCommand(desired_base_vel)
