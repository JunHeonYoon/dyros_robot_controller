import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from .robot_data import RobotData


class RobotController(drc.MobileRobotController):
    """
    A Python wrapper for the C++ RobotController::Mobile::MobileBase class.
    
    This class computes wheel velocities based on desired base velocity
    using inverse kinematics Jacobians. Supports Differential, Mecanum, and Caster drive types.
    """
    def __init__(self, dt: float, robot_data: RobotData):
        """
        Constructor.

        Parameters:
            dt         : (float) Control loop time step in seconds.
            robot_data : (DataMobileBase) An instance of the Python MobileBase wrapper which contains the robot's kinematic model.

        Raises:
            TypeError: If the robot_data is not an instance of the Python MobileBase wrapper.
        """
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python MobileBase wrapper")
        self._robot_data = robot_data
        self._dt = float(dt)
        super().__init__(self._dt, self._robot_data)

    def compute_wheel_vel(self, base_vel: np.ndarray) -> np.ndarray:
        """
        Compute wheel velocities from desired base velocity using inverse kinematics.

        Parameters:
            base_vel : (np.ndarray) Desired base velocity [vx, vy, wz], size = 3.
        
        Returns:
            (np.ndarray) Computed wheel velocities [rad/s], size = number of wheels.
        """
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
        return super().VelocityCommand(desired_base_vel)