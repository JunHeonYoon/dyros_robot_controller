import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from drc import KinematicParam


class RobotData(drc.MobileRobotData):
    """
    A Python wrapper for the C++ RobotData::Mobile::MobileBase class.
    
    This class provides a general interface and shared logic for various mobile robot types
    (e.g., Differential, Mecanum, Caster). It supports state update, forward velocity computation,
    and Jacobian calculation based on the specified kinematic parameters.
    """
    def __init__(self, param: KinematicParam):
        """
        Constructor.

        Parameters:
            param : (KinematicParam) Kinematic parameter instance containing drive type and geometry.
        """
        if not isinstance(param, KinematicParam):
            raise TypeError("param must be a KinematicParam instance")
        self._param = param
        super().__init__(self._param.cpp())
    
    def get_verbose(self) -> str:
        """
        Prints debug information.

        Return:
            (str) Debug information.
        """
        return super().getVerbose()
        
    def update_state(self, wheel_pos: np.ndarray, wheel_vel: np.ndarray) -> bool:
        """
        Update internal mobile robot data.

        Parameters:
            wheel_pos : (np.ndarray) Wheel positions [rad].
            wheel_vel : (np.ndarray) Wheel velocities [rad/s].

        Returns:
            (bool) True if state update is successful.
        """
        return bool(super().updateState(np.asarray(wheel_pos), np.asarray(wheel_vel)))
    
    # ================================ Compute Functions ================================
    def compute_base_vel(self, wheel_pos: np.ndarray, wheel_vel: np.ndarray) -> np.ndarray:
        """
        Compute the robot base velocity in the base frame.

        Parameters:
            wheel_pos : (np.ndarray) Wheel positions [rad].
            wheel_vel : (np.ndarray) Wheel velocities [rad/s].

        Returns:
            (np.ndarray) Base velocity vector [vx, vy, wz].
        """
        return np.asarray(super().computeBaseVel(np.asarray(wheel_pos), np.asarray(wheel_vel)))

    def compute_fk_jacobian(self, wheel_pos: np.ndarray) -> np.ndarray:
        """
        Compute the forward kinematics Jacobian of the base.
        Maps wheel velocities to base velocity.

        Parameters:
            wheel_pos : (np.ndarray) Wheel positions [rad].

        Returns:
            (np.ndarray) Base velocity Jacobian matrix.
        """
        return np.asarray(super().computeFKJacobian(np.asarray(wheel_pos)))
    
    # ================================ Get Functions ================================
    def get_kine_param(self) -> KinematicParam:
        """
        Get the kinematic parameters used for this base.

        Returns:
            (KinematicParam) Reference to KinematicParam.
        """
        return self.kine_param
    
    def get_wheel_num(self) -> int:
        """
        Get the number of wheels.

        Returns:
            (int) Integer number of wheels.
        """
        return int(super().getWheelNum())

    def get_wheel_pos(self) -> np.ndarray:
        """
        Get the current wheel positions.

        Returns:
            (np.ndarray) Vector of wheel positions.
        """
        return np.asarray(super().getWheelPosition())

    def get_wheel_vel(self) -> np.ndarray:
        """
        Get the current wheel velocities.

        Returns:
            (np.ndarray) Vector of wheel velocities.
        """
        return np.asarray(super().getWheelVelocity())

    def get_base_vel(self) -> np.ndarray:
        """
        Get the last computed base velocity.

        Returns:
            (np.ndarray) Base velocity vector [vx, vy, wz].
        """
        return np.asarray(super().getBaseVel())

    def get_FK_jacobian(self) -> np.ndarray:
        """
        Get the last computed base Jacobian.

        Returns:
            (np.ndarray) Jacobian matrix from wheel velocity to base velocity.
        """
        return np.asarray(super().getFKJacobian())