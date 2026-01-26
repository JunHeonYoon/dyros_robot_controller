import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp
from drc import KinematicParam


class RobotData(drc_cpp.MobileRobotData):
    """
    A Python wrapper for the C++ RobotData::Mobile::MobileBase class.
    
    This class provides a general interface and shared logic for various mobile robot types
    (e.g., Differential, Mecanum, Caster). It supports state update, forward velocity computation,
    and Jacobian calculation based on the specified kinematic parameters.
    """
    def __init__(self, dt: float, param: KinematicParam):
        """
        Constructor.

        Parameters:
            dt : (float) Control loop time step in seconds.
            param : (KinematicParam) Kinematic parameter instance containing drive type and geometry.
        """
        self._dt = float(dt)
        if not isinstance(param, KinematicParam):
            raise TypeError("param must be a KinematicParam instance")
        self._param = param
        super().__init__(self._dt, self._param.cpp())
        self.wheel_num = super().getWheelNum()
    
    def get_verbose(self) -> str:
        """
        Prints debug information.

        Return:
            (str) Debug information.
        """
        return super().getVerbose()

    def get_dt(self) -> float:
        """
        Get the control time step.

        Return:
            (float) Control loop time step in seconds.
        """
        return float(super().getDt())
        
    def update_state(self, wheel_pos: np.ndarray, wheel_vel: np.ndarray) -> bool:
        """
        Update internal mobile robot data.

        Parameters:
            wheel_pos : (np.ndarray) Wheel positions [rad].
            wheel_vel : (np.ndarray) Wheel velocities [rad/s].

        Returns:
            (bool) True if state update is successful.
        """
        wheel_pos = wheel_pos.reshape(-1)
        wheel_vel = wheel_vel.reshape(-1)
        assert wheel_pos.size == self.wheel_num, f"Size of wheel_pos {wheel_pos.size} is not equal to # of Wheels {self.wheel_num}."
        assert wheel_vel.size == self.wheel_num, f"Size of wheel_vel {wheel_vel.size} is not equal to # of Wheels {self.wheel_num}."
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
        wheel_pos = wheel_pos.reshape(-1)
        wheel_vel = wheel_vel.reshape(-1)
        assert wheel_pos.size == self.wheel_num, f"Size of wheel_pos {wheel_pos.size} is not equal to # of Wheels {self.wheel_num}."
        assert wheel_vel.size == self.wheel_num, f"Size of wheel_vel {wheel_vel.size} is not equal to # of Wheels {self.wheel_num}."
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
        wheel_pos = wheel_pos.reshape(-1)
        assert wheel_pos.size == self.wheel_num, f"Size of wheel_pos {wheel_pos.size} is not equal to # of Wheels {self.wheel_num}."
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
