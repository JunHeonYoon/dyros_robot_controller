import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from .robot_data import RobotData


class RobotController(drc.MobileManipulatorRobotController):
    """
    A Python wrapper for the C++ RobotController::MobileManipulator::MobileManipulatorBase class.
    
    This class consists of functions that compute control inputs for mobile manipulator and helpers that generate smooth trajectories.
    Joint space functions compute control inputs of the manipulator to track desired joint positions, velocities, or accelerations.
    Task space functions compute control inputs of the whole body to track desired position or velocity of a link.
    """
    def __init__(self, dt: float, robot_data: RobotData):
        """
        Constructor.

        Parameters:
            dt         : (float) Control loop time step in seconds.
            robot_data : (DataMomaBase) An instance of the Python MobileManipulatorBase wrapper which contains the robot's kinematic and dynamic parameters.

        Raises:
            TypeError: If the robot_data is not an instance of the Python MobileManipulatorBase wrapper.
        """
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python MobileManipulatorBase")
        self._dt = float(dt)
        self._robot_data = robot_data
        super().__init__(self._dt, self._robot_data)

    def set_manipulator_joint_gain(self, kp: np.ndarray, kv: np.ndarray):
        """
        Set joint space PD gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
            kv : (np.ndarray) Derivative gains.
        """
        super().setManipulatorJointGain(kp, kv)

    def set_manipulator_joint_kp_gain(self, kp: np.ndarray):
        """
        Set joint space P gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
        """
        super().setManipulatorJointKpGain(kp)
        
    def set_manipulator_joint_kv_gain(self, kv: np.ndarray):
        """
        Set joint space D gains for the manipulator.

        Parameters:
            kv : (np.ndarray) Derivative gains.
        """
        super().setManipulatorJointKvGain(kv)

    def set_task_gain(self, kp: np.ndarray, kv: np.ndarray):
        """
        Set task space PD gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
            kv : (np.ndarray) Derivative gains.
        """
        super().setTaskGain(kp, kv)
        
    def set_task_kp_gain(self, kp: np.ndarray):
        """
        Set task space P gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
        """
        super().setTaskKpGain(kp)
        
    def set_task_kv_gain(self, kv: np.ndarray):
        """
        Set task space D gains for the manipulator.

        Parameters:
            kv : (np.ndarray) Derivative gains.
        """
        super().setTaskKpGain(kv)
        
    def set_QPIK_gain(self, w_tracking: np.ndarray, w_damping: np.ndarray):
        """
        Set the wight vector for the cost terms of the QPIK
        
        Parameters:
            w_tracking : (np.ndarray) Weight for task velocity tracking; its size must be 6.
            w_damping : (np.ndarray) Weight for joint velocity damping; its size must same as actuator dof.
        """
        super().setQPIKGain(w_tracking, w_damping)
        
    def set_QPID_gain(self, w_tracking: np.ndarray, w_vel_damping: np.ndarray, w_acc_damping: np.ndarray):
        """
        Set the wight vector for the cost terms of the QPID
        
        Parameters:
            w_tracking : (np.ndarray) Weight for task acceleration tracking; its size must be 6.
            w_vel_damping : (np.ndarray) Weight for joint velocity damping; its size must same as actuator dof.
            w_acc_damping : (np.ndarray) Weight for joint acceleration damping; its size must same as actuator dof.
        """
        super().setQPIDGain(w_tracking, w_vel_damping, w_acc_damping)

    # ================================ Joint space Functions ================================        

    def move_manipulator_joint_position_cubic(self,
                                              q_mani_target: np.ndarray,
                                              qdot_mani_target: np.ndarray,
                                              q_mani_init: np.ndarray,
                                              qdot_mani_init: np.ndarray,
                                              current_time: float,
                                              init_time: float,
                                              duration: float,
                                              ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration.

        Parameters:
            q_mani_target     : (np.ndarray) Desired manipulator joint positions at the end of the segment.
            qdot_mani_target  : (np.ndarray) Desired manipulator joint velocities at the end of the segment.
            q_mani_init       : (np.ndarray) Initial manipulator joint positions at the start of the segment.
            qdot_mani_init    : (np.ndarray) Initial manipulator joint velocities at the start of the segment.
            current_time      : (float) Current time.
            init_time         : (float) Start time of the segment.
            duration          : (float) Time duration.

        Returns:
            (np.ndarray) Desired manipulator joint positions.
        """
        return super().moveManipulatorJointPositionCubic(q_mani_target,
                                                         qdot_mani_target,
                                                         q_mani_init,
                                                         qdot_mani_init,
                                                         current_time,
                                                         init_time,
                                                         duration,
                                                         )

    # def move_manipulator_joint_torque_step(self, q_mani_target: np.ndarray, qdot_mani_target: np.ndarray) -> np.ndarray:
    #     return super().moveManipulatorJointTorqueStep(q_mani_target, qdot_mani_target)
    
    # def move_manipulator_joint_torque_step(self, qddot_mani_target: np.ndarray) -> np.ndarray:
    #     return super().moveManipulatorJointTorqueStep(qddot_mani_target)
    def move_manipulator_joint_torque_step(self,
                                           q_mani_target:     np.ndarray | None = None,
                                           qdot_mani_target:  np.ndarray | None = None,
                                           qddot_mani_target: np.ndarray | None = None,
                                           use_mass:          bool              = True,
                                           ) -> np.ndarray:
        """
        Computes manipulator joint torques to achieve desired manipulator joint configurations using equations of motion and PD control law.

        Parameters:
            q_mani_target     : (np.ndarray) [Required if qddot_mani_target is None]
                                Desired manipulator joint positions.
            qdot_mani_target  : (np.ndarray) [Required if qddot_mani_target is None]
                                Desired manipulator joint velocities.
            qddot_mani_target : (np.ndarray) [Required if q_mani_target and qdot_mani_target are None]
                                Desired manipulator joint accelerations.
            use_mass          : (bool) Whether use mass matrix.

        Returns:
            (np.ndarray) Desired manipulator joint torques.
        """
        if qddot_mani_target is not None:
            return super().moveManipulatorJointTorqueStep(qddot_mani_target, use_mass)

        if q_mani_target is not None and qdot_mani_target is not None:
            return super().moveManipulatorJointTorqueStep(q_mani_target, qdot_mani_target, use_mass)

    def move_manipulator_joint_torque_cubic(self,
                                            q_mani_target: np.ndarray,
                                            qdot_mani_target: np.ndarray,
                                            q_mani_init: np.ndarray,
                                            qdot_mani_init: np.ndarray,
                                            current_time: float,
                                            init_time: float,
                                            duration: float,
                                            use_mass: bool = True
                                            ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration, then compute manipulator joint torques to follow the resulting trajectory.

        Parameters:
            q_mani_target     : (np.ndarray) Desired manipulator joint positions at the end of the segment.
            qdot_mani_target  : (np.ndarray) Desired manipulator joint velocities at the end of the segment.
            q_mani_init       : (np.ndarray) Initial manipulator joint positions at the start of the segment.
            qdot_mani_init    : (np.ndarray) Initial manipulator joint velocities at the start of the segment.
            current_time      : (float) Current time.
            init_time         : (float) Start time of the segment.
            duration          : (float) Time duration.
            use_mass          : (bool) Whether use mass matrix.

        Returns:
            (np.ndarray) Desired manipulator joint torques.
        """
        return super().moveManipulatorJointTorqueCubic(q_mani_target,
                                                       qdot_mani_target,
                                                       q_mani_init,
                                                       qdot_mani_init,
                                                       current_time,
                                                       init_time,
                                                       duration,
                                                       use_mass,
                                                       )

    # ================================ Task space Functions ================================

    def QPIK(self, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        """
        Computes velocities for mobile base and manipulator joints to achieve desired velocity of a link by solving inverse kinematics QP.

        Parameters:
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base velocities, Output optimal manipulator joint velocities.
        """
        return super().QPIK(xdot_target, link_name)

    def QPIK_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        """
        Computes velocities for mobile base and manipulator joints to achieve desired position & velocity of a link by solving inverse kinematics QP.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base velocities, Output optimal manipulator joint velocities.
        """
        return super().QPIKStep(x_target, xdot_target, link_name)

    def QPIK_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   ) -> tuple[np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute velocities for mobile base and manipulator joints using QP to follow the resulting trajectory.

        Parameters:
            x_target     : (np.ndarray) Desired position of a link at the end of the segment.
            xdot_target  : (np.ndarray) Desired velocity of a link at the end of the segment.
            x_init       : (np.ndarray) Initial position of a link at the start of the segment.
            xdot_init    : (np.ndarray) Initial velocity of a link at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            link_name    : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base velocities, Output optimal manipulator joint velocities.
        """
        return super().QPIKCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name
                                 )

    def QPID(self, xddot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration of a link by solving inverse dynamics QP.

        Parameters:
            xddot_target : (np.ndarray) Desired acceleration of a link.
            link_name   : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base accelerations, Output optimal manipulator joint torques.
        """
        return super().QPID(xddot_target, link_name)

    def QPID_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired position & velocity of a link by solving inverse dynamics QP.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base accelerations, Output optimal manipulator joint torques.
        """
        return super().QPIDStep(x_target, xdot_target, link_name)

    def QPID_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   ) -> tuple[np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute mobile base accelerations and manipulator joint torques using QP to follow the resulting trajectory.

        Parameters:
            x_target     : (np.ndarray) Desired position of a link at the end of the segment.
            xdot_target  : (np.ndarray) Desired velocity of a link at the end of the segment.
            x_init       : (np.ndarray) Initial position of a link at the start of the segment.
            xdot_init    : (np.ndarray) Initial velocity of a link at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            link_name    : (str) Name of the link.

        Returns:
            (tuple[np.ndarray, np.ndarray]) Output optimal mobile base accelerations, Output optimal manipulator joint torques.
        """
        return super().QPIDCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name,
                                 )
