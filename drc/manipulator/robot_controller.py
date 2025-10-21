import numpy as np
from typing import Optional
import dyros_robot_controller_cpp_wrapper as drc
from .robot_data import RobotData

class RobotController(drc.ManipulatorRobotController):
    """
    A Python wrapper for the C++ RobotController::Manipulator::ManipulatorBase class.
    
    This class consists of functions that compute manipulator control inputs and helpers that generate smooth trajectories.
    Joint space functions compute control inputs to track desired joint positions, velocities, or accelerations.
    Task space functions compute control inputs to track desired position, velocity, or acceleration of a link.
    """
    def __init__(self, dt: float, robot_data: RobotData):
        """
        Constructor.

        Parameters:
            dt         : (float) Control loop time step in seconds.
            robot_data : (DataManipulatorBase) An instance of the Python ManipulatorBase wrapper which contains the robot's kinematic and dynamic parameters.

        Raises:
            TypeError: If the robot_data is not an instance of the Python ManipulatorBase wrapper.
        """
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python ManipulatorBase wrapper")

        self._dt = float(dt)
        self._robot_data = robot_data
        super().__init__(self._dt, self._robot_data)

    def set_joint_gain(self, kp: np.ndarray, kv: np.ndarray):
        """
        Set joint space PD gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
            kv : (np.ndarray) Derivative gains.
        """
        super().setJointGain(kp, kv)
        
    def set_joint_kp_gain(self, kp: np.ndarray):
        """
        Set joint space P gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains.
        """
        super().setJointKpGain(kp)
        
    def set_joint_kv_gain(self, kv: np.ndarray):
        """
        Set joint space D gains for the manipulator.

        Parameters:
            kv : (np.ndarray) Derivative gains.
        """
        super().setJointKvGain(kv)

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

    # ================================ Joint space Functions ================================
    def move_joint_position_cubic(self,
                                  q_target: np.ndarray,
                                  qdot_target: np.ndarray,
                                  q_init: np.ndarray,
                                  qdot_init: np.ndarray,
                                  current_time: float,
                                  init_time: float,
                                  duration: float,
                                  ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired joint configurations over the given duration.

        Parameters:
            q_target     : (np.ndarray) Desired manipulator joint positions at the end of the segment.
            qdot_target  : (np.ndarray) Desired manipulator joint velocities at the end of the segment.
            q_init       : (np.ndarray) Initial manipulator joint positions at the start of the segment.
            qdot_init    : (np.ndarray) Initial manipulator joint velocities at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.

        Returns:
            (np.ndarray) Desired joint positions.
        """
        return super().moveJointPositionCubic(q_target,
                                              qdot_target,
                                              q_init,
                                              qdot_init,
                                              current_time,
                                              init_time,
                                              duration, 
                                              )
        
    def move_joint_velocity_cubic(self,
                                  q_target: np.ndarray,
                                  qdot_target: np.ndarray,
                                  q_init: np.ndarray,
                                  qdot_init: np.ndarray,
                                  current_time: float,
                                  init_time: float,
                                  duration: float,
                                  ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired joint configurations over the given duration.

        Parameters:
            q_target     : (np.ndarray) Desired manipulator joint positions at the end of the segment.
            qdot_target  : (np.ndarray) Desired manipulator joint velocities at the end of the segment.
            q_init       : (np.ndarray) Initial manipulator joint positions at the start of the segment.
            qdot_init    : (np.ndarray) Initial manipulator joint velocities at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        return super().moveJointVelocityCubic(q_target,
                                              qdot_target,
                                              q_init,
                                              qdot_init,
                                              current_time,
                                              init_time,
                                              duration, 
                                              )

    # def move_joint_torque_step(self, q_target: np.ndarray, qdot_target: np.ndarray) -> np.ndarray:
    #     return super().moveJointTorqueStep(q_target, qdot_target)
    
    # def move_joint_torque_step(self, qddot_target: np.ndarray) -> np.ndarray:
    #     return super().moveJointTorqueStep(qddot_target)
    
    def move_joint_torque_step(self,
                               q_target:     Optional[np.ndarray] = None,
                               qdot_target:  Optional[np.ndarray] = None,
                               qddot_target: Optional[np.ndarray] = None,
                               ) -> np.ndarray:
        """
        Computes joint torques to achieve desired joint configurations using equations of motion and PD control law.

        Parameters:
            q_target     : (np.ndarray) [Required if qddot_target is None]
                            Desired manipulator joint positions.
            qdot_target  : (np.ndarray) [Required if qddot_target is None]
                            Desired manipulator joint velocities.
            qddot_target : (np.ndarray) [Required if q_target and qdot_target are None]
                            Desired manipulator joint accelerations.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        if qddot_target is not None:
            return super().moveJointTorqueStep(qddot_target)

        if q_target is not None and qdot_target is not None:
            return super().moveJointTorqueStep(q_target, qdot_target)

    def move_joint_torque_cubic(self,
                                q_target: np.ndarray,
                                qdot_target: np.ndarray,
                                q_init: np.ndarray,
                                qdot_init: np.ndarray,
                                current_time: float,
                                init_time: float,
                                duration: float,
                                ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired joint configurations over the given duration, then compute joint torques to follow the resulting trajectory.

        Parameters:
            q_target     : (np.ndarray) Desired manipulator joint positions at the end of the segment.
            qdot_target  : (np.ndarray) Desired manipulator joint velocities at the end of the segment.
            q_init       : (np.ndarray) Initial manipulator joint positions at the start of the segment.
            qdot_init    : (np.ndarray) Initial manipulator joint velocities at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        return super().moveJointTorqueCubic(q_target,
                                            qdot_target,
                                            q_init,
                                            qdot_init,
                                            current_time,
                                            init_time,
                                            duration,
                                            )

    # ================================ Task space Functions ================================
    def CLIK_step(self,
                  x_target: np.ndarray,
                  xdot_target: np.ndarray,
                  link_name: str,
                  null_qdot: Optional[np.ndarray] = None,
                  ) -> np.ndarray:
        """
        Computes joint velocity to achieve desired position & velocity of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy if provided.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.
            null_qdot   : (np.ndarray) [Optional] Desired joint velocity to be projected on null space.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        if null_qdot is None:
            return super().CLIKStep(x_target,
                                    xdot_target,
                                    link_name,
                                    )
        else:
            return super().CLIKStep(x_target,
                                    xdot_target,
                                    null_qdot,
                                    link_name,
                                    )

    def CLIK_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   null_qdot: Optional[np.ndarray] = None,
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint velocities to follow the resulting trajectory.

        Parameters:
            x_target     : (np.ndarray) Desired position of a link at the end of the segment.
            xdot_target  : (np.ndarray) Desired velocity of a link at the end of the segment.
            x_init       : (np.ndarray) Initial position of a link at the start of the segment.
            xdot_init    : (np.ndarray) Initial velocity of a link at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            link_name    : (str) Name of the link.
            null_qdot    : (np.ndarray) [Optional] Desired joint velocity to be projected on null space.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        if null_qdot is None:
            return super().CLIKCubic(x_target, 
                                     xdot_target,
                                     x_init,   
                                     xdot_init,
                                     current_time, 
                                     init_time, 
                                     duration,
                                     link_name
                                     )
        else:
            return super().CLIKCubic(x_target, 
                                     xdot_target,
                                     x_init,   
                                     xdot_init,
                                     current_time, 
                                     init_time, 
                                     duration,
                                     null_qdot,
                                     link_name
                                     )


    def OSF(self,
            xddot_target: np.ndarray,
            link_name: str,
            null_torque: Optional[np.ndarray] = None,
            ) -> np.ndarray:
        """
        Computes joint torque to achieve desired acceleration of a link using operational space control, projecting null_torque into null space to exploit redundancy if provided.

        Parameters:
            xddot_target : (np.ndarray) Desired acceleration of a link.
            link_name    : (str) Name of the link.
            null_torque  : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        if null_torque is None:
            return super().OSF(xddot_target, link_name)
        else:
            return super().OSF(xddot_target, null_torque, link_name)

    def OSF_step(self,
                 x_target: np.ndarray,
                 xdot_target: np.ndarray,
                 link_name: str,
                 null_torque: Optional[np.ndarray] = None,
                 ) -> np.ndarray:
        """
        Computes joint torque to achieve desired position & velocity of a link using operational space control, projecting null_torque into null space to exploit redundancy if provided.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.
            null_torque : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        if null_torque is None:
            return super().OSFStep(x_target, xdot_target, link_name)
        else: 
            return super().OSFStep(x_target, xdot_target, null_torque, link_name)

    def OSF_cubic(self,
                  x_target: np.ndarray,
                  xdot_target: np.ndarray,
                  x_init: np.ndarray,
                  xdot_init: np.ndarray,
                  current_time: float,
                  init_time: float,
                  duration: float,
                  link_name: str,
                  null_torque: Optional[np.ndarray] = None,
                  ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint torques to follow the resulting trajectory.

        Parameters:
            x_target     : (np.ndarray) Desired position of a link at the end of the segment.
            xdot_target  : (np.ndarray) Desired velocity of a link at the end of the segment.
            x_init       : (np.ndarray) Initial position of a link at the start of the segment.
            xdot_init    : (np.ndarray) Initial velocity of a link at the start of the segment.
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            link_name    : (str) Name of the link.
            null_torque  : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        if null_torque is None:
            return super().OSFCubic(x_target,
                                    xdot_target,
                                    x_init,
                                    xdot_init,
                                    current_time,
                                    init_time,
                                    duration,
                                    link_name,
                                    )
        else:
            return super().OSFCubic(x_target,
                                    xdot_target,
                                    x_init,
                                    xdot_init,
                                    current_time,
                                    init_time,
                                    duration,
                                    null_torque,
                                    link_name,
                                    )
    def QPIK(self, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
        """
        Computes joint velocities to achieve desired velocity of a link by solving inverse kinematics QP.

        Parameters:
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        return super().QPIK(xdot_target, link_name)

    def QPIK_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
        """
        Computes joint velocities to achieve desired position & velocity of a link by solving inverse kinematics QP.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (np.ndarray) Desired joint velocities.
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
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute joint velocities using QP to follow the resulting trajectory.

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
            (np.ndarray) Desired joint velocities.
        """
        return super().QPIKCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name,
                                 )

    def QPID(self, xddot_target: np.ndarray, link_name: str) -> np.ndarray:
        """
        Computes joint torques to achieve desired acceleration of a link by solving inverse dynamics QP.

        Parameters:
            xddot_target : (np.ndarray) Desired acceleration of a link.
            link_name    : (str) Name of the link.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        return super().QPID(xddot_target, link_name)

    def QPID_step( self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
        """
        Computes joint torques to achieve desired position & velocity of a link by solving inverse dynamics QP.

        Parameters:
            x_target    : (np.ndarray) Desired position of a link.
            xdot_target : (np.ndarray) Desired velocity of a link.
            link_name   : (str) Name of the link.

        Returns:
            (np.ndarray) Desired joint torques.
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
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute joint torques using QP to follow the resulting trajectory.

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
            (np.ndarray) Desired joint torques.
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