import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp
from .robot_data import RobotData
from drc import TaskSpaceData

class RobotController(drc_cpp.ManipulatorRobotController):
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

    def set_joint_gain(self, 
                       kp: np.ndarray | None = None, 
                       kv: np.ndarray | None = None,
                       ):
        """
        Set joint space PD gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains; its size must same as dof.
            kv : (np.ndarray) Derivative gains; its size must same as dof.
        """
        if kp is not None:
            kp = kp.reshape(-1)
            assert kp.size == self._robot_data.dof, f"Size of kp {kp.size} is not equal to dof {self._robot_data.dof}"
            super().setJointKpGain(kp)
            
        if kv is not None:
            kv = kv.reshape(-1)
            assert kv.size == self._robot_data.dof, f"Size of kv {kv.size} is not equal to dof {self._robot_data.dof}"
            super().setJointKvGain(kv)

    def set_task_gain(self, 
                      link_kp: dict[str, np.ndarray] | None = None,
                      link_kv: dict[str, np.ndarray] | None = None,
                      ):
        """
        Set task space PD gains for the manipulator per links.

        Parameters:
            kp : (dict[str, np.ndarray]) Proportional gains.
            kv : (dict[str, np.ndarray]) Derivative gains.
        """
        if link_kp is not None:
            for k, v in link_kp.items():
                link_kp[k] = v.reshape(-1)
                assert link_kp[k].size == 6, f"Size of kp {link_kp[k].size} at link {k} is not equal to 6"
            super().setTaskKpGain(link_kp)
            
        if link_kv is not None:
            for k, v in link_kv.items():
                link_kv[k] = v.reshape(-1)
                assert link_kv[k].size == 6, f"Size of kv {link_kv[k].size} at link {k} is not equal to 6"
            super().setTaskKvGain(link_kv)
        
        
    def set_QPIK_gain(self, 
                      link_w_tracking: dict[str, np.ndarray],
                      w_damping: np.ndarray,
                      ):
        """
        Set the wight vector for the cost terms of the QPIK
        
        Parameters:
            link_w_tracking : (dict[str, np.ndarray]) Weight for task velocity tracking per links.
            w_damping : (np.ndarray) Weight for joint velocity damping; its size must same as dof.
        """
        w_damping = w_damping.reshape(-1)
        assert w_damping.size == self._robot_data.dof, f"Size of w_damping {w_damping.size} is not equal to dof {self._robot_data.dof}"
        for k,v in link_w_tracking.items():
            link_w_tracking[k] = v.reshape(-1)
            assert link_w_tracking[k].size == 6, f"Size of link_w_tracking {link_w_tracking[k].size} at link {k} is not equal to 6"
        super().setQPIKGain(link_w_tracking, w_damping)
        
    def set_QPID_gain(self, 
                      link_w_tracking: dict[str, np.ndarray], 
                      w_vel_damping: np.ndarray, 
                      w_acc_damping: np.ndarray):
        """
        Set the wight vector for the cost terms of the QPID
        
        Parameters:
            link_w_tracking : (dict[str, np.ndarray]) Weight for task acceleration tracking per links.
            w_vel_damping : (np.ndarray) Weight for joint velocity damping; its size must same as dof.
            w_acc_damping : (np.ndarray) Weight for joint acceleration damping; its size must same as dof.
        """
        w_vel_damping = w_vel_damping.reshape(-1)
        w_acc_damping = w_acc_damping.reshape(-1)
        assert w_vel_damping.size == self._robot_data.dof, f"Size of w_damping {w_vel_damping.size} is not equal to dof {self._robot_data.dof}"
        assert w_acc_damping.size == self._robot_data.dof, f"Size of w_damping {w_acc_damping.size} is not equal to dof {self._robot_data.dof}"
        for k,v in link_w_tracking.items():
            link_w_tracking[k] = v.reshape(-1)
            assert link_w_tracking[k].size == 6, f"Size of link_w_tracking {link_w_tracking[k].size} at link {k} is not equal to 6"
        super().setQPIDGain(link_w_tracking, w_vel_damping, w_acc_damping)

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
        q_target = q_target.reshape(-1)
        qdot_target = qdot_target.reshape(-1)
        q_init = q_init.reshape(-1)
        qdot_init = qdot_init.reshape(-1)
        assert q_target.size == self._robot_data.dof, f"Size of q_target {q_target.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_target.size == self._robot_data.dof, f"Size of qdot_target {qdot_target.size} is not equal to dof {self._robot_data.dof}"
        assert q_init.size == self._robot_data.dof, f"Size of q_init {q_init.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_init.size == self._robot_data.dof, f"Size of qdot_init {qdot_init.size} is not equal to dof {self._robot_data.dof}"
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
        q_target = q_target.reshape(-1)
        qdot_target = qdot_target.reshape(-1)
        q_init = q_init.reshape(-1)
        qdot_init = qdot_init.reshape(-1)
        assert q_target.size == self._robot_data.dof, f"Size of q_target {q_target.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_target.size == self._robot_data.dof, f"Size of qdot_target {qdot_target.size} is not equal to dof {self._robot_data.dof}"
        assert q_init.size == self._robot_data.dof, f"Size of q_init {q_init.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_init.size == self._robot_data.dof, f"Size of qdot_init {qdot_init.size} is not equal to dof {self._robot_data.dof}"
        return super().moveJointVelocityCubic(q_target,
                                              qdot_target,
                                              q_init,
                                              qdot_init,
                                              current_time,
                                              init_time,
                                              duration, 
                                              )

    
    def move_joint_torque_step(self,
                               q_target:     np.ndarray | None = None,
                               qdot_target:  np.ndarray | None = None,
                               qddot_target: np.ndarray | None = None,
                               use_mass:     bool              = True,
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
            use_mass     : (bool) Whether use mass matrix.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        if qddot_target is not None:
            qddot_target = qddot_target.reshape(-1)
            assert qddot_target.size == self._robot_data.dof, f"Size of qddot_target {qddot_target.size} is not equal to dof {self._robot_data.dof}"
            return super().moveJointTorqueStep(qddot_target, use_mass)

        if q_target is not None and qdot_target is not None:
            q_target = q_target.reshape(-1)
            qdot_target = qdot_target.reshape(-1)
            assert q_target.size == self._robot_data.dof, f"Size of q_target {q_target.size} is not equal to dof {self._robot_data.dof}"
            assert qdot_target.size == self._robot_data.dof, f"Size of qdot_target {qdot_target.size} is not equal to dof {self._robot_data.dof}"
            return super().moveJointTorqueStep(q_target, qdot_target, use_mass)

    def move_joint_torque_cubic(self,
                                q_target: np.ndarray,
                                qdot_target: np.ndarray,
                                q_init: np.ndarray,
                                qdot_init: np.ndarray,
                                current_time: float,
                                init_time: float,
                                duration: float,
                                use_mass: bool = True,
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
            use_mass     : (bool) Whether use mass matrix.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        q_target = q_target.reshape(-1)
        qdot_target = qdot_target.reshape(-1)
        q_init = q_init.reshape(-1)
        qdot_init = qdot_init.reshape(-1)
        assert q_target.size == self._robot_data.dof, f"Size of q_target {q_target.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_target.size == self._robot_data.dof, f"Size of qdot_target {qdot_target.size} is not equal to dof {self._robot_data.dof}"
        assert q_init.size == self._robot_data.dof, f"Size of q_init {q_init.size} is not equal to dof {self._robot_data.dof}"
        assert qdot_init.size == self._robot_data.dof, f"Size of qdot_init {qdot_init.size} is not equal to dof {self._robot_data.dof}"
        return super().moveJointTorqueCubic(q_target,
                                            qdot_target,
                                            q_init,
                                            qdot_init,
                                            current_time,
                                            init_time,
                                            duration,
                                            use_mass
                                            )

    # ================================ Task space Functions ================================
    def CLIK(self,
             link_task_data: dict[str, TaskSpaceData],
             null_qdot: np.ndarray | None = None,
             ) -> np.ndarray:
        """
        Computes joint velocities to achieve desired velocity of a link by using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xdot_desired.
            null_qdot      : (np.ndarray) [Optional] Desired joint velocity to be projected on null space.
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIK(link_task_data_cpp, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().CLIK(link_task_data_cpp, null_qdot)
            
    def CLIK_step(self,
                  link_task_data: dict[str, TaskSpaceData],
                  null_qdot: np.ndarray | None = None,
                  ) -> np.ndarray:
        """
        Computes joint velocity to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            null_qdot   : (np.ndarray) [Optional] Desired joint velocity to be projected on null space.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIKStep(link_task_data_cpp, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().CLIKStep(link_task_data_cpp, null_qdot)

    def CLIK_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   init_time: float,
                   duration: float,
                   null_qdot: np.ndarray | None = None,
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration, then compute joint velocities with null_qdot to follow the resulting trajector if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            null_qdot    : (np.ndarray) [Optional] Desired joint velocity to be projected on null space.

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIKCubic(link_task_data_cpp, current_time, init_time, duration, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().CLIKCubic(link_task_data_cpp, current_time, init_time, duration, null_qdot)


    def OSF(self,
            link_task_data: dict[str, TaskSpaceData],
            null_torque: np.ndarray | None = None,
            ) -> np.ndarray:
        """
        Computes joint torque to achieve desired acceleration (xddot_desired) of a link using operational space control, projecting null_torque into null space to exploit redundancy if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xddot_desired.
            null_torque  : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSF(link_task_data_cpp, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().OSF(link_task_data_cpp, null_torque)

    def OSF_step(self,
                 link_task_data: dict[str, TaskSpaceData],
                 null_torque: np.ndarray | None = None,
                 ) -> np.ndarray:
        """
        Computes joint torque to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space control, projecting null_torque into null space to exploit redundancy if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Desired position of a link; it must include (x_desired, xdot_desired).
            null_torque : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSFStep(link_task_data_cpp, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().OSFStep(link_task_data_cpp, null_torque)

    def OSF_cubic(self,
                  link_task_data: dict[str, TaskSpaceData],
                  current_time: float,
                  init_time: float,
                  duration: float,
                  null_torque: np.ndarray | None = None,
                  ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration, then compute joint torques with null_torque to follow the resulting trajectory if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            null_torque  : (np.ndarray) [Optional] Desired joint torque to be projected on null space.

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSFCubic(link_task_data_cpp, current_time, init_time, duration, np.zerosself._robot_data.dof)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.dof, f"Size of null_qdot {null_qdot.size} is not equal to dof {self._robot_data.dof}"
            return super().OSFCubic(link_task_data_cpp, current_time, init_time, duration, null_torque)
        
    def QPIK(self, 
             link_task_data: dict[str, TaskSpaceData],
             time_verbose: bool = False,
             ) -> np.ndarray:
        """
        Computes joint velocities to achieve desired velocity (xdot_desired) of a link by solving inverse kinematics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xdot_desired.
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIK(link_task_data_cpp,time_verbose)

    def QPIK_step(self,
                  link_task_data: dict[str, TaskSpaceData],
                  time_verbose: bool = False,
                  ) -> np.ndarray:
        """
        Computes joint velocities to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse kinematics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIKStep(link_task_data_cpp,time_verbose)

    def QPIK_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   init_time: float,
                   duration: float,
                   time_verbose: bool = False,
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute joint velocities using QP to follow the resulting trajectory.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            time_verbose : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIKCubic(link_task_data_cpp,
                                 current_time,
                                 init_time,
                                 duration,
                                 time_verbose
                                 )

    def QPID(self, 
             link_task_data: dict[str, TaskSpaceData],
             time_verbose: bool = False,
             ) -> np.ndarray:
        """
        Computes joint torques to achieve desired acceleration (xddot_desired) of a link by solving inverse dynamics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xddot_desired.
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPID(link_task_data_cpp,time_verbose)

    def QPID_step(self, 
                  link_task_data: dict[str, TaskSpaceData],
                  time_verbose: bool = False,
                  ) -> np.ndarray:
        """
        Computes joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse dynamics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIDStep(link_task_data_cpp,time_verbose)

    def QPID_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   init_time: float,
                   duration: float,
                   time_verbose: bool = False,
                   ) -> np.ndarray:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute joint torques using QP to follow the resulting trajectory.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
            current_time : (float) Current time.
            init_time    : (float) Start time of the segment.
            duration     : (float) Time duration.
            time_verbose : (bool) If true, print the computation time for QP. 

        Returns:
            (np.ndarray) Desired joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIDCubic(link_task_data_cpp,
                                 current_time,
                                 init_time,
                                 duration,
                                 time_verbose)