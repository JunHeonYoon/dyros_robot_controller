import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp
from .robot_data import RobotData
from drc import TaskSpaceData


class RobotController(drc_cpp.MobileManipulatorRobotController):
    """
    A Python wrapper for the C++ RobotController::MobileManipulator::MobileManipulatorBase class.
    
    This class consists of functions that compute control inputs for mobile manipulator and helpers that generate smooth trajectories.
    Joint space functions compute control inputs of the manipulator to track desired joint positions, velocities, or accelerations.
    Task space functions compute control inputs of the whole body to track desired position or velocity of a link.
    """
    def __init__(self, robot_data: RobotData):
        """
        Constructor.

        Parameters:
            robot_data : (DataMomaBase) An instance of the Python MobileManipulatorBase wrapper which contains the robot's kinematic and dynamic parameters.

        Raises:
            TypeError: If the robot_data is not an instance of the Python MobileManipulatorBase wrapper.
        """
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python MobileManipulatorBase")
        self._robot_data = robot_data
        self._dt = float(self._robot_data.get_dt())
        super().__init__(self._robot_data)

    def set_manipulator_joint_gain(self, 
                                   kp: np.ndarray | None = None, 
                                   kv: np.ndarray| None = None,
                                   ):
        """
        Set joint space PD gains for the manipulator.

        Parameters:
            kp : (np.ndarray) Proportional gains; its size must same as mani_dof.
            kv : (np.ndarray) Derivative gains; its size must same as mani_dof.
        """
        if kp is not None:
            kp = kp.reshape(-1)
            assert kp.size == self._robot_data.mani_dof, f"Size of kp {kp.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setManipulatorJointKpGain(kp)
            
        if kv is not None:
            kv = kv.reshape(-1)
            assert kv.size == self._robot_data.mani_dof, f"Size of kv {kv.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setManipulatorJointKvGain(kv)

    def set_IK_gain(self, link_kp: dict[str, np.ndarray]):
        """
        Set IK task-space proportional gains for specified links.
        Invalid link names are ignored with a warning.

        Parameters:
            link_kp : (dict[str, np.ndarray]) Link-name to 6D task gain map.
        """
        for k, v in link_kp.items():
            link_kp[k] = v.reshape(-1)
            assert link_kp[k].size == 6, f"Size of kp {link_kp[k].size} at link {k} is not equal to 6"
        super().setIKGain(link_kp)
        
    def set_IK_gain(self, kp: np.ndarray):
        """
        Set the same IK task-space proportional gain for all link frames.

        Parameters:
            kp : (np.ndarray) 6D task-space proportional gain applied to every link.
        """
        kp = kp.reshape(-1)
        assert kp.size == 6, f"Size of kp {kp.size} is not equal to 6"
        super().setIKGain(kp)

    def set_ID_gain(self,
                    link_kp: dict[str, np.ndarray] | None = None,
                    link_kv: dict[str, np.ndarray] | None = None):
        """
        Set ID task-space proportional/derivative gains for specified links.

        Parameters:
            link_kp : (dict[str, np.ndarray]) Link-name to 6D proportional gain map.
            link_kv : (dict[str, np.ndarray]) Link-name to 6D derivative gain map.
        """
        if link_kp is not None:
            for k, v in link_kp.items():
                link_kp[k] = v.reshape(-1)
                assert link_kp[k].size == 6, f"Size of kp {link_kp[k].size} at link {k} is not equal to 6"
            super().setIDKpGain(link_kp)

        if link_kv is not None:
            for k, v in link_kv.items():
                link_kv[k] = v.reshape(-1)
                assert link_kv[k].size == 6, f"Size of kv {link_kv[k].size} at link {k} is not equal to 6"
            super().setIDKvGain(link_kv)
            
    def set_ID_gain(self, 
                    kp: np.ndarray | None = None, 
                    kv: np.ndarray | None = None):
        """
        Set the same ID task-space proportional/derivative gains for all link frames.

        Parameters:
            kp : (np.ndarray) 6D proportional gain applied to every link.
            kv : (np.ndarray) 6D derivative gain applied to every link.
        """
        if kp is not None:
            kp = kp.reshape(-1)
            assert kp.size == 6, f"Size of kp {kp.size} is not equal to 6"
        if kv is not None:
            kv = kv.reshape(-1)
            assert kv.size == 6, f"Size of kv {kv.size} is not equal to 6"
        super().setIDGain(kp, kv)
    
    def set_QPIK_gain(self,
                      link_w_tracking: dict[str, np.ndarray] | None = None,
                      w_mani_vel_damping: np.ndarray | None = None,
                      w_mani_acc_damping: np.ndarray | None = None,
                      w_base_vel_damping: np.ndarray | None = None,
                      w_base_acc_damping: np.ndarray | None = None,
                      ):
        """
        Set the weight vector for the cost terms of the QPIK.

        Parameters:
            link_w_tracking    : (dict[str, np.ndarray]) Weight for task velocity tracking per links.
            w_mani_vel_damping : (np.ndarray) Weight for manipulator joint velocity damping; its size must same as mani_dof.
            w_mani_acc_damping : (np.ndarray) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
            w_base_vel_damping : (np.ndarray) Weight for mobile base velocity damping; size must be 3 ([vx, vy, wz]).
            w_base_acc_damping : (np.ndarray) Weight for mobile base acceleration damping; size must be 3 ([vx, vy, wz]).
        """
        if link_w_tracking is not None:
            for k,v in link_w_tracking.items():
                link_w_tracking[k] = v.reshape(-1)
                assert link_w_tracking[k].size == 6, f"Size of link_w_tracking {link_w_tracking[k].size} at link {k} is not equal to 6"
            super().setQPIKTrackingGain(link_w_tracking)

        if w_mani_vel_damping is not None:
            w_mani_vel_damping = w_mani_vel_damping.reshape(-1)
            assert w_mani_vel_damping.size == self._robot_data.mani_dof, f"Size of w_mani_vel_damping {w_mani_vel_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIKManiJointVelGain(w_mani_vel_damping)

        if w_mani_acc_damping is not None:
            w_mani_acc_damping = w_mani_acc_damping.reshape(-1)
            assert w_mani_acc_damping.size == self._robot_data.mani_dof, f"Size of w_mani_acc_damping {w_mani_acc_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIKManiJointAccGain(w_mani_acc_damping)

        if w_base_vel_damping is not None:
            w_base_vel_damping = np.asarray(w_base_vel_damping).reshape(-1)
            assert w_base_vel_damping.size == 3, f"Size of w_base_vel_damping {w_base_vel_damping.size} is not equal to 3"
            super().setQPIKBaseVelGain(w_base_vel_damping)

        if w_base_acc_damping is not None:
            w_base_acc_damping = np.asarray(w_base_acc_damping).reshape(-1)
            assert w_base_acc_damping.size == 3, f"Size of w_base_acc_damping {w_base_acc_damping.size} is not equal to 3"
            super().setQPIKBaseAccGain(w_base_acc_damping)

    def set_QPIK_gain(self,
                      w_tracking: np.ndarray | None = None,
                      w_mani_vel_damping: np.ndarray | None = None,
                      w_mani_acc_damping: np.ndarray | None = None,
                      w_base_vel_damping: np.ndarray | None = None,
                      w_base_acc_damping: np.ndarray | None = None,
                      ):
        """
        Set the weight vector for the cost terms of the QPIK.

        Parameters:
            w_tracking         : (np.ndarray) Weight for task velocity tracking for every link.
            w_mani_vel_damping : (np.ndarray) Weight for manipulator joint velocity damping; its size must same as mani_dof.
            w_mani_acc_damping : (np.ndarray) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
            w_base_vel_damping : (np.ndarray) Weight for mobile base velocity damping; size must be 3 ([vx, vy, wz]).
            w_base_acc_damping : (np.ndarray) Weight for mobile base acceleration damping; size must be 3 ([vx, vy, wz]).
        """
        if w_tracking is not None:
            w_tracking = w_tracking.reshape(-1)
            assert w_tracking.size == 6, f"Size of w_tracking {w_tracking.size} is not equal to 6"
            super().setQPIKTrackingGain(w_tracking)

        if w_mani_vel_damping is not None:
            w_mani_vel_damping = w_mani_vel_damping.reshape(-1)
            assert w_mani_vel_damping.size == self._robot_data.mani_dof, f"Size of w_mani_vel_damping {w_mani_vel_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIKManiJointVelGain(w_mani_vel_damping)

        if w_mani_acc_damping is not None:
            w_mani_acc_damping = w_mani_acc_damping.reshape(-1)
            assert w_mani_acc_damping.size == self._robot_data.mani_dof, f"Size of w_mani_acc_damping {w_mani_acc_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIKManiJointAccGain(w_mani_acc_damping)

        if w_base_vel_damping is not None:
            w_base_vel_damping = np.asarray(w_base_vel_damping).reshape(-1)
            assert w_base_vel_damping.size == 3, f"Size of w_base_vel_damping {w_base_vel_damping.size} is not equal to 3"
            super().setQPIKBaseVelGain(w_base_vel_damping)

        if w_base_acc_damping is not None:
            w_base_acc_damping = np.asarray(w_base_acc_damping).reshape(-1)
            assert w_base_acc_damping.size == 3, f"Size of w_base_acc_damping {w_base_acc_damping.size} is not equal to 3"
            super().setQPIKBaseAccGain(w_base_acc_damping)

    def set_QPID_gain(self,
                      link_w_tracking: dict[str, np.ndarray] | None = None,
                      w_mani_vel_damping: np.ndarray | None = None,
                      w_mani_acc_damping: np.ndarray | None = None,
                      w_base_vel_damping: np.ndarray | None = None,
                      w_base_acc_damping: np.ndarray | None = None,
                      ):
        """
        Set the weight vector for the cost terms of the QPID.

        Parameters:
            link_w_tracking    : (dict[str, np.ndarray]) Weight for task acceleration tracking per links.
            w_mani_vel_damping : (np.ndarray) Weight for manipulator joint velocity damping; its size must same as mani_dof.
            w_mani_acc_damping : (np.ndarray) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
            w_base_vel_damping : (np.ndarray) Weight for mobile base velocity damping; size must be 3 ([vx, vy, wz]).
            w_base_acc_damping : (np.ndarray) Weight for mobile base acceleration damping; size must be 3 ([vx, vy, wz]).
        """
        if link_w_tracking is not None:
            for k,v in link_w_tracking.items():
                link_w_tracking[k] = v.reshape(-1)
                assert link_w_tracking[k].size == 6, f"Size of link_w_tracking {link_w_tracking[k].size} at link {k} is not equal to 6"
            super().setQPIDTrackingGain(link_w_tracking)

        if w_mani_vel_damping is not None:
            w_mani_vel_damping = w_mani_vel_damping.reshape(-1)
            assert w_mani_vel_damping.size == self._robot_data.mani_dof, f"Size of w_mani_vel_damping {w_mani_vel_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIDManiJointVelGain(w_mani_vel_damping)

        if w_mani_acc_damping is not None:
            w_mani_acc_damping = w_mani_acc_damping.reshape(-1)
            assert w_mani_acc_damping.size == self._robot_data.mani_dof, f"Size of w_mani_acc_damping {w_mani_acc_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIDManiJointAccGain(w_mani_acc_damping)

        if w_base_vel_damping is not None:
            w_base_vel_damping = np.asarray(w_base_vel_damping).reshape(-1)
            assert w_base_vel_damping.size == 3, f"Size of w_base_vel_damping {w_base_vel_damping.size} is not equal to 3"
            super().setQPIDBaseVelGain(w_base_vel_damping)

        if w_base_acc_damping is not None:
            w_base_acc_damping = np.asarray(w_base_acc_damping).reshape(-1)
            assert w_base_acc_damping.size == 3, f"Size of w_base_acc_damping {w_base_acc_damping.size} is not equal to 3"
            super().setQPIDBaseAccGain(w_base_acc_damping)

    def set_QPID_gain(self,
                      w_tracking: np.ndarray | None = None, 
                      w_mani_vel_damping: np.ndarray | None = None, 
                      w_mani_acc_damping: np.ndarray | None = None, 
                      w_base_vel_damping: np.ndarray | None = None, 
                      w_base_acc_damping: np.ndarray | None = None, 
                      ):
        """
        Set the weight vector for the cost terms of the QPID.

        Parameters:
            w_tracking         : (np.ndarray) Weight for task acceleration tracking for every link.
            w_mani_vel_damping : (np.ndarray) Weight for manipulator joint velocity damping; its size must same as mani_dof.
            w_mani_acc_damping : (np.ndarray) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
            w_base_vel_damping : (np.ndarray) Weight for mobile base velocity damping; size must be 3 ([vx, vy, wz]).
            w_base_acc_damping : (np.ndarray) Weight for mobile base acceleration damping; size must be 3 ([vx, vy, wz]).
        """
        if w_tracking is not None:
            w_tracking = w_tracking.reshape(-1)
            assert w_tracking.size == 6, f"Size of w_tracking {w_tracking.size} is not equal to 6"
            super().setQPIDTrackingGain(w_tracking)

        if w_mani_vel_damping is not None:
            w_mani_vel_damping = w_mani_vel_damping.reshape(-1)
            assert w_mani_vel_damping.size == self._robot_data.mani_dof, f"Size of w_mani_vel_damping {w_mani_vel_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIDManiJointVelGain(w_mani_vel_damping)

        if w_mani_acc_damping is not None:
            w_mani_acc_damping = w_mani_acc_damping.reshape(-1)
            assert w_mani_acc_damping.size == self._robot_data.mani_dof, f"Size of w_mani_acc_damping {w_mani_acc_damping.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            super().setQPIDManiJointAccGain(w_mani_acc_damping)

        if w_base_vel_damping is not None:
            w_base_vel_damping = np.asarray(w_base_vel_damping).reshape(-1)
            assert w_base_vel_damping.size == 3, f"Size of w_base_vel_damping {w_base_vel_damping.size} is not equal to 3"
            super().setQPIDBaseVelGain(w_base_vel_damping)

        if w_base_acc_damping is not None:
            w_base_acc_damping = np.asarray(w_base_acc_damping).reshape(-1)
            assert w_base_acc_damping.size == 3, f"Size of w_base_acc_damping {w_base_acc_damping.size} is not equal to 3"
            super().setQPIDBaseAccGain(w_base_acc_damping)

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
        q_mani_target = q_mani_target.reshape(-1)
        qdot_mani_target = qdot_mani_target.reshape(-1)
        q_mani_init = q_mani_init.reshape(-1)
        qdot_mani_init = qdot_mani_init.reshape(-1)
        assert q_mani_target.size == self._robot_data.mani_dof, f"Size of q_mani_target {q_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_target.size == self._robot_data.mani_dof, f"Size of qdot_mani_target {qdot_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert q_mani_init.size == self._robot_data.mani_dof, f"Size of q_mani_init {q_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_init.size == self._robot_data.mani_dof, f"Size of qdot_mani_init {qdot_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        return super().moveManipulatorJointPositionCubic(q_mani_target,
                                                         qdot_mani_target,
                                                         q_mani_init,
                                                         qdot_mani_init,
                                                         current_time,
                                                         init_time,
                                                         duration,
                                                         )
        
    def move_manipulator_joint_velocity_cubic(self,
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
            (np.ndarray) Desired manipulator joint velocities.
        """
        q_mani_target = q_mani_target.reshape(-1)
        qdot_mani_target = qdot_mani_target.reshape(-1)
        q_mani_init = q_mani_init.reshape(-1)
        qdot_mani_init = qdot_mani_init.reshape(-1)
        assert q_mani_target.size == self._robot_data.mani_dof, f"Size of q_mani_target {q_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_target.size == self._robot_data.mani_dof, f"Size of qdot_mani_target {qdot_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert q_mani_init.size == self._robot_data.mani_dof, f"Size of q_mani_init {q_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_init.size == self._robot_data.mani_dof, f"Size of qdot_mani_init {qdot_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        return super().moveManipulatorJointVelocityCubic(q_mani_target,
                                                         qdot_mani_target,
                                                         q_mani_init,
                                                         qdot_mani_init,
                                                         current_time,
                                                         init_time,
                                                         duration,
                                                         )
        
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
            qddot_mani_target = qddot_mani_target.reshape(-1)
            assert qddot_mani_target.size == self._robot_data.dof, f"Size of qddot_mani_target {qddot_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            return super().moveManipulatorJointTorqueStep(qddot_mani_target, use_mass)

        if q_mani_target is not None and qdot_mani_target is not None:
            q_mani_target = q_mani_target.reshape(-1)
            qdot_mani_target = qdot_mani_target.reshape(-1)
            assert q_mani_target.size == self._robot_data.mani_dof, f"Size of q_mani_target {q_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
            assert qdot_mani_target.size == self._robot_data.mani_dof, f"Size of qdot_mani_target {qdot_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
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
        q_mani_target = q_mani_target.reshape(-1)
        qdot_mani_target = qdot_mani_target.reshape(-1)
        q_mani_init = q_mani_init.reshape(-1)
        qdot_mani_init = qdot_mani_init.reshape(-1)
        assert q_mani_target.size == self._robot_data.mani_dof, f"Size of q_mani_target {q_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_target.size == self._robot_data.mani_dof, f"Size of qdot_mani_target {qdot_mani_target.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert q_mani_init.size == self._robot_data.mani_dof, f"Size of q_mani_init {q_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
        assert qdot_mani_init.size == self._robot_data.mani_dof, f"Size of qdot_mani_init {qdot_mani_init.size} is not equal to mani_dof {self._robot_data.mani_dof}"
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
    def CLIK(self,
             link_task_data: dict[str, TaskSpaceData],
             null_qdot: np.ndarray | None = None,
             ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base and manipulator joint velocities to achieve desired velocity of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xdot_desired.
            null_qdot      : (np.ndarray) [Optional] Desired actuated joint velocity to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIK(link_task_data_cpp)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.actuated_dof, f"Size of null_qdot {null_qdot.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().CLIK(link_task_data_cpp, null_qdot)

    def CLIK_step(self,
                  link_task_data: dict[str, TaskSpaceData],
                  null_qdot: np.ndarray | None = None,
                  ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base and manipulator joint velocities to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics, projecting null_qdot into null space if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            null_qdot      : (np.ndarray) [Optional] Desired actuated joint velocity to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIKStep(link_task_data_cpp)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.actuated_dof, f"Size of null_qdot {null_qdot.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().CLIKStep(link_task_data_cpp, null_qdot)

    def CLIK_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   duration: float,
                   null_qdot: np.ndarray | None = None,
                   ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base and manipulator joint velocities using CLIK with null_qdot if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired). Each entry's control_start_time is used as the segment start time.
            current_time : (float) Current time.
            duration     : (float) Time duration.
            null_qdot    : (np.ndarray) [Optional] Desired actuated joint velocity to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_qdot is None:
            return super().CLIKCubic(link_task_data_cpp, current_time, 0.0, duration)
        else:
            null_qdot = null_qdot.reshape(-1)
            assert null_qdot.size == self._robot_data.actuated_dof, f"Size of null_qdot {null_qdot.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().CLIKCubic(link_task_data_cpp, current_time, 0.0, duration, null_qdot)

    def OSF(self,
            link_task_data: dict[str, TaskSpaceData],
            null_torque: np.ndarray | None = None,
            ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link using operational space formulation, projecting null_torque into null space if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xddot_desired.
            null_torque    : (np.ndarray) [Optional] Desired actuated joint torque to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSF(link_task_data_cpp)
        else:
            null_torque = null_torque.reshape(-1)
            assert null_torque.size == self._robot_data.actuated_dof, f"Size of null_torque {null_torque.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().OSF(link_task_data_cpp, null_torque)

    def OSF_step(self,
                 link_task_data: dict[str, TaskSpaceData],
                 null_torque: np.ndarray | None = None,
                 ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space formulation, projecting null_torque into null space if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            null_torque    : (np.ndarray) [Optional] Desired actuated joint torque to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSFStep(link_task_data_cpp)
        else:
            null_torque = null_torque.reshape(-1)
            assert null_torque.size == self._robot_data.actuated_dof, f"Size of null_torque {null_torque.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().OSFStep(link_task_data_cpp, null_torque)

    def OSF_cubic(self,
                  link_task_data: dict[str, TaskSpaceData],
                  current_time: float,
                  duration: float,
                  null_torque: np.ndarray | None = None,
                  ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using OSF with null_torque if provided.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired). Each entry's control_start_time is used as the segment start time.
            current_time : (float) Current time.
            duration     : (float) Time duration.
            null_torque  : (np.ndarray) [Optional] Desired actuated joint torque to be projected on null space.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        if null_torque is None:
            return super().OSFCubic(link_task_data_cpp, current_time, 0.0, duration)
        else:
            null_torque = null_torque.reshape(-1)
            assert null_torque.size == self._robot_data.actuated_dof, f"Size of null_torque {null_torque.size} is not equal to actuated_dof {self._robot_data.actuated_dof}"
            return super().OSFCubic(link_task_data_cpp, current_time, 0.0, duration, null_torque)

    def QPIK(self, 
             link_task_data: dict[str, TaskSpaceData],
             time_verbose: bool = False,
             ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes velocities for mobile base and manipulator joints to achieve desired velocity (xdot_desired) of a link by solving inverse kinematics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xdot_desired.
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIK(link_task_data_cpp,time_verbose)

    def QPIK_step(self,
                  link_task_data: dict[str, TaskSpaceData],
                  time_verbose: bool = False,
                  ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes velocities for mobile base and manipulator joints to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse kinematics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIKStep(link_task_data_cpp,time_verbose)

    def QPIK_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   duration: float,
                   time_verbose: bool = False,
                   ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute velocities for mobile base and manipulator joints using QP to follow the resulting trajectory.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired). Each entry's control_start_time is used as the segment start time.
            current_time : (float) Current time.
            duration     : (float) Time duration.
            time_verbose : (bool) If true, print the computation time for QP.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base velocities, output optimal manipulator joint velocities.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIKCubic(link_task_data_cpp,
                                 current_time,
                                 0.0,
                                 duration,
                                 time_verbose
                                 )

    def QPID(self, 
             link_task_data: dict[str, TaskSpaceData],
             time_verbose: bool = False,
             ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link by solving inverse dynamics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include xddot_desired.
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPID(link_task_data_cpp,time_verbose)

    def QPID_step(self, 
                  link_task_data: dict[str, TaskSpaceData],
                  time_verbose: bool = False,
                  ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse dynamics QP.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_desired, xdot_desired).
            time_verbose   : (bool) If true, print the computation time for QP. 

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIDStep(link_task_data_cpp,time_verbose)

    def QPID_cubic(self,
                   link_task_data: dict[str, TaskSpaceData],
                   current_time: float,
                   duration: float,
                   time_verbose: bool = False,
                   ) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using QP to follow the resulting trajectory.

        Parameters:
            link_task_data : (dict[str, TaskSpaceData]) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired). Each entry's control_start_time is used as the segment start time.
            current_time : (float) Current time.
            duration     : (float) Time duration.
            time_verbose : (bool) If true, print the computation time for QP.

        Returns:
            (tuple[bool, np.ndarray, np.ndarray]) Success flag, output optimal mobile base accelerations, output optimal manipulator joint torques.
        """
        link_task_data_cpp = {}
        for k, v in link_task_data.items():
            if hasattr(v, "cpp"):
                link_task_data_cpp[k] = v.cpp()
        return super().QPIDCubic(link_task_data_cpp,
                                 current_time,
                                 0.0,
                                 duration,
                                 time_verbose)
