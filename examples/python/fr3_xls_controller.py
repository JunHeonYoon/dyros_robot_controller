# ============================ FR3 XLS Mobile Manipulator Controller ============================
import os
import numpy as np
from typing import Dict
from pynput import keyboard  # Non-blocking, global keyboard listener
from drc import TaskSpaceData 
from drc.mobile_manipulator.robot_data import RobotData
from drc.mobile_manipulator.robot_controller import RobotController
from drc import KinematicParam, DriveType, JointIndex, ActuatorIndex


class FR3XLSController:
    """
    FR3XLSController
    -------------
    Minimal example of using Dyros Robot Controller for a 7-DoF FR3 arm with 4-wheel Mecanum base.

    - Keeps internal state (q, qdot, EE pose/velocity).
    - Switches control modes via global hotkeys (1/2/3).
    - On every mode switch, snapshots current state as initial references.

    Parameters
    ----------
    dt : float
        Simulation/control timestep [s].
    """

    def __init__(self, dt: float):
        # --- Core configuration/state ---
        self.dt: float = dt
        self.sim_time: float = 0.0

        # Paths to URDF/SRDF (model files)
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_file_path = os.path.join(root, "robots", "fr3_xls", "fr3_xls.urdf")
        srdf_file_path = os.path.join(root, "robots", "fr3_xls", "fr3_xls.srdf")
        
        # Define mobile base kinematics (Mecanum drive) and constraints
        # All geometry is expressed in the base frame:
        #  - base2wheel_positions: 2D position of each wheel center wrt base origin
        #  - base2wheel_angles: wheel mounting angles (fixed here; not steering angles)
        #  - roller_angles: Mecanum roller angles for each wheel
        #  - max_*: saturation limits (used by controller for velocity/acceleration limits)
        mobile_param = KinematicParam(
            type=DriveType.Mecanum,
            wheel_radius=0.120,
            base2wheel_positions=[
                np.array([ 0.2225,  0.2045]),  # FL
                np.array([ 0.2225, -0.2045]),  # FR
                np.array([-0.2225,  0.2045]),  # RL
                np.array([-0.2225, -0.2045]),  # RR
            ],
            base2wheel_angles=[0, 0, 0, 0],
            roller_angles=[-np.pi/4, np.pi/4, np.pi/4, -np.pi/4],
            max_lin_speed=2,
            max_ang_speed=3,
            max_lin_acc=3,
            max_ang_acc=6,
        )
        
        joint_idx = JointIndex(virtual_start=0, mani_start=3, mobi_start=10)
        actuator_idx = ActuatorIndex(mani_start=0, mobi_start=7)

        # Instantiate dyros robot model/controller
        self.robot_data = RobotData(dt=self.dt,
                                    mobile_param=mobile_param, 
                                    joint_idx=joint_idx, 
                                    actuator_idx=actuator_idx, 
                                    urdf_path=urdf_file_path, 
                                    srdf_path=srdf_file_path)
        self.robot_controller = RobotController(robot_data=self.robot_data)

        # Degree of freedom
        self.virual_dof: int = 3
        self.mani_dof: int = self.robot_data.get_manipulator_dof()
        self.mobi_dof: int = self.robot_data.get_mobile_dof()
        self.actuated_dof: int = self.robot_data.get_actuator_dof()

        # --- Joint space states (measured/desired/snapshots) ---
        # Mobile Base (computed base twist [vx, vy, wz] in base frame
        self.base_vel             = np.zeros(self.virual_dof)
        self.base_vel_desired     = np.zeros(self.virual_dof)
        self.base_vel_init        = np.zeros(self.virual_dof)
        
        # Virtual joint state (integrated planar base pose and twist: [x, y, yaw])
        self.q_virtual            = np.zeros(self.virual_dof)
        self.q_virtual_desired    = np.zeros(self.virual_dof)
        self.q_virtual_init       = np.zeros(self.virual_dof)
        self.qdot_virtual         = np.zeros(self.virual_dof)
        self.qdot_virtual_desired = np.zeros(self.virual_dof)
        self.qdot_virtual_init    = np.zeros(self.virual_dof)
        
        # Manipulator joint state
        self.q_mani               = np.zeros(self.mani_dof)
        self.q_mani_desired       = np.zeros(self.mani_dof)
        self.q_mani_init          = np.zeros(self.mani_dof)
        self.qdot_mani            = np.zeros(self.mani_dof)
        self.qdot_mani_desired    = np.zeros(self.mani_dof)
        self.qdot_mani_init       = np.zeros(self.mani_dof)
        self.tau_mani_desired     = np.zeros(self.mani_dof) # Manipulator torque command [Nm]
        
        # Mobile wheel joint state
        self.q_mobile             = np.zeros(self.mobi_dof)
        self.q_mobile_desired     = np.zeros(self.mobi_dof)
        self.q_mobile_init        = np.zeros(self.mobi_dof)
        self.qdot_mobile          = np.zeros(self.mobi_dof)
        self.qdot_mobile_desired  = np.zeros(self.mobi_dof) # Wheel velocity command (mapped directly to output)
        self.qdot_mobile_init     = np.zeros(self.mobi_dof)

        # --- Task space (end-effector) states (measured/desired/snapshots) ---
        self.ee_link_name = "fr3_hand_tcp"
        self.link_ee_task = {self.ee_link_name : TaskSpaceData.Zero()}

        # --- Mode bookkeeping ---
        self.control_mode: str = "Home"
        self.is_control_mode_changed: bool = True
        self.control_start_time: float = 0.0
        
        # --- Gain
        self.kp_mani_joint = np.array([600.0,600.0,600.0,600.0,250.0,150.0,50.0])
        self.kv_mani_joint = np.array([30.0,30.0,30.0,30.0,10.0,10.0,5.0])
        self.link_kp_task = {self.ee_link_name: np.array([100, 100, 100, 100, 100, 100])}
        self.link_kv_task = {self.ee_link_name: np.array([20, 20, 20, 20, 20, 20])}
        self.qpik_link_tracking = {self.ee_link_name: np.array([10, 10, 10, 10, 10, 10])}
        self.qpik_mani_damping = np.array([10, 10, 10, 10, 10, 10, 10])
        self.qpik_base_damping = np.array([0.2, 0.2, 0.2])  # [vx, vy, wz]
        self.qpid_link_tracking = {self.ee_link_name: np.array([1, 1, 1, 1, 1, 1])}
        self.qpid_mani_vel_damping = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.qpid_mani_acc_damping = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.qpid_base_vel_damping = np.array([10, 10, 10])      # [vx, vy, wz]
        self.qpid_base_acc_damping = np.array([0.0001, 0.0001, 0.0001])
        
        self.robot_controller.set_manipulator_joint_gain(kp=self.kp_mani_joint, kv=self.kv_mani_joint)
        self.robot_controller.set_task_gain(link_kp=self.link_kp_task, link_kv=self.link_kv_task)
        self.robot_controller.set_QPIK_gain(link_w_tracking=self.qpik_link_tracking,
                                            w_mani_damping=self.qpik_mani_damping,
                                            w_base_damping=self.qpik_base_damping)
        self.robot_controller.set_QPID_gain(link_w_tracking=self.qpid_link_tracking,
                                            w_mani_vel_damping=self.qpid_mani_vel_damping,
                                            w_mani_acc_damping=self.qpid_mani_acc_damping,
                                            w_base_vel_damping=self.qpid_base_vel_damping,
                                            w_base_acc_damping=self.qpid_base_acc_damping)
        
        # Print FR3XLS URDF info
        print("info:")
        print(self.robot_data.get_verbose())
        
        print("link frame info:")
        print(self.robot_data.get_link_frame_vector())
        
        print("joint frame info:")
        print(self.robot_data.get_joint_frame_vector())
        

        # Global keyboard listener (non-blocking)
        self._listener = keyboard.Listener(on_press=self._on_key_press)
        self._listener.daemon = True
        self._listener.start()
        print("[FR3XLSController] Keyboard: [1]=Home, [2]=QPIK, [3]=Gravity Compensation W QPID")

    def update_model(self, current_time: float, qpos_dict: Dict[str, float], qvel_dict: Dict[str, float]) -> None:
        """
        Update internal model/state from simulator readings.

        Parameters
        ----------
        current_time : float
            Simulator time [s].
        qpos_dict : Dict[str, float]
            Mapping of joint name -> position (rad).
        qvel_dict : Dict[str, float]
            Mapping of joint name -> velocity (rad/s).
        """
        # Time update (shared convention)
        self.sim_time = current_time
        
        # Read joint states (joint naming must match the simulator)
        for i in range(self.mani_dof):
            jn = f"fr3_joint{i+1}"
            self.q_mani[i] = qpos_dict[jn]
            self.qdot_mani[i] = qvel_dict[jn]
            
        self.q_mobile[:] = np.array([
            qpos_dict["front_left_wheel"],
            qpos_dict["front_right_wheel"],
            qpos_dict["rear_left_wheel"],
            qpos_dict["rear_right_wheel"],
        ])
        self.qdot_mobile[:] = np.array([
            qvel_dict["front_left_wheel"],
            qvel_dict["front_right_wheel"],
            qvel_dict["rear_left_wheel"],
            qvel_dict["rear_right_wheel"],
        ])
        
        # Compute base twist [vx, vy, wz] from mobile kinematics, then integrate planar base pose q_virtual=[x,y,yaw]
        self.base_vel = self.robot_data.compute_mobile_base_vel(q_mobile=self.q_mobile, qdot_mobile=self.qdot_mobile)
        self.qdot_virtual[2] = self.base_vel[2]
        self.q_virtual[2] += self.qdot_virtual[2] * self.dt
        
        R_w_m = np.array([[np.cos(self.q_virtual[2]), -np.sin(self.q_virtual[2]), 0],
                          [np.sin(self.q_virtual[2]),  np.cos(self.q_virtual[2]), 0],
                          [0,                          0,                         1]])
        # Rotate base-frame twist into world frame for integrating x/y
        self.qdot_virtual = R_w_m @ self.base_vel
        self.q_virtual[:2] += self.qdot_virtual[:2] * self.dt

        # Push to dyros robot model and cache EE pose/twist
        self.robot_data.update_state(q_virtual=self.q_virtual, 
                                     q_mobile=self.q_mobile, 
                                     q_mani=self.q_mani, 
                                     qdot_virtual=self.qdot_virtual, 
                                     qdot_mobile=self.qdot_mobile, 
                                     qdot_mani=self.qdot_mani)
        self.link_ee_task[self.ee_link_name].x = self.robot_data.get_pose(self.ee_link_name).copy()
        self.link_ee_task[self.ee_link_name].xdot = self.robot_data.get_velocity(self.ee_link_name).copy()
        

    def compute(self) -> Dict[str, float]:
        """
        Compute control command for the current mode.

        Returns
        -------
        Dict[str, float]
            Actuator command map:
              - wheels: commanded wheel angular velocity [rad/s]
              - arm joints: commanded joint torque [Nm]
        """
        # One-time init per mode entry
        if self.is_control_mode_changed:
            self.is_control_mode_changed = False
            self.control_start_time = self.sim_time

            # Snapshot current measured states as new references
            self.q_virtual_init = self.q_virtual.copy()
            self.q_mobile_init  = self.q_mobile.copy()
            self.q_mani_init    = self.q_mani.copy()
            self.qdot_virtual_init = self.qdot_virtual.copy()
            self.qdot_mobile_init  = self.qdot_mobile.copy()
            self.qdot_mani_init    = self.qdot_mani.copy()
            self.link_ee_task[self.ee_link_name].setInit()

            # Reset desired trajectories to snapshots
            self.q_virtual_desired = self.q_virtual_init.copy()
            self.q_mobile_desired  = self.q_mobile_init.copy()
            self.q_mani_desired    = self.q_mani_init.copy()
            self.qdot_virtual_desired = np.zeros_like(self.qdot_virtual_init)
            self.qdot_mobile_desired  = np.zeros_like(self.qdot_mobile_init)
            self.qdot_mani_desired    = np.zeros_like(self.qdot_mani_init)
            self.link_ee_task[self.ee_link_name].setDesired()

            self.link_ee_task[self.ee_link_name].xdot_desired = np.zeros(6)

        # --- Mode: Home (joint-space cubic to a predefined posture) ---
        if self.control_mode == "Home":
            q_mani_home = np.array([0.0, 0.0, 0.0, -np.pi/2.0, 0.0, np.pi/2.0, np.pi/4.0])
            self.tau_mani_desired = self.robot_controller.move_manipulator_joint_torque_cubic(
                q_mani_target=q_mani_home,
                qdot_mani_target=np.zeros(self.mani_dof),
                q_mani_init=self.q_mani_init,
                qdot_mani_init=self.qdot_mani_init,
                current_time=self.sim_time,
                init_time=self.control_start_time,
                duration=3.0,
                use_mass=False,
            )
            self.qdot_mobile_desired = np.zeros(self.mobi_dof)

        # --- Mode: QPIK (task-space, QP-based IK with cubic profiling) ---
        elif self.control_mode == "QPIK":
            self.link_ee_task[self.ee_link_name].x_desired = self.link_ee_task[self.ee_link_name].x_init.copy()
            self.link_ee_task[self.ee_link_name].x_desired[0:3, 3] += np.array([0, 0.1, 0.1])  # +10 cm in Y and Z

            _, self.qdot_mobile_desired, self.qdot_mani_desired = self.robot_controller.QPIK_cubic(
                link_task_data=self.link_ee_task,
                init_time=self.control_start_time,
                current_time=self.sim_time,
                duration=2.0,
            )
            # Simple Euler integration for generating a joint-position target from qdot_desired (one-step lookahead)
            self.q_mani_desired = self.q_mani + self.qdot_mani_desired * self.dt

            # Map (q_target, qdot_target) -> manipulator torque command (joint PD + gravity compensation inside)
            self.tau_mani_desired = self.robot_controller.move_manipulator_joint_torque_step(
                q_mani_target=self.q_mani_desired, qdot_mani_target=self.qdot_mani_desired
            )
            
        # --- Mode: Gravity Compensation W QPID (task-space QPID with zero desired acceleration) ---
        elif self.control_mode == "Gravity Compensation W QPID":
            self.link_ee_task[self.ee_link_name].xddot = np.zeros(6)
            _, qddot_mobile_desired, self.tau_mani_desired = self.robot_controller.QPID(link_task_data=self.link_ee_task)
            # Integrate mobile acceleration output to wheel velocity command
            self.qdot_mobile_desired = self.qdot_mobile + qddot_mobile_desired * self.dt

        # Format output for simulator actuators
        output = {
            "front_left_wheel":  float(self.qdot_mobile_desired[0]),
            "front_right_wheel": float(self.qdot_mobile_desired[1]),
            "rear_left_wheel":   float(self.qdot_mobile_desired[2]),
            "rear_right_wheel":  float(self.qdot_mobile_desired[3]),
        }
        output.update({f"fr3_joint{i+1}": float(self.tau_mani_desired[i]) for i in range(self.mani_dof)})
        return output

    def _set_mode(self, control_mode: str) -> None:
        """
        Switch control mode and trigger per-mode re-initialization.

        Parameters
        ----------
        control_mode : str
            One of {"Home", "QPIK", "Gravity Compensation W QPID"}.
        """
        self.is_control_mode_changed = True
        self.control_mode = control_mode
        print(f"Control Mode Changed: {self.control_mode}")

    def _on_key_press(self, key) -> None:
        """Global hotkeys for mode switching: 1=Home, 2=QPIK, 3=Gravity Compensation W QPID."""
        try:
            if key.char == '1':
                self._set_mode("Home")
            elif key.char == '2':
                self._set_mode("QPIK")
            elif key.char == '3':
                self._set_mode("Gravity Compensation W QPID")
        except AttributeError:
            # Ignore non-character keys
            pass
