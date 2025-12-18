# ============================ FR3 Manipulator Controller ============================
import os
import numpy as np
from typing import Dict
from pynput import keyboard  # Non-blocking, global keyboard listener
from drc import TaskSpaceData 
from drc.manipulator.robot_data import RobotData
from drc.manipulator.robot_controller import RobotController


class FR3Controller:
    """
    FR3Controller
    -------------
    Minimal example of using Dyros Robot Controller for a 7-DoF FR3 arm.

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
        urdf_file_path = os.path.join(root, "robots", "fr3", "fr3.urdf")
        srdf_file_path = os.path.join(root, "robots", "fr3", "fr3.srdf")

        # Instantiate dyros robot model/controller
        self.robot_data = RobotData(urdf_path=urdf_file_path, srdf_path=srdf_file_path)
        self.robot_controller = RobotController(dt=self.dt, robot_data=self.robot_data)

        # Degree of freedom
        self.dof: int = self.robot_data.get_dof()

        # --- Joint space states (measured/desired/snapshots) ---
        self.q            = np.zeros(self.dof)  # measured joints
        self.qdot         = np.zeros(self.dof)  # measured joint velocities
        self.q_desired    = np.zeros(self.dof)  # desired joints
        self.qdot_desired = np.zeros(self.dof)  # desired joint velocities
        self.q_init       = np.zeros(self.dof)  # snapshot at mode entry
        self.qdot_init    = np.zeros(self.dof)  # snapshot at mode entry
        self.tau_desired  = np.zeros(self.dof)  # output torques

        # --- Task space (end-effector) states (measured/desired/snapshots) ---
        self.ee_link_name = "fr3_link8"
        self.link_ee_task = {self.ee_link_name : TaskSpaceData.Zero()}

        # --- Mode bookkeeping (naming/style unified with XLSController) ---
        self.control_mode: str = "Home"
        self.is_control_mode_changed: bool = True
        self.control_start_time: float = 0.0
        
        # --- Gain
        self.kp_joint = np.array([600.0,600.0,600.0,600.0,250.0,150.0,50.0])
        self.kv_joint = np.array([30.0,30.0,30.0,30.0,10.0,10.0,5.0])
        self.link_kp_task = {self.ee_link_name: np.array([100, 100, 100, 100, 100, 100])}
        self.link_kv_task = {self.ee_link_name: np.array([20, 20, 20, 20, 20, 20])}
        self.qpik_link_tracking = {self.ee_link_name: np.array([10, 10, 10, 10, 10, 10])}
        self.qpik_damping = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.qpid_link_tracking = {self.ee_link_name: np.array([1, 1, 1, 1, 1, 1])}
        self.qpid_vel_damping = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.qpid_acc_damping = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        
        self.robot_controller.set_joint_gain(kp=self.kp_joint, kv=self.kv_joint)
        self.robot_controller.set_task_gain(link_kp=self.link_kp_task, link_kv=self.link_kv_task)
        self.robot_controller.set_QPIK_gain(link_w_tracking=self.qpik_link_tracking, w_damping=self.qpik_damping)
        self.robot_controller.set_QPID_gain(link_w_tracking=self.qpid_link_tracking, w_vel_damping=self.qpid_vel_damping, w_acc_damping=self.qpid_acc_damping)
        
        # Print FR3 URDF info
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
        print("[FR3Controller] Keyboard: [1]=Home, [2]=QPIK, [3]=Gravity Compensation, [3]=Gravity Compensation W QPID")

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
        for i in range(self.dof):
            jn = f"fr3_joint{i+1}"
            self.q[i] = qpos_dict[jn]
            self.qdot[i] = qvel_dict[jn]

        # Push to dyros robot model and cache EE pose/twist
        self.robot_data.update_state(self.q, self.qdot)
        self.link_ee_task[self.ee_link_name].x = self.robot_data.get_pose(self.ee_link_name).copy()
        self.link_ee_task[self.ee_link_name].xdot = self.robot_data.get_velocity(self.ee_link_name).copy()

    def compute(self) -> Dict[str, float]:
        """
        Compute control command for the current mode.

        Returns
        -------
        Dict[str, float]
            Actuator torque map: joint name -> tau [Nm].
        """
        # One-time init per mode entry
        if self.is_control_mode_changed:
            self.is_control_mode_changed = False
            self.control_start_time = self.sim_time

            # Snapshot current measured states as new references
            self.q_init = self.q.copy()
            self.qdot_init = self.qdot.copy()
            self.link_ee_task[self.ee_link_name].setInit()

            # Reset desired trajectories to snapshots
            self.q_desired = self.q_init.copy()
            self.qdot_desired = np.zeros_like(self.qdot_init)
            self.link_ee_task[self.ee_link_name].setDesired()
            self.link_ee_task[self.ee_link_name].xdot_desired = np.zeros(6)

        # --- Mode: Home (joint-space cubic to a predefined posture) ---
        if self.control_mode == "Home":
            q_home = np.array([0.0, 0.0, 0.0, -np.pi/2.0, 0.0, np.pi/2.0, np.pi/4.0])
            self.q_desired = self.robot_controller.move_joint_position_cubic(
                q_target=q_home,
                qdot_target=np.zeros(self.dof),
                q_init=self.q_init,
                qdot_init=self.qdot_init,
                init_time=self.control_start_time,
                current_time=self.sim_time,
                duration=3.0,
            )
            self.qdot_desired = self.robot_controller.move_joint_velocity_cubic(
                q_target=q_home,
                qdot_target=np.zeros(self.dof),
                q_init=self.q_init,
                qdot_init=self.qdot_init,
                init_time=self.control_start_time,
                current_time=self.sim_time,
                duration=3.0,
            )
            self.tau_desired = self.robot_controller.move_joint_torque_step(
                q_target=self.q_desired, qdot_target=self.qdot_desired, use_mass=False
            )

        # --- Mode: QPIK (task-space, QP-based IK with cubic profiling) ---
        elif self.control_mode == "QPIK":
            self.link_ee_task[self.ee_link_name].x_desired = self.link_ee_task[self.ee_link_name].x_init.copy()
            self.link_ee_task[self.ee_link_name].x_desired[0:3, 3] += np.array([0, 0.1, 0.1])  # +10 cm in Y and Z

            self.qdot_desired = self.robot_controller.QPIK_cubic(
                link_task_data=self.link_ee_task,
                init_time=self.control_start_time,
                current_time=self.sim_time,
                duration=2.0,
            )
            # Simple Euler integrate desired joint positions from qdot_desired
            self.q_desired = self.q + self.qdot_desired * self.dt

            # Map (q, qdot) -> torque (PD + gravity)
            self.tau_desired = self.robot_controller.move_joint_torque_step(
                q_target=self.q_desired, qdot_target=self.qdot_desired, use_mass=False
            )
            
        # --- Mode: Gravity Compensation W QPID (no tracking) ---
        elif self.control_mode == "Gravity Compensation":
            self.tau_desired = self.robot_data.get_gravity()

        # --- Mode: Gravity Compensation W QPID (no tracking) ---
        elif self.control_mode == "Gravity Compensation W QPID":
            self.link_ee_task[self.ee_link_name].xddot = np.zeros(6)
            self.tau_desired = self.robot_controller.QPID(link_task_data=self.link_ee_task)

        # Format output for simulator actuators
        return {f"fr3_joint{i+1}": float(self.tau_desired[i]) for i in range(self.dof)}

    def _set_mode(self, control_mode: str) -> None:
        """
        Switch control mode and trigger per-mode re-initialization.

        Parameters
        ----------
        control_mode : str
            One of {"Home", "QPIK", "Gravity Compensation", "Gravity Compensation W QPID"}.
        """
        self.is_control_mode_changed = True
        self.control_mode = control_mode
        print(f"Control Mode Changed: {self.control_mode}")

    def _on_key_press(self, key) -> None:
        """Global hotkeys for mode switching: 1=Home, 2=QPIK, 3=Gravity Compensation, 4=Gravity Compensation W QPID."""
        try:
            if key.char == '1':
                self._set_mode("Home")
            elif key.char == '2':
                self._set_mode("QPIK")
            elif key.char == '3':
                self._set_mode("Gravity Compensation")
            elif key.char == '4':
                self._set_mode("Gravity Compensation W QPID")
        except AttributeError:
            # Ignore non-character keys
            pass
