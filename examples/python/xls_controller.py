# ============================= XLS Mecanum Base Controller =============================
import numpy as np
import threading
from typing import Dict
from pynput import keyboard  # Non-blocking, global keyboard listener

from drc import KinematicParam, DriveType
from drc.mobile import RobotData
from drc.mobile import RobotController


class XLSController:
    """
    XLSController
    -------------
    Minimal example of using Dyros Robot Controller for a 4-wheel Mecanum base.

    - Keeps internal state (wheel positions/velocities, desired base twist).
    - Switches control modes via global hotkeys (1/2).
    - On every mode switch, uses a consistent one-time initialization block.

    Parameters
    ----------
    dt : float
        Simulation/control timestep [s].
    """

    def __init__(self, dt: float):
        # --- Core configuration/state (same style as FR3) ---
        self.dt: float = dt
        self.sim_time: float = 0.0

        # Define mobile base kinematics (Mecanum drive) and constraints
        # All geometry is expressed in the base frame:
        #  - base2wheel_positions: 2D position of each wheel center wrt base origin
        #  - base2wheel_angles: wheel steering angles (fixed here)
        #  - roller_angles: Mecanum roller angles for each wheel
        #  - max_*: saturation limits (used by controller for velocity/acceleration limits)
        param = KinematicParam(
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

        # Instantiate dyros robot model/controller
        self.robot_data = RobotData(self.dt, param)
        self.robot_controller = RobotController(robot_data=self.robot_data)

        # Dimensions
        self.wheel_num: int = self.robot_data.get_wheel_num()

        # Wheel states/commands
        self.wheel_pos = np.zeros(self.wheel_num)        # measured wheel angles
        self.wheel_vel = np.zeros(self.wheel_num)        # measured wheel speeds
        self.wheel_vel_desired = np.zeros(self.wheel_num)  # command (wheel speeds)

        # Desired base twist in base frame: [vx, vy, w]
        self.base_vel_desired = np.zeros(3)

        # --- Mode bookkeeping ---
        self.control_mode: str = "Stop"
        self.is_control_mode_changed: bool = True
        self.control_start_time: float = 0.0
        
        # Print XLS Kinematic Parameter info
        print("info:")
        print(self.robot_data.get_verbose())

        # Global keyboard listener (tracks held keys for continuous command)
        self._pressed_keys = set()
        self._keys_lock = threading.Lock()
        self._listener = keyboard.Listener(on_press=self._on_key_press, on_release=self._on_key_release)
        self._listener.daemon = True
        self._listener.start()
        print("[XLSController] Keyboard: [1]=Stop, [2]=Base velocity Tracking")

    def update_model(self, current_time: float, qpos_dict: Dict[str, float], qvel_dict: Dict[str, float]) -> None:
        """
        Update internal model/state from simulator readings.

        Parameters
        ----------
        current_time : float
            Simulator time [s].
        qpos_dict : Dict[str, float]
            Mapping of wheel joint name -> position (rad).
        qvel_dict : Dict[str, float]
            Mapping of wheel joint name -> velocity (rad/s).
        """
        # Time update (shared convention)
        self.sim_time = current_time

        # Read wheel states (ordering must match KinematicParam definition)
        self.wheel_pos[:] = np.array([
            qpos_dict["front_left_wheel"],
            qpos_dict["front_right_wheel"],
            qpos_dict["rear_left_wheel"],
            qpos_dict["rear_right_wheel"],
        ])
        self.wheel_vel[:] = np.array([
            qvel_dict["front_left_wheel"],
            qvel_dict["front_right_wheel"],
            qvel_dict["rear_left_wheel"],
            qvel_dict["rear_right_wheel"],
        ])

        # Push to dyros robot model
        self.robot_data.update_state(self.wheel_pos, self.wheel_vel)

    def compute(self) -> Dict[str, float]:
        """
        Compute control command for the current mode.

        Returns
        -------
        Dict[str, float]
            Actuator velocity map: wheel joint name -> velocity command.
        """
        # One-time init per mode entry
        if self.is_control_mode_changed:
            self.is_control_mode_changed = False
            self.control_start_time = self.sim_time

        # --- Mode: Stop (zero wheel speeds) ---
        if self.control_mode == "Stop":
            self.wheel_vel_desired[:] = 0.0

        # --- Mode: Base Velocity Tracking (map desired twist -> wheel speeds) ---
        elif self.control_mode == "Base Velocity Tracking":
            self.base_vel_desired = self._keys_to_xdot()
            self.wheel_vel_desired = self.robot_controller.velocity_command(self.base_vel_desired)

        # Format output for simulator actuators
        return {
            "front_left_wheel":  float(self.wheel_vel_desired[0]),
            "front_right_wheel": float(self.wheel_vel_desired[1]),
            "rear_left_wheel":   float(self.wheel_vel_desired[2]),
            "rear_right_wheel":  float(self.wheel_vel_desired[3]),
        }

    def _set_mode(self, control_mode: str) -> None:
        """
        Switch control mode and trigger per-mode re-initialization.

        Parameters
        ----------
        control_mode : str
            One of {"Stop", "Base Velocity Tracking"}.
        """
        self.is_control_mode_changed = True
        self.control_mode = control_mode
        print(f"Control Mode Changed: {self.control_mode}")

    def _on_key_press(self, key) -> None:
        """Global hotkeys: 1=Stop, 2=Base Velocity Tracking(placeholder)."""
        # Track held keys for continuous command
        with self._keys_lock:
            self._pressed_keys.add(key)
        # Mode hotkeys
        try:
            if key.char == '1':
                self._set_mode("Stop")
            elif key.char == '2':
                self._set_mode("Base Velocity Tracking")
                print("=================================================")
                print("Arrow Up/Down : +vx / -vx (forward/backward)")
                print("Arrow Left/Right : +vy / -vy (left/right strafe)")
                print("'b' / 'v' : +w / -w (yaw rate)")
                print("=================================================")
        except AttributeError:
            # Ignore non-character keys
            pass

    def _on_key_release(self, key) -> None:
        """Stop contributing that axis when key is released (continuous control semantics)."""
        with self._keys_lock:
            self._pressed_keys.discard(key)

    def _keys_to_xdot(self) -> np.ndarray:
        """
        Convert currently held keys into a desired base twist [vx, vy, w].

        Controls
        --------
        Arrow Up/Down : +vx / -vx (forward/backward)
        Arrow Left/Right : +vy / -vy (left/right strafe)
        'b' / 'v' : +w / -w (yaw rate)

        Returns
        -------
        np.ndarray
            Desired base twist [vx, vy, w] in the base frame.
        """
        vx = vy = w = 0.0
        with self._keys_lock:
            held = set(self._pressed_keys)

        # Linear components with arrow keys
        if keyboard.Key.up in held:      vx += 1.0
        if keyboard.Key.down in held:    vx -= 1.0
        if keyboard.Key.left in held:    vy += 1.0
        if keyboard.Key.right in held:   vy -= 1.0

        # Yaw rate with letters
        chars = [getattr(k, 'char', None) for k in held]
        if 'b' in chars:  w += 1.0
        if 'v' in chars:  w -= 1.0

        return np.array([vx, vy, w], dtype=float)
