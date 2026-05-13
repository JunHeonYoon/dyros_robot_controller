# Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
#
# Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
# Seoul National University, under a research agreement with ETRI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import IntEnum
from typing import List
import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp

class DriveType(IntEnum):
    """Enum class defining the type of mobile drive system. (Differential, Mecanum, Caster)"""

    Differential = int(drc_cpp.DriveType.Differential)
    Mecanum      = int(drc_cpp.DriveType.Mecanum)
    Caster       = int(drc_cpp.DriveType.Caster)
    
class KinematicParam:
    """
    Kinematic parameters for a mobile robot base.

    This structure contains geometry and dynamic constraints for various mobile base types
    (Differential, Mecanum, or Caster).

    Required fields and valid values vary depending on the selected drive type.
    """

    def __init__(self,
                 type: DriveType,
                 wheel_radius: float,
                 max_lin_speed: float = 2.0,
                 max_ang_speed: float = 2.0,
                 max_lin_acc: float = 2.0,
                 max_ang_acc: float = 2.0,
                 base_width: float | None = None,
                 roller_angles: List | None = None,
                 base2wheel_positions: List[np.ndarray] | None = None,
                 base2wheel_angles: List | None = None,
                 wheel_offset: float | None = None,
                 ) -> None:
        """
        Constructor for mobile base kinematic parameters.

        Parameters:
            type                 : (DriveType) [Required] Drive type of the mobile base.
            wheel_radius         : (float) [Required] Radius of the wheels [m].
            max_lin_speed        : (float) [Optional] Maximum linear speed in m/s. Default = 2.0.
            max_ang_speed        : (float) [Optional] Maximum angular speed in rad/s. Default = 2.0.
            max_lin_acc          : (float) [Optional] Maximum linear acceleration in m/s^2. Default = 2.0.
            max_ang_acc          : (float) [Optional] Maximum angular acceleration in rad/s^2. Default = 2.0.
            base_width           : (float | None) [Required if type == DriveType.Differential]
                                   Distance between left and right wheels [m].
            roller_angles        : (List | None) [Required if type == DriveType.Mecanum]
                                   Vector of roller angles [rad] between base and wheel frame.
            base2wheel_positions : (List[np.ndarray] | None) [Required if type == DriveType.Mecanum or Caster]
                                   Positions of wheels relative to base frame [m].
            base2wheel_angles    : (List | None) [Required if type == DriveType.Mecanum or Caster]
                                   Orientation angle of each wheel relative to base frame [rad].
            wheel_offset         : (float | None) [Required if type == DriveType.Caster]
                                   Offset from caster joint rotation axis to wheel center [m].
        """

        p = drc_cpp.KinematicParam()
        self.type                 = type
        self.wheel_radius         = wheel_radius
        self.max_lin_speed        = max_lin_speed
        self.max_ang_speed        = max_ang_speed
        self.max_lin_acc          = max_lin_acc
        self.max_ang_acc          = max_ang_acc
        self.base_width           = base_width
        self.roller_angles        = roller_angles
        self.base2wheel_positions = base2wheel_positions
        self.base2wheel_angles    = base2wheel_angles
        self.wheel_offset         = wheel_offset

        p.type = drc_cpp.DriveType(int(type))
        p.wheel_radius = wheel_radius
        p.max_lin_speed = max_lin_speed
        p.max_ang_speed = max_ang_speed
        p.max_lin_acc = max_lin_acc
        p.max_ang_acc = max_ang_acc

        if type == DriveType.Differential:
            assert(base_width is not None)
            p.base_width = base_width
        elif type == DriveType.Mecanum:
            assert(roller_angles is not None)
            assert(base2wheel_angles is not None)
            assert(base2wheel_positions is not None)
            p.roller_angles        = roller_angles
            p.base2wheel_angles    = base2wheel_angles
            p.base2wheel_positions = base2wheel_positions
        elif type == DriveType.Caster:
            assert(wheel_offset is not None)
            assert(base2wheel_positions is not None)
            p.wheel_offset         = wheel_offset
            p.base2wheel_positions = base2wheel_positions

        self._cpp = p

    def cpp(self) -> drc_cpp.KinematicParam:
        """
        Return the wrapped C++ KinematicParam.

        Return:
            (drc_cpp.KinematicParam) C++ kinematic parameter object.
        """
        return self._cpp
    
class JointIndex:
    """
    Index offsets used in unified joint state vectors.

    In a mobile manipulator system, the full joint vector is typically composed of:
    virtual joints, manipulator joints, and mobile base joints.
    This structure defines the starting indices of each group within a single
    concatenated joint vector.
    """

    def __init__(self, 
                 virtual_start: int, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        """
        Constructor for unified joint index offsets.

        Parameters:
            virtual_start : (int) Index at which virtual joints start in the full joint vector.
            mani_start    : (int) Index at which manipulator joints start.
            mobi_start    : (int) Index at which mobile wheel joints start.
        """
        j = drc_cpp.JointIndex()
        self.virtual_start = int(virtual_start)
        self.mani_start    = int(mani_start)
        self.mobi_start    = int(mobi_start)
        
        j.virtual_start = self.virtual_start
        j.mobi_start    = self.mobi_start
        j.mani_start    = self.mani_start
        
        self._cpp = j
        
    def cpp(self) -> drc_cpp.JointIndex:
        """
        Return the wrapped C++ JointIndex.

        Return:
            (drc_cpp.JointIndex) C++ joint index object.
        """
        return self._cpp

class ActuatorIndex:
    """
    Index offsets used in unified actuator vectors.

    The actuator vector may be composed of manipulator actuators and mobile base
    actuators. This structure defines the starting indices of each group within
    that vector.
    """

    def __init__(self, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        """
        Constructor for unified actuator index offsets.

        Parameters:
            mani_start : (int) Index at which manipulator actuator commands begin.
            mobi_start : (int) Index at which mobile wheel actuator commands begin.
        """
        a = drc_cpp.ActuatorIndex()
        self.mani_start = int(mani_start)
        self.mobi_start = int(mobi_start)
        
        a.mani_start = self.mani_start
        a.mobi_start = self.mobi_start
        
        self._cpp = a
        
    def cpp(self) -> drc_cpp.ActuatorIndex:
        """
        Return the wrapped C++ ActuatorIndex.

        Return:
            (drc_cpp.ActuatorIndex) C++ actuator index object.
        """
        return self._cpp

class TaskSpaceData:
    """
    Task-space state and trajectory information for a robot link or frame.

    This structure stores current, initial, and desired task-space states
    (pose, velocity, and acceleration). It is used for task-space control,
    trajectory tracking, and logging.
    """

    def __init__(self,
                 x: np.ndarray | None = None,
                 xdot: np.ndarray | None = None,
                 xddot: np.ndarray | None = None,
                 x_init: np.ndarray | None = None,
                 xdot_init: np.ndarray | None = None,
                 xddot_init: np.ndarray | None = None,
                 x_desired: np.ndarray | None = None,
                 xdot_desired: np.ndarray | None = None,
                 xddot_desired: np.ndarray | None = None,
                 current_time: float = 0.0,
                 control_start_time: float = 0.0) -> None:
        """
        Constructor for task-space state and trajectory data.

        Parameters:
            x                  : (np.ndarray | None) Current pose of the frame, shape (4, 4).
            xdot               : (np.ndarray | None) Current task-space velocity [v, w], size 6.
            xddot              : (np.ndarray | None) Current task-space acceleration, size 6.
            x_init             : (np.ndarray | None) Initial pose, shape (4, 4).
            xdot_init          : (np.ndarray | None) Initial velocity, size 6.
            xddot_init         : (np.ndarray | None) Initial acceleration, size 6.
            x_desired          : (np.ndarray | None) Desired pose for task-space control, shape (4, 4).
            xdot_desired       : (np.ndarray | None) Desired task-space velocity, size 6.
            xddot_desired      : (np.ndarray | None) Desired task-space acceleration, size 6.
            current_time       : (float) Current time.
            control_start_time : (float) Initial control time.
        """

        # --------- Allocate defaults (identity / zeros) ----------
        self.x             = np.eye(4) if x is None else np.array(x, dtype=float, copy=True)
        self.xdot          = np.zeros(6) if xdot is None else np.array(xdot, dtype=float, copy=True).reshape(-1)
        self.xddot         = np.zeros(6) if xddot is None else np.array(xddot, dtype=float, copy=True).reshape(-1)

        self.x_init        = np.eye(4) if x_init is None else np.array(x_init, dtype=float, copy=True)
        self.xdot_init     = np.zeros(6) if xdot_init is None else np.array(xdot_init, dtype=float, copy=True).reshape(-1)
        self.xddot_init    = np.zeros(6) if xddot_init is None else np.array(xddot_init, dtype=float, copy=True).reshape(-1)

        self.x_desired     = np.eye(4) if x_desired is None else np.array(x_desired, dtype=float, copy=True)
        self.xdot_desired  = np.zeros(6) if xdot_desired is None else np.array(xdot_desired, dtype=float, copy=True).reshape(-1)
        self.xddot_desired = np.zeros(6) if xddot_desired is None else np.array(xddot_desired, dtype=float, copy=True).reshape(-1)

        self.current_time = float(current_time)
        self.control_start_time = float(control_start_time)

        # --------- Shape checks ----------
        self._assert_shapes()

        # --------- Create and sync C++ object ----------
        self._cpp = drc_cpp.TaskSpaceData()
        self._sync_to_cpp()

    def _assert_shapes(self) -> None:
        """Validate array shapes."""
        assert self.x.shape == (4, 4), f"Shape of x {self.x.shape} is not equal to (4, 4)"
        assert self.xdot.size == 6, f"Size of xdot {self.xdot.size} is not equal to 6"
        assert self.xddot.size == 6, f"Size of xddot {self.xddot.size} is not equal to 6"

        assert self.x_init.shape == (4, 4), f"Shape of x_init {self.x_init.shape} is not equal to (4, 4)"
        assert self.xdot_init.size == 6, f"Size of xdot_init {self.xdot_init.size} is not equal to 6"
        assert self.xddot_init.size == 6, f"Size of xddot_init {self.xddot_init.size} is not equal to 6"

        assert self.x_desired.shape == (4, 4), f"Shape of x_desired {self.x_desired.shape} is not equal to (4, 4)"
        assert self.xdot_desired.size == 6, f"Size of xdot_desired {self.xdot_desired.size} is not equal to 6"
        assert self.xddot_desired.size == 6, f"Size of xddot_desired {self.xddot_desired.size} is not equal to 6"

    def _sync_to_cpp(self) -> None:
        """Copy current numpy fields into the wrapped C++ object."""
        self._cpp.x = self.x
        self._cpp.xdot = self.xdot.reshape(6,1)
        self._cpp.xddot = self.xddot.reshape(6,1)

        self._cpp.x_init = self.x_init
        self._cpp.xdot_init = self.xdot_init.reshape(6,1)
        self._cpp.xddot_init = self.xddot_init.reshape(6,1)

        self._cpp.x_desired = self.x_desired
        self._cpp.xdot_desired = self.xdot_desired.reshape(6,1)
        self._cpp.xddot_desired = self.xddot_desired.reshape(6,1)

        self._cpp.current_time = self.current_time
        self._cpp.control_start_time = self.control_start_time

    def cpp(self) -> drc_cpp.TaskSpaceData:
        """Return the wrapped C++ TaskSpaceData."""
        # NOTE: ensure C++ always sees the latest numpy values
        self._sync_to_cpp()
        return self._cpp

    def setZero(self) -> None:
        """
        Reset all pose and derivative data to zero/identity.

        Sets x, x_init, and x_desired to identity transform, and sets all twist
        and acceleration vectors to zero.
        """
        self.x[:] = np.eye(4)
        self.xdot[:] = 0.0
        self.xddot[:] = 0.0

        self.x_init[:] = np.eye(4)
        self.xdot_init[:] = 0.0
        self.xddot_init[:] = 0.0

        self.x_desired[:] = np.eye(4)
        self.xdot_desired[:] = 0.0
        self.xddot_desired[:] = 0.0

        self.current_time = 0.0
        self.control_start_time = 0.0

        self._sync_to_cpp()

    @staticmethod
    def Zero() -> "TaskSpaceData":
        """
        Convenience function that returns a zero-initialized TaskSpaceData.

        Returns:
            (TaskSpaceData) New instance with all fields zeroed.
        """
        t = TaskSpaceData()
        t.setZero()
        return t

    def setInit(self) -> None:
        """
        Store the current task-space state into the *_init fields.

        Typical usage: call at the start of a motion or trajectory.
        """
        self.x_init[:] = self.x
        self.xdot_init[:] = self.xdot
        self.xddot_init[:] = self.xddot
        self.control_start_time = self.current_time
        self._sync_to_cpp()

    def setDesired(self) -> None:
        """
        Store the current task-space state into the *_desired fields.

        Often used when updating the reference trajectory in controllers.
        """
        self.x_desired[:] = self.x
        self.xdot_desired[:] = self.xdot
        self.xddot_desired[:] = self.xddot
        self._sync_to_cpp()
