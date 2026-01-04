from enum import IntEnum
from typing import List
import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp

class DriveType(IntEnum):
    Differential = int(drc_cpp.DriveType.Differential)
    Mecanum      = int(drc_cpp.DriveType.Mecanum)
    Caster       = int(drc_cpp.DriveType.Caster)
    
class KinematicParam:
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
        return self._cpp
    
class JointIndex:
    def __init__(self, 
                 virtual_start: int, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        j = drc_cpp.JointIndex()
        self.virtual_start = int(virtual_start)
        self.mani_start    = int(mani_start)
        self.mobi_start    = int(mobi_start)
        
        j.virtual_start = self.virtual_start
        j.mobi_start    = self.mobi_start
        j.mani_start    = self.mani_start
        
        self._cpp = j
        
    def cpp(self) -> drc_cpp.JointIndex:
        return self._cpp

class ActuatorIndex:
    def __init__(self, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        a = drc_cpp.ActuatorIndex()
        self.mani_start = int(mani_start)
        self.mobi_start = int(mobi_start)
        
        a.mani_start = self.mani_start
        a.mobi_start = self.mobi_start
        
        self._cpp = a
        
    def cpp(self) -> drc_cpp.ActuatorIndex:
        return self._cpp

class TaskSpaceData:
    """
    Python mirror of C++ TaskSpaceData.

    - x, x_init, x_desired: (4, 4) homogeneous transform (SE(3))
    - xdot, xddot and *_init, *_desired: (6, ) twist/accel [v; w]
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
                 xddot_desired: np.ndarray | None = None) -> None:

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

    def cpp(self) -> drc_cpp.TaskSpaceData:
        """Return the wrapped C++ TaskSpaceData."""
        # NOTE: ensure C++ always sees the latest numpy values
        self._sync_to_cpp()
        return self._cpp

    def setZero(self) -> None:
        """Reset all pose and derivative data to identity/zero (same as C++ setZero())."""
        self.x[:] = np.eye(4)
        self.xdot[:] = 0.0
        self.xddot[:] = 0.0

        self.x_init[:] = np.eye(4)
        self.xdot_init[:] = 0.0
        self.xddot_init[:] = 0.0

        self.x_desired[:] = np.eye(4)
        self.xdot_desired[:] = 0.0
        self.xddot_desired[:] = 0.0

        self._sync_to_cpp()

    @staticmethod
    def Zero() -> "TaskSpaceData":
        """Create a zero-initialized TaskSpaceData (same as C++ static Zero())."""
        t = TaskSpaceData()
        t.setZero()
        return t

    def setInit(self) -> None:
        """Store current state into *_init (same as C++ setInit())."""
        self.x_init[:] = self.x
        self.xdot_init[:] = self.xdot
        self.xddot_init[:] = self.xddot
        self._sync_to_cpp()

    def setDesired(self) -> None:
        """Store current state into *_desired (same as C++ setDesired())."""
        self.x_desired[:] = self.x
        self.xdot_desired[:] = self.xdot
        self.xddot_desired[:] = self.xddot
        self._sync_to_cpp()
