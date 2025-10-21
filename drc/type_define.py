from enum import IntEnum
from typing import List, Optional
import numpy as np
import dyros_robot_controller_cpp_wrapper as drc

class DriveType(IntEnum):
    Differential = int(drc.DriveType.Differential)
    Mecanum      = int(drc.DriveType.Mecanum)
    Caster       = int(drc.DriveType.Caster)
    
class KinematicParam:
    def __init__(self,
                 type: DriveType,
                 wheel_radius: float,
                 max_lin_speed: float = 2.0,
                 max_ang_speed: float = 2.0,
                 max_lin_acc: float = 2.0,
                 max_ang_acc: float = 2.0,
                 base_width: Optional[float] = None,
                 roller_angles: Optional[List] = None,
                 base2wheel_positions: Optional[List[np.ndarray]] = None,
                 base2wheel_angles: Optional[List] = None,
                 wheel_offset: Optional[float] = None,
                 ) -> None:

        p = drc.KinematicParam()

        p.type = drc.DriveType(int(type))
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

    def cpp(self) -> drc.KinematicParam:
        return self._cpp
    
class JointIndex:
    def __init__(self, 
                 virtual_start: int, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        j = drc.JointIndex()
        self.virtual_start = int(virtual_start)
        self.mani_start    = int(mani_start)
        self.mobi_start    = int(mobi_start)
        
        j.virtual_start = self.virtual_start
        j.mobi_start    = self.mobi_start
        j.mani_start    = self.mani_start
        
        self._cpp = j
        
    def cpp(self) -> drc.JointIndex:
        return self._cpp

class ActuatorIndex:
    def __init__(self, 
                 mani_start: int, 
                 mobi_start: int
                 ) -> None:
        a = drc.ActuatorIndex()
        self.mani_start = int(mani_start)
        self.mobi_start = int(mobi_start)
        
        a.mani_start = self.mani_start
        a.mobi_start = self.mobi_start
        
        self._cpp = a
        
    def cpp(self) -> drc.ActuatorIndex:
        return self._cpp
