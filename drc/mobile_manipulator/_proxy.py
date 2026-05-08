# Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Python-level proxy sub-objects for MobileManipulator.

After construction:
    moma_data.mani  -> _ManiDataProxy  (arm-only RobotData view, mobile-base frame)
    moma_data.mobi  -> _MobiDataProxy  (mobile-base RobotData view)
    moma_data.moma  -> moma_data itself

    moma_ctrl.mani  -> _ManiCtrlProxy  (arm-only RobotController view)
    moma_ctrl.mobi  -> _MobiCtrlProxy  (mobile-base RobotController view)
    moma_ctrl.moma  -> moma_ctrl itself
"""

from __future__ import annotations
import numpy as np
from drc import TaskSpaceData


def _to_cpp_tasks(link_task_data: dict) -> dict:
    return {link: task.cpp() if hasattr(task, "cpp") else task
            for link, task in link_task_data.items()}


def _to_cpp_hierarchy(hierarchy: list) -> list:
    return [_to_cpp_tasks(level) for level in hierarchy]


# ──────────────────────────────────────────────────────────────────────────────
# RobotData proxies
# ──────────────────────────────────────────────────────────────────────────────

class _ManiDataProxy:
    """
    Arm-only RobotData view backed by C++ ManipulatorProxy.

    - compute_*(q_mani, ...) methods take **arm-only** joint vectors.
    - All poses, Jacobians, and dynamics are expressed in the **mobile-base frame**.
    - get_*() cached getters are valid after moma_data.update_state().
    """

    def __init__(self, cpp_mani_proxy, parent_cpp_moma):
        self._cpp  = cpp_mani_proxy   # drc_cpp.ManipulatorRobotData (ManipulatorProxy)
        self._moma = parent_cpp_moma  # drc_cpp.MobileManipulatorRobotData

    # ── Info ──────────────────────────────────────────────────────────────────
    @property
    def dof(self) -> int:
        return int(self._cpp.getDof())

    def get_dof(self) -> int:
        return int(self._cpp.getDof())

    def get_dt(self) -> float:
        return float(self._moma.getDt())

    def get_link_frame_vector(self) -> list:
        return self._cpp.getLinkFrameVector()

    def has_link_frame(self, name: str) -> bool:
        return self._cpp.hasLinkFrame(name)

    def get_joint_position_limit(self):
        lo, hi = self._cpp.getJointPositionLimit()
        return np.array(lo), np.array(hi)

    def get_joint_velocity_limit(self):
        lo, hi = self._cpp.getJointVelocityLimit()
        return np.array(lo), np.array(hi)

    def get_joint_effort_limit(self):
        lo, hi = self._cpp.getJointEffortLimit()
        return np.array(lo), np.array(hi)

    # ── Cached state (updated by moma_data.update_state) ─────────────────────
    def get_joint_position(self) -> np.ndarray:
        return np.array(self._moma.getManiJointPosition())

    def get_joint_velocity(self) -> np.ndarray:
        return np.array(self._moma.getManiJointVelocity())

    def get_mass_matrix(self) -> np.ndarray:
        return np.array(self._cpp.getMassMatrix())

    def get_mass_matrix_inv(self) -> np.ndarray:
        return np.array(self._cpp.getMassMatrixInv())

    def get_gravity(self) -> np.ndarray:
        return np.array(self._cpp.getGravity())

    def get_coriolis(self) -> np.ndarray:
        return np.array(self._cpp.getCoriolis())

    def get_nonlinear_effects(self) -> np.ndarray:
        return np.array(self._cpp.getNonlinearEffects())

    def get_pose(self, link_name: str) -> np.ndarray:
        """Link pose in the mobile-base frame (4×4 matrix)."""
        return np.array(self._cpp.getPose(link_name))

    def get_jacobian(self, link_name: str) -> np.ndarray:
        """Arm Jacobian in the mobile-base frame, shape (6, mani_dof)."""
        return np.array(self._cpp.getJacobian(link_name))

    def get_jacobian_time_variation(self, link_name: str) -> np.ndarray:
        return np.array(self._cpp.getJacobianTimeVariation(link_name))

    def get_velocity(self, link_name: str) -> np.ndarray:
        return np.array(self._cpp.getVelocity(link_name))

    def get_min_distance(self, with_grad: bool = False, with_graddot: bool = False):
        return self._cpp.getMinDistance(with_grad, with_graddot)

    def get_manipulability(self, with_grad: bool = False, with_graddot: bool = False,
                            link_name: str = ""):
        return self._cpp.getManipulability(with_grad, with_graddot, link_name)

    # ── Explicit compute (q_mani input, base-frame output) ───────────────────
    def compute_mass_matrix(self, q_mani: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeMassMatrix(
            np.asarray(q_mani, dtype=float).reshape(-1)))

    def compute_gravity(self, q_mani: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeGravity(
            np.asarray(q_mani, dtype=float).reshape(-1)))

    def compute_coriolis(self, q_mani: np.ndarray, qdot_mani: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeCoriolis(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1)))

    def compute_nonlinear_effects(self, q_mani: np.ndarray,
                                   qdot_mani: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeNonlinearEffects(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1)))

    def compute_pose(self, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        """Compute link pose in the mobile-base frame for given q_mani."""
        return np.array(self._cpp.computePose(
            np.asarray(q_mani, dtype=float).reshape(-1), link_name))

    def compute_jacobian(self, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        """Compute arm Jacobian in the mobile-base frame, shape (6, mani_dof)."""
        return np.array(self._cpp.computeJacobian(
            np.asarray(q_mani, dtype=float).reshape(-1), link_name))

    def compute_jacobian_time_variation(self, q_mani: np.ndarray,
                                         qdot_mani: np.ndarray,
                                         link_name: str) -> np.ndarray:
        return np.array(self._cpp.computeJacobianTimeVariation(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1),
            link_name))

    def compute_velocity(self, q_mani: np.ndarray, qdot_mani: np.ndarray,
                          link_name: str) -> np.ndarray:
        return np.array(self._cpp.computeVelocity(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1),
            link_name))

    def compute_min_distance(self, q_mani, qdot_mani,
                              with_grad: bool, with_graddot: bool):
        return self._cpp.computeMinDistance(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1),
            with_grad, with_graddot)

    def compute_manipulability(self, q_mani, qdot_mani,
                                with_grad: bool, with_graddot: bool,
                                link_name: str):
        return self._cpp.computeManipulability(
            np.asarray(q_mani, dtype=float).reshape(-1),
            np.asarray(qdot_mani, dtype=float).reshape(-1),
            with_grad, with_graddot, link_name)


class _MobiDataProxy:
    """
    Mobile-base RobotData view. Mirrors drc.mobile.RobotData interface.
    get_*() cached getters are valid after moma_data.update_state().
    """

    def __init__(self, cpp_mobi_proxy, parent_cpp_moma):
        self._cpp  = cpp_mobi_proxy
        self._moma = parent_cpp_moma

    @property
    def wheel_num(self) -> int:
        return int(self._cpp.getWheelNum())

    def get_wheel_num(self) -> int:
        return int(self._cpp.getWheelNum())

    def get_dt(self) -> float:
        return float(self._moma.getDt())

    def get_wheel_pos(self) -> np.ndarray:
        return np.array(self._moma.getMobileJointPosition())

    def get_wheel_vel(self) -> np.ndarray:
        return np.array(self._moma.getMobileJointVelocity())

    def get_base_vel(self) -> np.ndarray:
        return np.array(self._moma.getMobileBaseVel())

    def get_fk_jacobian(self) -> np.ndarray:
        return np.array(self._moma.getMobileFKJacobian())

    def compute_base_vel(self, wheel_pos: np.ndarray,
                          wheel_vel: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeBaseVel(
            np.asarray(wheel_pos, dtype=float).reshape(-1),
            np.asarray(wheel_vel, dtype=float).reshape(-1)))

    def compute_fk_jacobian(self, wheel_pos: np.ndarray) -> np.ndarray:
        return np.array(self._cpp.computeFKJacobian(
            np.asarray(wheel_pos, dtype=float).reshape(-1)))


# ──────────────────────────────────────────────────────────────────────────────
# RobotController proxies
# ──────────────────────────────────────────────────────────────────────────────

class _ManiCtrlProxy:
    """
    Arm-only RobotController view backed by C++ mani_ctrl_ (Manipulator::RobotController).

    - Gain setters modify the shared controller state (same effect as calling them on moma).
    - Joint-space methods operate on arm DOFs only.
    - Task-space methods (CLIK, OSF, QP*) use arm-only Jacobians in the mobile-base frame
      and return (ok, qdot_arm) / (ok, tau_arm) with mani_dof-sized vectors.
    """

    def __init__(self, cpp_mani_ctrl, parent_moma_ctrl):
        self._cpp  = cpp_mani_ctrl     # drc_cpp.ManipulatorRobotController
        self._moma = parent_moma_ctrl  # Python MobileManipulator RobotController

    # ── Gain setters ──────────────────────────────────────────────────────────
    def set_joint_gain(self, kp=None, kv=None):
        self._moma.set_manipulator_joint_gain(kp, kv)

    def set_joint_Kp_gain(self, kp):
        self._moma.set_manipulator_joint_gain(kp=kp)

    def set_joint_Kv_gain(self, kv):
        self._moma.set_manipulator_joint_gain(kv=kv)

    def set_IK_gain(self, kp=None, **kwargs):
        self._moma.set_IK_gain(kp, **kwargs)

    def set_ID_gain(self, kp=None, kv=None, **kwargs):
        self._moma.set_ID_gain(kp, kv, **kwargs)

    def set_QPIK_gain(self, **kwargs):
        self._moma.set_QPIK_gain(**kwargs)

    def set_QPID_gain(self, **kwargs):
        self._moma.set_QPID_gain(**kwargs)

    def set_HQPIK_gain(self, **kwargs):
        self._moma.set_HQPIK_gain(**kwargs)

    def set_HQPID_gain(self, **kwargs):
        self._moma.set_HQPID_gain(**kwargs)

    # ── Joint-space arm control ───────────────────────────────────────────────
    def move_joint_position_cubic(self, *args):
        return self._moma.move_manipulator_joint_position_cubic(*args)

    def move_joint_velocity_cubic(self, *args):
        return self._moma.move_manipulator_joint_velocity_cubic(*args)

    def move_joint_torque_step(self, *args):
        return self._moma.move_manipulator_joint_torque_step(*args)

    def move_joint_torque_cubic(self, *args):
        return self._moma.move_manipulator_joint_torque_cubic(*args)

    # ── Task-space arm-only ───────────────────────────────────────────────────
    # Returns (ok, qdot_arm) or (ok, tau_arm) with mani_dof-sized vectors.

    def CLIK(self, link_task_data, null_qdot=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_qdot is None:
            return self._cpp.CLIK(tasks)
        return self._cpp.CLIK(tasks,
                               np.asarray(null_qdot, dtype=float).reshape(-1))

    def CLIK_step(self, link_task_data, null_qdot=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_qdot is None:
            return self._cpp.CLIKStep(tasks)
        return self._cpp.CLIKStep(tasks,
                                   np.asarray(null_qdot, dtype=float).reshape(-1))

    def CLIK_cubic(self, link_task_data, duration: float, null_qdot=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_qdot is None:
            return self._cpp.CLIKCubic(tasks, float(duration))
        return self._cpp.CLIKCubic(tasks, float(duration),
                                    np.asarray(null_qdot, dtype=float).reshape(-1))

    def OSF(self, link_task_data, null_torque=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_torque is None:
            return self._cpp.OSF(tasks)
        return self._cpp.OSF(tasks,
                              np.asarray(null_torque, dtype=float).reshape(-1))

    def OSF_step(self, link_task_data, null_torque=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_torque is None:
            return self._cpp.OSFStep(tasks)
        return self._cpp.OSFStep(tasks,
                                  np.asarray(null_torque, dtype=float).reshape(-1))

    def OSF_cubic(self, link_task_data, duration: float, null_torque=None):
        tasks = _to_cpp_tasks(link_task_data)
        if null_torque is None:
            return self._cpp.OSFCubic(tasks, float(duration))
        return self._cpp.OSFCubic(tasks, float(duration),
                                   np.asarray(null_torque, dtype=float).reshape(-1))

    def QPIK(self, link_task_data):
        return self._cpp.QPIK(_to_cpp_tasks(link_task_data))

    def QPIK_step(self, link_task_data):
        return self._cpp.QPIKStep(_to_cpp_tasks(link_task_data))

    def QPIK_cubic(self, link_task_data, duration: float):
        return self._cpp.QPIKCubic(_to_cpp_tasks(link_task_data), float(duration))

    def QPID(self, link_task_data):
        return self._cpp.QPID(_to_cpp_tasks(link_task_data))

    def QPID_step(self, link_task_data):
        return self._cpp.QPIDStep(_to_cpp_tasks(link_task_data))

    def QPID_cubic(self, link_task_data, duration: float):
        return self._cpp.QPIDCubic(_to_cpp_tasks(link_task_data), float(duration))

    def HQPIK(self, task_hierarchy):
        return self._cpp.HQPIK(_to_cpp_hierarchy(task_hierarchy))

    def HQPIK_step(self, task_hierarchy):
        return self._cpp.HQPIKStep(_to_cpp_hierarchy(task_hierarchy))

    def HQPIK_cubic(self, task_hierarchy, duration: float):
        return self._cpp.HQPIKCubic(_to_cpp_hierarchy(task_hierarchy), float(duration))

    def HQPID(self, task_hierarchy):
        return self._cpp.HQPID(_to_cpp_hierarchy(task_hierarchy))

    def HQPID_step(self, task_hierarchy):
        return self._cpp.HQPIDStep(_to_cpp_hierarchy(task_hierarchy))

    def HQPID_cubic(self, task_hierarchy, duration: float):
        return self._cpp.HQPIDCubic(_to_cpp_hierarchy(task_hierarchy), float(duration))


class _MobiCtrlProxy:
    """
    Mobile-base RobotController view. Mirrors drc.mobile.RobotController interface.
    """

    def __init__(self, cpp_mobi_ctrl, parent_moma_ctrl):
        self._cpp  = cpp_mobi_ctrl
        self._moma = parent_moma_ctrl

    def compute_wheel_vel(self, base_vel: np.ndarray) -> np.ndarray:
        return np.array(self._moma.computeMobileWheelVel(
            np.asarray(base_vel, dtype=float).reshape(-1)))

    def compute_IK_jacobian(self) -> np.ndarray:
        return np.array(self._moma.computeMobileIKJacobian())

    def velocity_command(self, desired_base_vel: np.ndarray) -> np.ndarray:
        desired_base_vel = np.asarray(desired_base_vel, dtype=float).reshape(-1)
        assert desired_base_vel.size == 3, \
            f"desired_base_vel size {desired_base_vel.size} must be 3"
        return np.array(self._moma.MobileVelocityCommand(desired_base_vel))
