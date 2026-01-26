from typing import Tuple
import numpy as np
from drc import KinematicParam, JointIndex, ActuatorIndex
import dyros_robot_controller_cpp_wrapper as drc_cpp


class RobotData(drc_cpp.MobileManipulatorRobotData):
    """
    A Python wrapper for the C++ RobotData::MobileManipulator::MobileManipulatorBase class.
    
    This class combines the functionality of a mobile base and a manipulator base.
    It provides methods for updating the state, computing kinematics and dynamics,
    and accessing kinematic parameters for both the mobile base and the manipulator.
    It supports a mobile manipulator with a floating base, manipulator joints, and mobile wheel joints.
    """
    def __init__(self,
                 dt: float,
                 mobile_param: KinematicParam,
                 joint_idx: JointIndex,
                 actuator_idx: ActuatorIndex,
                 urdf_path: str,
                 srdf_path: str="",
                 packages_path: str="",
                 ):
        """
        Constructor.

        Parameters:
            dt            : (float) Control loop time step in seconds.
            mobile_param  : (KinematicParam) Kinematic parameter instance containing drive type and geometry.
            joint_idx     : (JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
            actuator_idx  : (ActuatorIndex) Actuator index structure containing starting indices for manipulator and mobile actuators.
            urdf_path     : (str) Path to the URDF file.
            srdf_path     : (str) Path to the SRDF file.
            packages_path : (str) Path to the packages directory.
        """
        self._dt = float(dt)
        self._mobile_kine_param = mobile_param
        self._joint_idx = joint_idx
        self._actuator_idx = actuator_idx
        super().__init__(self._dt,
                         self._mobile_kine_param.cpp(),
                         self._joint_idx.cpp(),
                         self._actuator_idx.cpp(),
                         urdf_path,
                         srdf_path,
                         packages_path,
                         )
        self.mani_dof     = int(super().getManipulatorDof())
        self.mobi_dof     = int(super().getMobileDof())
        self.virtual_dof  = 3
        self.actuated_dof = int(super().getActuatordDof())
        
    def get_verbose(self) -> str:
        """
        Prints debug information.

        Return:
            (str) Debug information.
        """
        return super().getVerbose()

    def get_dt(self) -> float:
        """
        Get the control time step.

        Return:
            (float) Control loop time step in seconds.
        """
        return float(super().getDt())

    def update_state(self,
                     q_virtual: np.ndarray,
                     q_mobile: np.ndarray,
                     q_mani: np.ndarray,
                     qdot_virtual: np.ndarray,
                     qdot_mobile: np.ndarray,
                     qdot_mani: np.ndarray,
                     ) -> bool:
        """
        Update the state of the manipulator.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.

        Return:
            (bool) True if state update is successful.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().updateState(q_virtual,
                                   q_mobile,
                                   q_mani,
                                   qdot_virtual,
                                   qdot_mobile,
                                   qdot_mani,
                                   )

    # ================================ Compute Functions ================================
    # Wholebody joint space
    def compute_mass_matrix(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        """
        Compute the mass matrix of the whole body.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.

        Return:
            (np.ndarray) Mass matrix of the whole body.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeMassMatrix(q_virtual, q_mobile, q_mani)

    def compute_gravity(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        """
        Compute the gravity vector of the whole body.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.

        Return:
            (np.ndarray) Gravity vector of the whole body.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeGravity(q_virtual, q_mobile, q_mani)

    def compute_coriolis(self,
                         q_virtual: np.ndarray,
                         q_mobile: np.ndarray,
                         q_mani: np.ndarray,
                         qdot_virtual: np.ndarray,
                         qdot_mobile: np.ndarray,
                         qdot_mani: np.ndarray,
                         ) -> np.ndarray:
        """
        Compute the coriolis vector of the whole body.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.

        Return:
            (np.ndarray) Coriolis vector of the whole body.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeCoriolis(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani)

    def compute_nonlinear_effects(self,
                                  q_virtual: np.ndarray,
                                  q_mobile: np.ndarray,
                                  q_mani: np.ndarray,
                                  qdot_virtual: np.ndarray,
                                  qdot_mobile: np.ndarray,
                                  qdot_mani: np.ndarray,
                                  ) -> np.ndarray:
        """
        Compute the nonlinear effects vector of the whole body.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.

        Return:
            (np.ndarray) Nonlinear effects vector of the whole body.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeNonlinearEffects(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani)

    def compute_mass_matrix_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        """
        Compute the mass matrix of the whole body, excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.

        Return:
            (np.ndarray) Mass matrix of the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeMassMatrixActuated(q_virtual, q_mobile, q_mani)

    def compute_gravity_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        """
        Compute the gravity vector of the whole body, excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.

        Return:
            (np.ndarray) Gravity vector of the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeGravityActuated(q_virtual, q_mobile, q_mani)

    def compute_coriolis_actuated(self,
                                  q_virtual: np.ndarray,
                                  q_mobile: np.ndarray,
                                  q_mani: np.ndarray,
                                  qdot_mobile: np.ndarray,
                                  qdot_mani: np.ndarray,) -> np.ndarray:
        """
        Compute the coriolis vector of the whole body, excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.

        Return:
            (np.ndarray) Coriolis vector of the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeCoriolisActuated(q_virtual, q_mobile, q_mani, qdot_mobile, qdot_mani)

    def compute_nonlinear_effects_actuated(self,
                                           q_virtual: np.ndarray,
                                           q_mobile: np.ndarray,
                                           q_mani: np.ndarray,
                                           qdot_mobile: np.ndarray,
                                           qdot_mani: np.ndarray,
                                           ) -> np.ndarray:
        """
        Compute the nonlinear effects vector of the whole body, excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.

        Return:
            (np.ndarray) Nonlinear effects vector of the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeNonlinearEffectsActuated(q_virtual, q_mobile, q_mani, qdot_mobile, qdot_mani)

    # Wholebody task space
    def compute_pose(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the pose of the link in the task space.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Pose of the link in the task space.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computePose(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the Jacobian of the link.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian of the link.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeJacobian(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian_time_variation(self,
                                        q_virtual: np.ndarray,
                                        q_mobile: np.ndarray,
                                        q_mani: np.ndarray,
                                        qdot_virtual: np.ndarray,
                                        qdot_mobile: np.ndarray,
                                        qdot_mani: np.ndarray,
                                        link_name: str,
                                        ) -> np.ndarray:
        """
        Compute the Jacobian time variation of the link.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian time variation of the link.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeJacobianTimeVariation(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name)

    def compute_velocity(self,
                         q_virtual: np.ndarray,
                         q_mobile: np.ndarray,
                         q_mani: np.ndarray,
                         qdot_virtual: np.ndarray,
                         qdot_mobile: np.ndarray,
                         qdot_mani: np.ndarray,
                         link_name: str,
                         ) -> np.ndarray:
        """
        Compute the velocity of the link in the task space.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Velocity of the link in the task space.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeVelocity(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name,)

    def compute_min_distance(self,
                             q_virtual: np.ndarray,
                             q_mobile: np.ndarray,
                             q_mani: np.ndarray,
                             qdot_virtual: np.ndarray,
                             qdot_mobile: np.ndarray,
                             qdot_mani: np.ndarray,
                             with_grad: bool,
                             with_graddot: bool,
                             verbose: bool = False,
                             )->Tuple[float, np.ndarray, np.ndarray]:
        """
        Compute the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.
            with_grad    : (bool) If true, computes the gradient of the minimum distance.
            with_graddot : (bool) If true, computes the gradient time variation of the minimum distance.
            verbose      : (bool) If true, prints the closest pair of links and their minimum distance.

        Return:
            (Tuple[float, np.ndarray, np.ndarray]) Minimum distance, its gradient, and its gradient time variation.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        min_result = super().computeMinDistance(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, with_grad, with_graddot, verbose)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def compute_selection_matrix(self, q_virtual: np.ndarray, q_mobile: np.ndarray) -> np.ndarray:
        """
        Compute the selection matrix that excludes floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.

        Return:
            (np.ndarray) Selection matrix.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        return super().computeSelectionMatrix(q_virtual, q_mobile)

    def compute_jacobian_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the Jacobian of the link excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian of the link for the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        return  super().computeJacobianActuated(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian_time_variation_actuated(self,
                                                 q_virtual: np.ndarray,
                                                 q_mobile: np.ndarray,
                                                 q_mani: np.ndarray,
                                                 qdot_virtual: np.ndarray,
                                                 qdot_mobile: np.ndarray,
                                                 qdot_mani: np.ndarray,
                                                 link_name: str,
                                                 ) -> np.ndarray:
        """
        Compute the Jacobian time variation of the link excluding the floating base.

        Parameters:
            q_virtual    : (np.ndarray) Joint positions of the floating base.
            q_mobile     : (np.ndarray) Wheel positions.
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_virtual : (np.ndarray) Joint velocities of the floating base.
            qdot_mobile  : (np.ndarray) Joint velocities of the wheels.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.
            link_name    : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian time variation of the link for the actuated joints.
        """
        q_virtual = q_virtual.reshape(-1)
        q_mobile = q_mobile.reshape(-1)
        q_mani = q_mani.reshape(-1)
        qdot_virtual = qdot_virtual.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_virtual.size    == self.virtual_dof, f"Size of q_virtual {q_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert qdot_virtual.size == self.virtual_dof, f"Size of qdot_virtual {qdot_virtual.size} is not equal to virtual_dof {self.virtual_dof}"
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        return super().computeJacobianTimeVariationActuated(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name)

    # Manipulator taskspace
    def compute_manipulability(self,
                               q_mani: np.ndarray,
                               qdot_mani: np.ndarray,
                               with_grad: bool,
                               with_graddot: bool,
                               link_name: str,
                               ):
        """
        Compute the manipulability of the link.(which indicates how well the link can move at current joint configuration) and (optionally) its time variations.

        Parameters:
            q_mani       : (np.ndarray) Joint positions of the manipulator.
            qdot_mani    : (np.ndarray) Joint velocities of the manipulator.
            with_grad    : (bool) If true, computes the gradient of the manipulability.
            with_graddot : (bool) If true, computes the gradient time variation of the manipulability.

        Return:
            (Tuple[float, np.ndarray, np.ndarray]) Manipulability, its gradient, and its gradient time variation.
        """
        q_mani = q_mani.reshape(-1)
        qdot_mani = qdot_mani.reshape(-1)
        assert q_mani.size       == self.mani_dof, f"Size of q_mani {q_mani.size} is not equal to mani_dof {self.mani_dof}"
        assert qdot_mani.size    == self.mani_dof, f"Size of qdot_mani {qdot_mani.size} is not equal to mani_dof {self.mani_dof}"
        mani_result = super().computeManipulability(q_mani, qdot_mani, with_grad, with_graddot, link_name)
        return mani_result.manipulability, mani_result.grad, mani_result.grad_dot

    # Mobile
    def compute_mobile_FK_jacobian(self, q_mobile: np.ndarray) -> np.ndarray:
        """
        Compute the Jacobian of the mobile base specific to the configured drive type.

        Parameters:
            q_mobile     : (np.ndarray) Wheel positions.

        Return:
            (np.ndarray) Jacobian of the mobile base.
        """
        q_mobile = q_mobile.reshape(-1)
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        return super().computeMobileFKJacobian(q_mobile)

    def compute_mobile_base_vel(self, q_mobile: np.ndarray, qdot_mobile: np.ndarray) -> np.ndarray:
        """
        Compute the velocity of the mobile base.

        Parameters:
            q_mobile    : (np.ndarray) Wheel positions.
            qdot_mobile : (np.ndarray) Joint velocities of the wheels.

        Return:
            (np.ndarray) Base velocity vector [vx, vy, wz].
        """
        q_mobile = q_mobile.reshape(-1)
        qdot_mobile = qdot_mobile.reshape(-1)
        assert q_mobile.size     == self.mobi_dof, f"Size of q_mobile {q_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        assert qdot_mobile.size  == self.mobi_dof, f"Size of qdot_mobile {qdot_mobile.size} is not equal to mobi_dof {self.mobi_dof}"
        return super().computeMobileBaseVel(q_mobile, qdot_mobile)

    # ================================ Get Functions ================================
    def get_urdf_path(self) -> str:
        return super().getURDFPath()
    def get_srdf_path(self) -> str:
        return super().getSRDFPath()
    def get_packages_path(self) -> str:
        return super().getPackagePath()
    def get_root_link_name(self) -> str:
        return super().getRootLinkName()
    def get_link_frame_vector(self) -> list:
        return super().getLinkFrameVector()
    def get_joint_frame_vector(self) -> list:
        return super().getJointFrameVector()
    def has_link_frame(self, name:str) -> bool:
        return super.hasLinkFrame(name)
    def has_joint_frame(self, name:str) -> bool:
        return super.hasJointFrame(name)
    def get_dof(self) -> int:
        """
        Get the full degrees of freedom of the robot.

        Return:
            (int) Full degrees of freedom.
        """
        return super().getDof()
    
    def get_actuator_dof(self) -> int:
        """
        Get the number of actuated joints.

        Return:
            (int) Number of actuated joints.
        """
        return int(super().getActuatordDof())

    def get_manipulator_dof(self) -> int:
        """
        Get the degrees of freedom of the manipulator.

        Return:
            (int) Degrees of freedom of the manipulator.
        """
        return int(super().getManipulatorDof())

    def get_mobile_dof(self) -> int:
        """
        Get the degrees of freedom of the mobile base.

        Return:
            (int) Degrees of freedom of the mobile base.
        """
        return int(super().getMobileDof())

    def get_joint_index(self) -> JointIndex:
        """
        Get the full joint index.

        Return:
            (JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
        """
        return self._joint_idx

    def get_actuator_index(self) -> ActuatorIndex:
        """
        Get the actuator joint index.

        Return:
            (JointIndex) Joint index structure containing starting indices for manipulator and mobile joints.
        """
        return self._actuator_idx
    
    def get_joint_position(self) -> np.ndarray:
        """
        Get the joint positions of the whole body.

        Return:
            (np.ndarray) Joint positions of the whole body.
        """
        return super().getJointPosition()

    def get_joint_velocity(self) -> np.ndarray:
        """
        Get the joint velocities of the whole body.

        Return:
            (np.ndarray) Joint velocities of the whole body.
        """
        return super().getJointVelocity()

    def get_joint_position_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get lower and upper joint position limits of the manipulator.
        
        return:
            (Tuple[np.ndarray, np.ndarray]) Joint position limits (lower, upper) of the manipulator.
        """
        return super().getJointPositionLimit()

    def get_joint_velocity_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get lower and upper joint velocity limits of the manipulator.
        
        return:
            (Tuple[np.ndarray, np.ndarray]) Joint velocity limits (lower, upper) of the manipulator.
        """
        return super().getJointVelocityLimit()

    def get_mobile_joint_position(self) -> np.ndarray:
        """
        Get the wheel positions.
        
        return:
            (np.ndarray) Wheel positions.
        """
        return super().getMobileJointPosition()

    def get_virtual_joint_position(self) -> np.ndarray:
        """
        Get the mobile base pose w.r.t world.
        
        return:
            (np.ndarray) Mobile base pose w.r.t world[x, y, yaw].
        """
        return super().getVirtualJointPosition()

    def get_manipulator_joint_position(self) -> np.ndarray:
        """
        Get the joint positions of the manipulator.
        
        return:
            (np.ndarray) Joint positions of the manipulator.
        """
        return super().getManiJointPosition()

    def get_joint_velocity_actuated(self) -> np.ndarray:
        """
        Get the joint velocities of the actuated joints.
        
        return:
            (np.ndarray) Joint velocities of the actuated joints.
        """
        return super().getJointVelocityActuated()

    def get_mobile_joint_velocity(self) -> np.ndarray:
        """
        Get the wheel velocities of the mobile base.
        
        return:
            (np.ndarray) Wheel velocities of the mobile base.
        """
        return super().getMobileJointVelocity()

    def get_virtual_joint_velocity(self) -> np.ndarray:
        """
        Get the mobile base velocity w.r.t world.
        
        return:
            (np.ndarray) Mobile base velocity w.r.t world[vx, vy, wz].
        """
        return super().getVirtualJointVelocity()

    def get_manipulator_joint_velocity(self) -> np.ndarray:
        """
        Get the joint velocities of the manipulator.
        
        return:
            (np.ndarray) Joint velocities of the manipulator.
        """
        return super().getManiJointVelocity()

    def get_joint_position_actuated(self) -> np.ndarray:
        """
        Get the joint positions of the actuated joints.
        
        return:
            (np.ndarray) Joint positions of the actuated joints.
        """
        return super().getJointPositionActuated()

    #  Wholebody joint space
    def get_mass_matrix(self) -> np.ndarray:
        """
        Get the mass matrix of the whole body.
        
        return:
            (np.ndarray) Mass matrix of the whole body.
        """
        return super().getMassMatrix()

    def get_mass_matrix_inv(self) -> np.ndarray:
        """
        Get the inversed mass matrix of the whole body.
        
        return:
            (np.ndarray) Inversed mass matrix of the whole body.
        """
        return super().getMassMatrixInv()

    def get_coriolis(self) -> np.ndarray:
        """
        Get the coriolis vector of the whole body.
        
        return:
            (np.ndarray) Coriolis vector of the whole body.
        """
        return super().getCoriolis()

    def get_gravity(self) -> np.ndarray:
        """
        Get the gravity vector of the whole body.
        
        return:
            (np.ndarray) Gravity vector of the whole body.
        """
        return super().getGravity()

    def get_nonlinear_effects(self) -> np.ndarray:
        """
        Get the nonlinear effects vector of the whole body.
        
        return:
            (np.ndarray) Nonlinear effects vector of the whole body.
        """
        return super().getNonlinearEffects()
    
    def get_mass_matrix_actuated(self) -> np.ndarray:
        """
        Get the mass matrix of the actuated joints.
        
        return:
            (np.ndarray) Mass matrix of the actuated joints.
        """
        return super().getMassMatrixActuated()

    def get_mass_matrix_actuated_inv(self) -> np.ndarray:
        """
        Get the inversed mass matrix of the actuated joints.
        
        return:
            (np.ndarray) Inversed Mass matrix of the actuated joints.
        """
        return super().getMassMatrixActuatedInv()

    def get_gravity_actuated(self) -> np.ndarray:
        """
        Get the gravity vector of the actuated joints.
        
        return:
            (np.ndarray) Gravity vector of the actuated joints.
        """
        return super().getGravityActuated()

    def get_coriolis_actuated(self) -> np.ndarray:
        """
        Get the coriolis vector of the actuated joints.
        
        return:
            (np.ndarray) Coriolis vector of the actuated joints.
        """
        return super().getCoriolisActuated()

    def get_nonlinear_effects_actuated(self) -> np.ndarray:
        """
        Get the nonlinear effects vector of the actuated joints.
        
        return:
            (np.ndarray) Nonlinear effects vector of the actuated joints.
        """
        return super().getNonlinearEffectsActuated()

    # Wholebody task space
    def get_pose(self, link_name: str) -> np.ndarray:
        """
        Get the pose of the link in the task space.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Pose of the link in the task space.
        """
        return super().getPose(link_name)

    def get_jacobian(self, link_name: str) -> np.ndarray:
        """
        Get the Jacobian of the link.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Jacobian of the link.
        """
        return super().getJacobian(link_name)

    def get_jacobian_time_variation(self, link_name: str) -> np.ndarray:
        """
        Get the Jacobian time variation of the link.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Jacobian time variation of the link.
        """
        return super().getJacobianTimeVariation(link_name)

    def get_velocity(self, link_name: str) -> np.ndarray:
        """
        Get the velocity of the link in the task space.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Velocity of the link in the task space.
        """
        return super().getVelocity(link_name)
    
    def get_min_distance(self, with_grad: bool, with_graddot: bool):
        """
        Get the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.

        Parameters:
            with_grad    : (bool) If true, computes the gradient of the minimum distance.
            with_graddot : (bool) If true, computes the gradient time variation of the minimum distance.
        
        return:
            (Tuple[float, np.ndarray, np.ndarray]) Minimum distance, its gradient, and its gradient time variation.
        """
        min_result = super().getMinDistance(with_grad, with_graddot)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def get_jacobian_actuated(self, link_name: str) -> np.ndarray:
        """
        Get the Jacobian of the link for the actuated joints.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Jacobian of the link for the actuated joints.
        """
        return super().getJacobianActuated(link_name)

    def get_jacobian_actuated_time_variation(self, link_name: str) -> np.ndarray:
        """
        Get the Jacobian time variation of the link for the actuated joints.

        Parameters:
            link_name : (str) Name of the link.
        
        return:
            (np.ndarray) Jacobian time variation of the link for the actuated joints.
        """
        return super().getJacobianActuatedTimeVariation(link_name)

    def get_selection_matrix(self) -> np.ndarray:
        """
        Get the selection matrix that excludes floating base.

        return:
            (np.ndarray) Selection matrix.
        """
        return super().getSelectionMatrix()

    def get_manipulability(self, with_grad: bool, with_graddot: bool, link_name:str):
        """
        Get the manipulability of the link (which indicates how well the link can move at current joint configuration) and (optionally) its time variations.

        Parameters:
            with_grad    : (bool) If true, get the gradient of the manipulability.
            with_graddot : (bool) If true, get the gradient time variation of the manipulability.
            link_name    : (str) Name of the link.

        Return:
            (Tuple[float, np.ndarray, np.ndarray]) Manipulability, its gradient, and its gradient time variation.
        """
        min_result = super().getManipulability(with_grad, with_graddot, link_name)
        return min_result.manipulability, min_result.grad, min_result.grad_dot
    
    # mobile getters
    def get_mobile_FK_jacobian(self) -> np.ndarray:
        """
        Get the Jacobian of the mobile base specific to the configured drive type.

        Returns:
            (np.ndarray) Jacobian of the mobile base.
        """
        return super().getMobileFKJacobian()

    def get_mobile_base_vel(self) -> np.ndarray:
        """
        Get the velocity of the mobile base.

        Returns:
            (np.ndarray) Base velocity vector [vx, vy, wz].
        """
        return super().getMobileBaseVel()
