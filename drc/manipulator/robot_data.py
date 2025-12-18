from typing import Tuple
import numpy as np
import dyros_robot_controller_cpp_wrapper as drc_cpp


class RobotData(drc_cpp.ManipulatorRobotData):
    """
    A Python wrapper for the C++ RobotData::Manipulator::ManipulatorBase class.
    
    This class provides a general interface and shared logic for manipulators.
    It supports state update, forward kinematics, dynamics computation and
    Jacobian, minimum distance and manipulability calculation based on the specified kinematic parameters.
    """
    def __init__(self, urdf_path: str, srdf_path: str="", packages_path: str=""):
        """
        Constructor.

        Parameters:
            urdf_path     : (str) Path to the URDF file.
            srdf_path     : (str) Path to the SRDF file.
            packages_path : (str) Path to the packages directory.
        """
        super().__init__(urdf_path, srdf_path, packages_path)
        self.dof = super().getDof()
        
    def get_verbose(self) -> str:
        """
        Prints debug information.

        Return:
            (str) Debug information.
        """
        return super().getVerbose()

    def update_state(self, q: np.ndarray, qdot: np.ndarray) -> bool:
        """
        Update the state of the manipulator.

        Parameters:
            q    : (np.ndarray) Joint positions.
            qdot : (np.ndarray) Joint velocities.

        Return:
            (bool) True if state update is successful.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        return super().updateState(q, qdot)

    # ================================ Compute Functions ================================
    # Joint space
    def compute_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Compute the mass matrix of the manipulator.

        Parameters:
            q : (np.ndarray) Joint positions.

        Return:
            (np.ndarray) Mass matrix of the manipulator.
        """
        q = q.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        return super().computeMassMatrix(q)

    def compute_gravity(self, q: np.ndarray) -> np.ndarray:
        """
        Compute the gravity vector of the manipulator.

        Parameters:
            q : (np.ndarray) Joint positions.

        Return:
            (np.ndarray) Gravity vector of the manipulator.
        """
        q = q.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        return super().computeGravity(q)

    def compute_coriolis(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """
        Compute the coriolis vector of the manipulator.

        Parameters:
            q    : (np.ndarray) Joint positions.
            qdot : (np.ndarray) Joint velocities.

        Return:
            (np.ndarray) Coriolis vector of the manipulator.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        return super().computeCoriolis(q, qdot)

    def compute_nonlinear_effects(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """
        Compute the nonlinear effects vector of the manipulator.

        Parameters:
            q    : (np.ndarray) Joint positions.
            qdot : (np.ndarray) Joint velocities.

        Return:
            (np.ndarray) Nonlinear effects vector of the manipulator.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        return super().computeNonlinearEffects(q, qdot)

    # Task space
    def compute_pose(self, q: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the pose of the link in the task space.

        Parameters:
            q         : (np.ndarray) Joint positions.
            link_name : (str) Name of the link.

        Return:
            (np.ndarray) Pose of the link in the task space.
        """
        q = q.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        return super().computePose(q, link_name)

    def compute_jacobian(self, q: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the Jacobian of the link.

        Parameters:
            q         : (np.ndarray) Joint positions.
            link_name : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian of the link.
        """
        q = q.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        return super().computeJacobian(q, link_name)

    def compute_jacobian_time_variation(self, q: np.ndarray, qdot: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the Jacobian time variation of the link.

        Parameters:
            q         : (np.ndarray) Joint positions.
            qdot      : (np.ndarray) Joint velocities.
            link_name : (str) Name of the link.

        Return:
            (np.ndarray) Jacobian time variation of the link.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        return super().computeJacobianTimeVariation(q, qdot, link_name)

    def compute_velocity(self, q: np.ndarray, qdot: np.ndarray, link_name: str) -> np.ndarray:
        """
        Compute the velocity of the link in the task space.

        Parameters:
            q         : (np.ndarray) Joint positions.
            qdot      : (np.ndarray) Joint velocities.
            link_name : (str) Name of the link.

        Return:
            (np.ndarray) Velocity of the link in the task space.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        return super().computeVelocity(q, qdot, link_name)
    
    def compute_min_distance(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool, with_graddot: bool) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        Compute the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.

        Parameters:
            q            : (np.ndarray) Joint positions.
            qdot         : (np.ndarray) Joint velocities.
            with_grad    : (bool) If true, computes the gradient of the minimum distance.
            with_graddot : (bool) If true, computes the gradient time variation of the minimum distance.

        Return:
            (Tuple[float, np.ndarray, np.ndarray]) Minimum distance, its gradient, and its gradient time variation.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        min_result = super().computeMinDistance(q, qdot, with_grad, with_graddot)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def compute_manipulability(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool, with_graddot: bool, link_name: str) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        Compute the manipulability of the link (which indicates how well the link can move at current joint configuration) and (optionally) its time variations.

        Parameters:
            q            : (np.ndarray) Joint positions.
            qdot         : (np.ndarray) Joint velocities.
            with_grad    : (bool) If true, computes the gradient of the minimum distance.
            with_graddot : (bool) If true, computes the gradient time variation of the minimum distance.
            link_name    : (str) Name of the link.

        Return:
            (Tuple[float, np.ndarray, np.ndarray]) Manipulability, its gradient, and its gradient time variation.
        """
        q = q.reshape(-1)
        qdot = qdot.reshape(-1)
        assert q.size == self.dof, f"Size of q {q.size} is not equal to dof {self.dof}"
        assert qdot.size == self.dof, f"Size of qot {qdot.size} is not equal to dof {self.dof}, 1"
        mani_result = super().computeManipulability(q, qdot, with_grad, with_graddot, link_name)
        return mani_result.manipulability, mani_result.grad, mani_result.grad_dot

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
        Get the degrees of freedom of the manipulator.

        Return:
            (int) Degrees of freedom of the manipulator.
        """
        return super().getDof()

    # Joint space
    def get_joint_position(self) -> np.ndarray:
        """
        Get the joint positions of the manipulator.

        return:
            (np.ndarray) Joint positions of the manipulator.
        """
        return super().getJointPosition()

    def get_joint_velocity(self) -> np.ndarray:
        """
        Get the joint velocities of the manipulator.

        return:
            (np.ndarray) Joint velocities of the manipulator.
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

    def get_mass_matrix(self) -> np.ndarray:
        """
        Get the mass matrix of the manipulator.

        return:
            (np.ndarray) Mass matrix of the manipulator.
        """
        return super().getMassMatrix()

    def get_mass_matrix_inv(self) -> np.ndarray:
        """
        Get the inversed mass matrix of the manipulator.

        return:
            (np.ndarray) Inversed mass matrix of the manipulator.
        """
        return super().getMassMatrixInv()

    def get_coriolis(self) -> np.ndarray:
        """
        Get the coriolis vector of the manipulator.

        return:
            (np.ndarray) Coriolis vector of the manipulator.
        """
        return super().getCoriolis()

    def get_gravity(self) -> np.ndarray:
        """
        Get the gravity vector of the manipulator.

        return:
            (np.ndarray) Gravity vector of the manipulator.
        """
        return super().getGravity()

    def get_nonlinear_effects(self) -> np.ndarray:
        """
        Get the nonlinear effects vector of the manipulator.

        return:
            (np.ndarray) Nonlinear effects vector of the manipulator.
        """
        return super().getNonlinearEffects()

    # Task space
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
        Get the Velocity of the link in the task space.

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
            with_grad    : (bool) If true, get the gradient of the minimum distance.
            with_graddot : (bool) If true, get the gradient time variation of the minimum distance.

        return:
            (Tuple[float, np.ndarray, np.ndarray]) Minimum distance, its gradient, and its gradient time variation.
        """
        min_result = super().getMinDistance(with_grad, with_graddot)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
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
