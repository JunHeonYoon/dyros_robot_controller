#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>
#include <filesystem>
#include <unordered_set>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include "math_type_define.h"

#include "dyros_robot_controller/type_define.h"

using namespace Eigen;

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Abstract base class for manipulator robot data.
         * 
         * This class provides a general interface and shared logic for manipulators.
         * It supports state update, forward kinematics, dynamics computation and
         * Jacobian, minimum distance and manipulability calculation based on the specified kinematic parameters.
         */
        class RobotData
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param dt            (double) Control loop time step in seconds.
                 * @param urdf_path     (std::string) Path to the URDF file.
                 * @param srdf_path     (std::string) Path to the SRDF file.
                 * @param packages_path (std::string) Path to the packages directory.
                 */
                RobotData(const double dt,
                          const std::string& urdf_path, 
                          const std::string& srdf_path="", 
                          const std::string& packages_path="");
                /**
                 * @brief Constructor.
                 * @param dt            (double) Control loop time step in seconds.
                 * @param urdf_source   (std::string) URDF file path or URDF XML string (when use_xml is true).
                 * @param srdf_source   (std::string) SRDF file path or SRDF XML string (when use_xml is true).
                 * @param packages_path (std::string) Path to the packages directory.
                 * @param use_xml       (bool) If true, treat URDF/SRDF inputs as XML strings.
                 */
                RobotData(const double dt,
                          const std::string& urdf_source,
                          const std::string& srdf_source,
                          const std::string& packages_path,
                          const bool use_xml);
                /**
                 * @brief Update the state of the manipulator.
                 * @param q     (Eigen::VectorXd) Joint positions.
                 * @param qdot  (Eigen::VectorXd) Joint velocities.
                 * @return (bool) True if state update is successful.
                */            
                virtual bool updateState(const VectorXd& q, const VectorXd& qdot);
                /**
                 * @brief Prints debug information.
                 * @return (std::string) Debug information.
                 */
                virtual std::string getVerbose() const;
                // ================================ Compute Functions ================================
                // Joint space 
                /**
                 * @brief Compute the mass matrix of the manipulator.
                 * @param q (Eigen::VectorXd) Joint positions.
                 * @return (Eigen::MatrixXd) Mass matrix of the manipulator.
                 */
                virtual MatrixXd computeMassMatrix(const VectorXd& q);
                /**
                 * @brief Compute the gravity vector of the manipulator.
                 * @param q (Eigen::VectorXd) Joint positions.
                 * @return (Eigen::VectorXd) Gravity vector of the manipulator.
                 */
                virtual VectorXd computeGravity(const VectorXd& q);
                /**
                 * @brief Compute the coriolis vector of the manipulator.
                 * @param q     (Eigen::VectorXd) Joint positions.
                 * @param qdot  (Eigen::VectorXd) Joint velocities.
                 * @return (Eigen::VectorXd) Coriolis vector of the manipulator.
                 */
                virtual VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
                /**
                 * @brief Compute the nonlinear effects vector of the manipulator.
                 * @param q     (Eigen::VectorXd) Joint positions.
                 * @param qdot  (Eigen::VectorXd) Joint velocities.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the manipulator.
                 */
                virtual VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);
                
                // Task space
                /**
                 * @brief Compute the pose of the link in the task space.
                 * @param q         (Eigen::VectorXd) Joint positions.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::Affine3d) Pose of the link in the task space.
                 */
                virtual Affine3d computePose(const VectorXd& q, const std::string& link_name);
                /**
                 * @brief Compute the Jacobian of the link.
                 * @param q         (Eigen::VectorXd) Joint positions.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link.
                 */
                virtual MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name);
                /**
                 * @brief Compute the Jacobian time variation of the link.
                 * @param q         (Eigen::VectorXd) Joint positions.
                 * @param qdot      (Eigen::VectorXd) Joint velocities.
                 * @param link_name (std::string) Name of the link.
                 * @return (MatrixXd) Jacobian time variation of the link.
                 */
                virtual MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name);
                /**
                 * @brief Compute the velocity of the link in the task space.
                 * @param q         (Eigen::VectorXd) Joint positions.
                 * @param qdot      (Eigen::VectorXd) Joint velocities.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Velocity of the link in the task space.
                 */
                virtual VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name);
                /**
                 * @brief Compute the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.
                 * @param q             (Eigen::VectorXd) Joint positions.
                 * @param qdot          (Eigen::VectorXd) Joint velocities.
                 * @param with_grad     (bool) If true, computes the gradient of the minimum distance.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the minimum distance.
                 * @param verbose       (bool) If true, prints the closest pair of links and their minimum distance.
                 * @return (MinDistResult) Minimum distance result containing distance, gradient, and gradient time variation.
                 */
                virtual MinDistResult computeMinDistance(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const bool verbose);
                /**
                 * @brief Compute the manipulability of the link (which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param q             (Eigen::VectorXd) Joint positions.
                 * @param qdot          (Eigen::VectorXd) Joint velocities.
                 * @param with_grad     (bool) If true, computes the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the manipulability.
                 * @param link_name     (std::string) Name of the link.
                 * @return (ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                 */
                virtual ManipulabilityResult computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const std::string& link_name);
                
                // ================================ Get Functions ================================
                const std::string getURDFPath() const {return urdf_path_;}
                const std::string getSRDFPath() const {return srdf_path_;}
                const std::string getPackagePath() const {return packages_path_;}
                double getDt() const {return dt_;}

                // Link frames (BODY)
                const std::vector<std::string>& getLinkFrameVector() const { return link_frame_names_; }
                bool hasLinkFrame(const std::string& name) const;

                // Joint frames (JOINT)
                const std::vector<std::string>& getJointFrameVector() const { return joint_frame_names_; }
                bool hasJointFrame(const std::string& name) const;

                // Root link (base link default)
                const std::string& getRootLinkName() const { return root_link_name_; }

                /**
                 * @brief Get the degrees of freedom of the manipulator.
                 * @return (int) Degrees of freedom of the manipulator.
                 */
                virtual int getDof() const {return dof_;}

                // Joint space
                /**
                 * @brief Get the joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Joint positions of the manipulator.
                 */
                virtual VectorXd getJointPosition() const {return q_;}
                /**
                 * @brief Get the joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Joint velocities of the manipulator.
                 */
                virtual VectorXd getJointVelocity() const {return qdot_;}
                /**
                 * @brief Get lower and upper joint position limits of the manipulator.
                 * @return (std::pair<Eigen::VectorXd, Eigen::VectorXd>) Joint position limits (lower, upper) of the manipulator.
                 */
                virtual std::pair<VectorXd,VectorXd> getJointPositionLimit() const {return std::make_pair(q_lb_, q_ub_);}
                /**
                 * @brief Get lower and upper joint velocity limits of the manipulator.
                 * @return (std::pair<Eigen::VectorXd, Eigen::VectorXd>) Joint velocity limits (lower, upper) of the manipulator.
                 */
                virtual std::pair<VectorXd,VectorXd> getJointVelocityLimit() const {return std::make_pair(qdot_lb_, qdot_ub_);}
                /**
                 * @brief Get lower and upper joint effort limits of the manipulator.
                 * @return (std::pair<Eigen::VectorXd, Eigen::VectorXd>) Joint torque limits (lower, upper) of the manipulator.
                 */
                virtual std::pair<VectorXd,VectorXd> getJointEffortLimit() const {return std::make_pair(torque_lb_, torque_ub_);}
                /**
                 * @brief Get the mass matrix of the manipulator.
                 * @return (Eigen::MatrixXd) Mass matrix of the manipulator.
                 */
                virtual MatrixXd getMassMatrix() const {return M_;}
                /**
                 * @brief Get the inversed mass matrix of the manipulator.
                 * @return (Eigen::MatrixXd) Inversed mass matrix of the manipulator.
                 */
                virtual MatrixXd getMassMatrixInv() const {return M_inv_;}
                /**
                 * @brief Get the coriolis vector of the manipulator.
                 * @return (Eigen::VectorXd) Coriolis vector of the manipulator.
                 */
                virtual VectorXd getCoriolis() const {return c_;}
                /**
                 * @brief Get the gravity vector of the manipulator.
                 * @return (Eigen::VectorXd) Gravity vector of the manipulator.
                 */
                virtual VectorXd getGravity() const {return g_;}
                /**
                 * @brief Get the nonlinear effects vector of the manipulator.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the manipulator.
                 */
                virtual VectorXd getNonlinearEffects() const {return NLE_;}

                // Task space
                /**
                 * @brief Get the pose of the link in the task space.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::Affine3d) Pose of the link in the task space.
                 */
                virtual Affine3d getPose(const std::string& link_name) const;
                /**
                 * @brief Get the Jacobian of the link.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link.
                 */
                virtual MatrixXd getJacobian(const std::string& link_name);
                /**
                 * @brief Get the Jacobian time variation of the link.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link.
                 */
                virtual MatrixXd getJacobianTimeVariation(const std::string& link_name); 
                /**
                 * @brief Get the velocity of the link in the task space.
                 * @param link_name (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Velocity of the link in the task space.
                 */
                virtual VectorXd getVelocity(const std::string& link_name);
                /**
                 * @brief Get the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.
                 * @param with_grad     (bool) If true, get the gradient of the minimum distance.
                 * @param with_graddot  (bool) If true, get the gradient time variation of the minimum distance.
                 * @param verbose       (bool) If true, prints the closest pair of links and their minimum distance.
                 * @return (MinDistResult) Minimum distance result containing distance, gradient, and gradient time variation.
                 */
                virtual MinDistResult getMinDistance(const bool& with_grad, const bool& with_graddot, const bool verbose);
                /**
                 * @brief Get the manipulability of the link (which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param with_grad     (bool) If true, get the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, get the gradient time variation of the manipulability.
                 * @param link_name     (std::string) Name of the link.
                 * @return (ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                 */
                virtual ManipulabilityResult getManipulability(const bool& with_grad, const bool& with_graddot, const std::string& link_name);

            protected:
                /**
                 * @brief Update the kinematic parameters of the manipulator.
                 * @param q     (Eigen::VectorXd) Joint positions.
                 * @param qdot  (Eigen::VectorXd) Joint velocities.
                 * @return (bool) True if the update was successful.
                 */
                virtual bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
                /**
                 * @brief Update the dynamic parameters of the manipulator.
                 * @param q     (Eigen::VectorXd) Joint positions.
                 * @param qdot  (Eigen::VectorXd) Joint velocities.
                 * @return (bool) True if the update was successful.
                 */
                virtual bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

                std::string urdf_path_;
                std::string srdf_path_;
                std::string packages_path_;
        
                // pinocchio data
                pinocchio::Model model_;                // Robot's structure containing joints, frames, inertias etc.
                pinocchio::Data data_;                  // Storage for computation results.
                pinocchio::GeometryModel geom_model_;   // Geometry objects aligned with each link's frames.(static)
                pinocchio::GeometryData geom_data_;     // Storage for poses of all geometry objects computed from current joint configuration.

                // Cached frame name lists
                std::vector<std::string> link_frame_names_;   // URDF <link> names
                std::vector<std::string> joint_frame_names_;  // URDF <joint> names

                std::unordered_set<std::string> link_frame_set_;
                std::unordered_set<std::string> joint_frame_set_;

                std::string root_link_name_;

                int dof_;           // Total degrees of freedom.
                double dt_;         // Control time step in seconds.
                
                // Joint space state
                VectorXd q_;        // Manipulator joint positions.
                VectorXd qdot_;     // Manipulator joint velocities.
                VectorXd q_lb_;     // Lower joint position limits of the manipulator.
                VectorXd q_ub_;     // Upper joint position limits of the manipulator.
                VectorXd qdot_lb_;  // Lower joint velocity limits of the manipulator.
                VectorXd qdot_ub_;  // Upper joint velocity limits of the manipulator.
                VectorXd torque_lb_;
                VectorXd torque_ub_;

                // Joint space Dynamics
                MatrixXd M_;        // Mass matrix of the manipulator.
                MatrixXd M_inv_;    // Inversed Mass matrix of the manipulator.
                VectorXd g_;        // Gravity vector of the manipulator.
                VectorXd c_;        // Coriolis vector of the manipulator.
                VectorXd NLE_;      // Nonlinear effects vector of the manipulator.
        };
    } //namespce Manipulator
} // namespace drc
