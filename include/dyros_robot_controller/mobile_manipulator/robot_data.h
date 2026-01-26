#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>
#include <filesystem>

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
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/mobile/robot_data.h"

using namespace Eigen;

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Abstract base class for mobile manipulator robot data.
         * 
         * This class combines the functionality of a mobile base and a manipulator base.
         * It provides methods for updating the state, computing kinematics and dynamics,
         * and accessing kinematic parameters for both the mobile base and the manipulator.
         * It supports a mobile manipulator with a floating base, manipulator joints, and mobile wheel joints.
        */
        class RobotData : public Manipulator::RobotData, public Mobile::RobotData
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param dt            (double) Control loop time step in seconds.
                 * @param mobile_param  (RobotData::Mobile::KinematicParam) Kinematic parameter structure of the mobile base containing drive type and geometry.
                 * @param urdf_path     (std::string) Path to the URDF file.
                 * @param srdf_path     (std::string) Path to the SRDF file.
                 * @param packages_path (std::string) Path to the packages directory.
                 * @param joint_idx     (RobotData::JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
                 * @param actuator_idx  (RobotData::ActuatorIndex) Actuator index structure containing starting indices for manipulator and mobile actuators.
                */
                RobotData(const double dt,
                          const Mobile::KinematicParam& mobile_param,
                          const JointIndex& joint_idx,
                          const ActuatorIndex& actuator_idx,
                          const std::string& urdf_path,
                          const std::string& srdf_path="", 
                          const std::string& packages_path="");

                using Manipulator::RobotData::getVerbose;
                using Mobile::RobotData::getVerbose;
                std::string getVerbose() const;
                double getDt() const { return Mobile::RobotData::getDt(); }
                                               
                using Manipulator::RobotData::updateState; 
                /**
                 * @brief Update internal mobile manipulator robot data.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if state update is successful.
                */
                bool updateState(const VectorXd& q_virtual,
                                 const VectorXd& q_mobile,
                                 const VectorXd& q_mani,
                                 const VectorXd& qdot_virtual,
                                 const VectorXd& qdot_mobile,
                                 const VectorXd& qdot_mani);

                // ================================ Compute Functions ================================
                // Wholebody joint space
                using Manipulator::RobotData::computeMassMatrix; 
                using Manipulator::RobotData::computeGravity; 
                using Manipulator::RobotData::computeCoriolis; 
                using Manipulator::RobotData::computeNonlinearEffects; 
                
                /**
                 * @brief Compute the mass matrix of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::MatrixXd) Mass matrix of the whole body.
                */ 
                virtual MatrixXd computeMassMatrix(const VectorXd& q_virtual,
                                                   const VectorXd& q_mobile,
                                                   const VectorXd& q_mani);
                /**
                 * @brief Compute the gravity vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Gravity vector of the whole body.
                */                                     
                virtual VectorXd computeGravity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani);
                /**
                 * @brief Compute the coriolis vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Coriolis vector of the whole body.
                */                                 
                virtual VectorXd computeCoriolis(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani,
                                                 const VectorXd& qdot_virtual,
                                                 const VectorXd& qdot_mobile,
                                                 const VectorXd& qdot_mani);
                /**
                 * @brief Compute the nonlinear effects vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the whole body.
                */                                  
                virtual VectorXd computeNonlinearEffects(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani,
                                                         const VectorXd& qdot_virtual,
                                                         const VectorXd& qdot_mobile,
                                                         const VectorXd& qdot_mani);

                /**
                 * @brief Compute the mass matrix of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::MatrixXd) Mass matrix of the actuated joints.
                */                                         
                virtual MatrixXd computeMassMatrixActuated(const VectorXd& q_virtual,
                                                           const VectorXd& q_mobile,
                                                           const VectorXd& q_mani);
                /**
                 * @brief Compute the gravity vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Gravity vector of the actuated joints.
                */                                              
                virtual VectorXd computeGravityActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani);
                /**
                 * @brief Compute the coriolis vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Coriolis vector of the actuated joints.
                */                                         
                virtual VectorXd computeCoriolisActuated(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani,
                                                         const VectorXd& qdot_mobile,
                                                         const VectorXd& qdot_mani);
                /**
                 * @brief Compute the nonlinear effects vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the actuated joints.
                */                                         
                virtual VectorXd computeNonlinearEffectsActuated(const VectorXd& q_virtual,
                                                                 const VectorXd& q_mobile,
                                                                 const VectorXd& q_mani,
                                                                 const VectorXd& qdot_mobile,
                                                                 const VectorXd& qdot_mani);

                // Wholebody task space
                using Manipulator::RobotData::computePose;
                using Manipulator::RobotData::computeJacobian;
                using Manipulator::RobotData::computeJacobianTimeVariation;
                using Manipulator::RobotData::computeVelocity;
                using Manipulator::RobotData::computeMinDistance;

                /**
                 * @brief Compute the pose of the link in the task space.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::Affine3d) Pose of the link in the task space.
                */ 
                virtual Affine3d computePose(const VectorXd& q_virtual,
                                             const VectorXd& q_mobile,
                                             const VectorXd& q_mani, 
                                             const std::string& link_name);
                /**
                 * @brief Compute the Jacobian of the link.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link.
                */                              
                virtual MatrixXd computeJacobian(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani, 
                                                 const std::string& link_name);
                /**
                 * @brief Compute the Jacobian time variation of the link.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link.
                */                                 
                virtual MatrixXd computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                              const VectorXd& q_mobile,
                                                              const VectorXd& q_mani,
                                                              const VectorXd& qdot_virtual,
                                                              const VectorXd& qdot_mobile,
                                                              const VectorXd& qdot_mani, 
                                                              const std::string& link_name);
                /**
                 * @brief Compute the velocity of the link in the task space.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Velocity of the link in the task space.
                */                                               
                virtual VectorXd computeVelocity(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani,
                                                 const VectorXd& qdot_virtual,
                                                 const VectorXd& qdot_mobile,
                                                 const VectorXd& qdot_mani, 
                                                 const std::string& link_name);
                /**
                 * @brief Compute the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param with_grad     (bool) If true, computes the gradient of the minimum distance.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the minimum distance.
                 * @param verbose       (bool) If true, prints the closest pair of links and their minimum distance.
                 * @return (RobotData::Manipulator::MinDistResult) Minimum distance result containing distance, gradient, and gradient time variation.
                */                                  
                virtual Manipulator::MinDistResult computeMinDistance(const VectorXd& q_virtual,
                                                                      const VectorXd& q_mobile,
                                                                      const VectorXd& q_mani,
                                                                      const VectorXd& qdot_virtual,
                                                                      const VectorXd& qdot_mobile,
                                                                      const VectorXd& qdot_mani, 
                                                                      const bool& with_grad, 
                                                                      const bool& with_graddot, 
                                                                      const bool verbose);
                /**
                 * @brief Compute the selection matrix that excludes floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @return (Eigen::MatrixXd) Selection matrix.
                 */                                                      
                virtual MatrixXd computeSelectionMatrix(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile);

                /**
                 * @brief Compute the Jacobian of the link excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link for the actuated joints.
                 */                                        
                virtual MatrixXd computeJacobianActuated(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani, 
                                                         const std::string& link_name);
                /**
                 * @brief Compute the Jacobian time variation of the link excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link for the actuated joints.
                 */                                         
                virtual MatrixXd computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                      const VectorXd& q_mobile,
                                                                      const VectorXd& q_mani,
                                                                      const VectorXd& qdot_virtual,
                                                                      const VectorXd& qdot_mobile,
                                                                      const VectorXd& qdot_mani, 
                                                                      const std::string& link_name);
                                                        
                                                              
                // Manipulator taskspace
                using Manipulator::RobotData::computeManipulability;
                /**
                 * @brief Compute the manipulability of the link.(which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param with_grad     (bool) If true, computes the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the manipulability.
                 * @param link_name     (std::string) Name of the link.
                 * @return (RobotData::Manipulator::ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                */
                virtual Manipulator::ManipulabilityResult computeManipulability(const VectorXd& q_mani, 
                                                                                const VectorXd& qdot_mani, 
                                                                                const bool& with_grad, 
                                                                                const bool& with_graddot, 
                                                                                const std::string& link_name);
                                                                        
                // Mobile
                /**
                 * @brief Compute the Jacobian of the mobile base specific to the configured drive type.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @return (Eigen::MatrixXd) Jacobian of the mobile base.
                 */
                virtual MatrixXd computeMobileFKJacobian(const VectorXd& q_mobile);
                /**
                 * @brief Compute the velocity of the mobile base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @return (Eigen::VectorXd) Base velocity vector [vx, vy, wz].
                 */
                virtual VectorXd computeMobileBaseVel(const VectorXd& q_mobile, const VectorXd& qdot_mobile);

                // ================================ Get Functions ================================
                /**
                 * @brief Get the number of actuated joints.
                 * @return (int) Number of actuated joints.
                */
                virtual int getActuatordDof() const {return actuated_dof_;}
                /**
                 * @brief Get the degrees of freedom of the manipulator.
                 * @return (int) Degrees of freedom of the manipulator.
                */
                virtual int getManipulatorDof() const {return mani_dof_;}
                /**
                 * @brief Get the degrees of freedom of the mobile base.
                 * @return (int) Degrees of freedom of the mobile base.
                */
                virtual int getMobileDof() const {return mobi_dof_;}
                /**
                 * @brief Get the full joint index.
                 * @return (RobotData::MobileManipulator::JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
                */
                virtual JointIndex getJointIndex() const {return joint_idx_;}
                /**
                 * @brief Get the actuator joint index.
                 * @return (RobotData::MobileManipulator::ActuatorIndex) Joint index structure containing starting indices for manipulator and mobile joints.
                */
                virtual ActuatorIndex getActuatorIndex() const {return actuator_idx_;}
                /**
                 * @brief Get the wheel positions.
                 * @return (Eigen::VectorXd) Wheel positions.
                */
                virtual VectorXd getMobileJointPosition() const {return q_mobile_;}
                /**
                 * @brief Get the mobile base pose w.r.t world.
                 * @return (Eigen::VectorXd) Mobile base pose w.r.t world[x, y, yaw].
                */
                virtual VectorXd getVirtualJointPosition() const {return q_virtual_;}
                /**
                 * @brief Get the joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Joint positions of the manipulator.
                */
                virtual VectorXd getManiJointPosition() const {return q_mani_;}
                /**
                 * @brief Get the joint velocities of the actuated joints.
                 * @return (Eigen::VectorXd) Joint velocities of the actuated joints.
                */
                virtual VectorXd getJointVelocityActuated() const {return qdot_actuated_;}
                /**
                 * @brief Get the wheel velocities of the mobile base.
                 * @return (Eigen::VectorXd) Wheel velocities of the mobile base.
                */
                virtual VectorXd getMobileJointVelocity() const {return qdot_mobile_;}
                /**
                 * @brief Get the mobile base velocity w.r.t world.
                 * @return (Eigen::VectorXd) Mobile base velocity w.r.t world[vx, vy, wz].
                */
                virtual VectorXd getVirtualJointVelocity() const {return qdot_virtual_;}
                /**
                 * @brief Get the joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Joint velocities of the manipulator.
                */
                virtual VectorXd getManiJointVelocity() const {return qdot_mani_;}
                /**
                 * @brief Get the joint positions of the actuated joints.
                 * @return (Eigen::VectorXd) Joint positions of the actuated joints.
                */
                virtual VectorXd getJointPositionActuated() const {return q_actuated_;}
                
                // Wholebody joint space
                /**
                 * @brief Get the mass matrix of the actuated joints.
                 * @return (Eigen::MatrixXd) Mass matrix of the actuated joints.
                 */
                virtual MatrixXd getMassMatrixActuated() const {return M_actuated_;}
                /**
                 * @brief Get the inversed mass matrix of the actuated joints.
                 * @return (Eigen::MatrixXd) Inversed mass matrix of the actuated joints.
                 */
                virtual MatrixXd getMassMatrixActuatedInv() const {return M_inv_actuated_;}
                /**
                 * @brief Get the gravity vector of the actuated joints.
                 * @return (Eigen::VectorXd) Gravity vector of the actuated joints.
                 */
                virtual VectorXd getGravityActuated() const {return g_actuated_;}
                /**
                 * @brief Get the coriolis vector of the actuated joints.
                 * @return (Eigen::VectorXd) Coriolis vector of the actuated joints.
                 */
                virtual VectorXd getCoriolisActuated() const {return c_actuated_;}
                /**
                 * @brief Get the nonlinear effects vector of the actuated joints.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the actuated joints.
                 */
                virtual VectorXd getNonlinearEffectsActuated() const {return NLE_actuated_;}

                // Wholebody task space
                /**
                 * @brief Get the Jacobian of the link for the actuated joints.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link for the actuated joints.
                 */
                virtual MatrixXd getJacobianActuated(const std::string& link_name);
                /**
                 * @brief Get the Jacobian time variation of the link for the actuated joints.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link for the actuated joints.
                 */
                virtual MatrixXd getJacobianActuatedTimeVariation(const std::string& link_name);
                /**
                 * @brief Get the selection matrix that excludes floating base.
                 * @return (Eigen::MatrixXd) Selection matrix.
                 */
                virtual MatrixXd getSelectionMatrix(){return S_;}

                // Manipulator taskspace
                using Manipulator::RobotData::getManipulability;
                /**
                 * @brief Get the manipulability of the link.(which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param with_grad     (bool) If true, get the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, get the time variation of the gradient.
                 * @param link_name     (std::string) Name of the link.
                 * @return (RobotData::Manipulator::ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                 */
                virtual Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, 
                                                                            const bool& with_graddot, 
                                                                            const std::string& link_name);

                // Mobile
                /**
                 * @brief Get the Jacobian of the mobile base specific to the configured drive type.
                 * @return (Eigen::MatrixXd) Jacobian of the mobile base.
                 */
                virtual MatrixXd getMobileFKJacobian() const {return J_mobile_;}
                /**
                 * @brief Get the velocity of the mobile base.
                 * @return (Eigen::VectorXd) Base velocity vector [vx, vy, wz].
                 */
                virtual VectorXd getMobileBaseVel() const {return Mobile::RobotData::getBaseVel();}
        

            protected:
                using Manipulator::RobotData::updateKinematics;
                using Manipulator::RobotData::updateDynamics;
                /**
                 * @brief Update the kinematic parameters of the mobile manipulator.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if the update was successful.
                 */
                virtual bool updateKinematics(const VectorXd& q_virtual,
                                              const VectorXd& q_mobile,
                                              const VectorXd& q_mani,
                                              const VectorXd& qdot_virtual,
                                              const VectorXd& qdot_mobile,
                                              const VectorXd& qdot_mani);
                /**
                 * @brief Update the dynamics of the mobile manipulator.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if the update was successful.
                 */                              
                virtual bool updateDynamics(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani,
                                            const VectorXd& qdot_virtual,
                                            const VectorXd& qdot_mobile,
                                            const VectorXd& qdot_mani);

                /**
                 * @brief Combine virtual, mobile, and manipulator joint vectors into a single joint vector.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Combined joint vector.
                 */                             
                VectorXd getJointVector(const VectorXd& q_virtual,
                                        const VectorXd& q_mobile,
                                        const VectorXd& q_mani);
                /**
                 * @brief Combine mobile and manipulator joint vectors into a single actuator joint vector.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Combined actuator joint vector.
                 */                         
                VectorXd getActuatorVector(const VectorXd& q_mobile,
                                           const VectorXd& q_mani);

                int mani_dof_;          // Manipulator degrees of freedom
                int mobi_dof_;          // Mobile base degrees of freedom
                int virtual_dof_{3};    // Virtual joint degrees of freedom
                int actuated_dof_;      // Actuated joint degrees of freedom
        
                JointIndex joint_idx_;          // Starting index for joints
                ActuatorIndex actuator_idx_;    // Starting index for actuators
                        
                // Selection Matrix for virtual joints (Actuated Joint Velocity -> Total Joint Velocity)
                MatrixXd S_;
                // MatrixXd Sdot_;
                
                VectorXd q_virtual_;        // Joint positions of the floating base.
                VectorXd q_mobile_;         // Wheel positions
                VectorXd q_mani_;           // Joint positions of the manipulator.
                VectorXd qdot_virtual_;     // Joint velocities of the floating base.
                VectorXd qdot_mobile_;      // Wheel velocities
                VectorXd qdot_mani_;        // Joint velocities of the manipulator.

                // Actuated Joint state
                VectorXd q_actuated_;        // Joint positions of the actuated joints.
                VectorXd qdot_actuated_;     // Joint velocities of the actuated joints.
                
                // ActuatedJoint space Dynamics
                MatrixXd M_actuated_;     // Mass matrix for the actuated joints.
                MatrixXd M_inv_actuated_; // Inversed Mass matrix for the actuated joints.
                VectorXd g_actuated_;     // Gravity vector for the actuated joints.
                VectorXd c_actuated_;     // Coriolis vector for the actuated joints.
                VectorXd NLE_actuated_;   // Nonlinear effects vector for the actuated joints.
        };
    } // namespace MobileManipulator
} // namespace drc
