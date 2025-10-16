#pragma once
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"
#include "dyros_robot_controller/manipulator/QP_IK.h"
#include "dyros_robot_controller/manipulator/QP_ID.h"
#include "dyros_robot_controller/mobile_manipulator/QP_IK.h"
#include "dyros_robot_controller/mobile_manipulator/QP_ID.h"

namespace drc
{
    namespace  MobileManipulator
    {
        /**
         * @brief Controller-side base class for mobile manipulator controller.
         *
         * This class consists of functions that compute control inputs for mobile manipulator and helpers that generate smooth trajectories.
         * Joint space functions compute control inputs of the manipulator to track desired joint positions, velocities, or accelerations.
         * Task space functions compute control inputs of the whole body to track desired position or velocity of a link.
        */
        class RobotController
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param dt         (double) Control loop time step in seconds.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>)
                 *                   Shared pointer to the RobotData class.
                */
                RobotController(const double& dt,
                                std::shared_ptr<MobileManipulator::RobotData> robot_data);
                
                /**
                 * @brief Set joint space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                */                
                virtual void setManipulatorJointGain(const VectorXd& Kp, 
                                                     const VectorXd& Kv);
                /**
                 * @brief Set joint space P gains for the robot.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                */                                     
                virtual void setManipulatorJointKpGain(const VectorXd& Kp);
                /**
                 * @brief Set joint space D gains for the robot.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                */ 
                virtual void setManipulatorJointKvGain(const VectorXd& Kv);
                /**
                 * @brief Set task space PD gains for the robot.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                */
                virtual void setTaskGain(const VectorXd& Kp, 
                                         const VectorXd& Kv);
                /**
                 * @brief Set task space P gains for the robot.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                */                         
                virtual void setTaskKpGain(const VectorXd& Kp);
                /**
                 * @brief Set task space D gains for the robot.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                */
                virtual void setTaskKvGain(const VectorXd& Kv);

                // ================================ Joint space Functions ================================                
                /**
                 * @brief Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_mani_init       (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_mani_init    (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segment.
                 * @param current_time      (double) Current time.
                 * @param init_time         (double) Start time of the segment.
                 * @param duration          (double) Time duration.
                 * @return (Eigen::VectorXd) Desired manipulator joint positions.
                */ 
                virtual VectorXd moveManipulatorJointPositionCubic(const VectorXd& q_mani_target,
                                                                   const VectorXd& qdot_mani_target,
                                                                   const VectorXd& q_mani_init,
                                                                   const VectorXd& qdot_mani_init,
                                                                   const double& current_time,
                                                                   const double& init_time,
                                                                   const double& duration);

                /**
                 * @brief Computes joint torques to achieve desired manipulator joint accelerations using equations of motion about manipulator.
                 * @param qddot_mani_target (Eigen::VectorXd) Desired joint accelerations.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                */                                                    
                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& qddot_mani_target);
                /**
                 * @brief Computes joint torques to achieve desired manipulator joint positions & velocities using PD control law.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */
                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& q_mani_target,
                                                                const VectorXd& qdot_mani_target);

                /**
                 * @brief Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration, then compute manipulator joint torques to follow the resulting trajectory.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_mani_init       (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_mani_init    (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segemnt.
                 * @param current_time      (double) Current time.
                 * @param init_time         (double) Start time of the segment.
                 * @param duration          (double) Time duration.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */                                                
                virtual VectorXd moveManipulatorJointTorqueCubic(const VectorXd& q_mani_target,
                                                                 const VectorXd& qdot_mani_target,
                                                                 const VectorXd& q_mani_init,
                                                                 const VectorXd& qdot_mani_init,
                                                                 const double& current_time,
                                                                 const double& init_time,
                                                                 const double& duration);

                // ================================ Task space Functions ================================
                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired velocity of a link by solving inverse kinematics QP.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name             (std::string) Name of the link.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile base velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                */                                                  
                virtual void QPIK(const VectorXd& xdot_target,
                                  const std::string& link_name,
                                  VectorXd& opt_qdot_mobile,
                                  VectorXd& opt_qdot_manipulator);

                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired position & velocity of a link by solving inverse kinematics QP.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name             (std::string) Name of the link.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile base velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                */                    
                virtual void QPIKStep(const Affine3d& x_target, 
                                      const VectorXd& xdot_target,
                                      const std::string& link_name,
                                      VectorXd& opt_qdot_mobile,
                                      VectorXd& opt_qdot_manipulator);

                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute velocities for mobile base and manipulator joints using QP to follow the resulting trajectory.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                (Eigen::Affine3d) Initial position of a link at the start of the segment.
                 * @param xdot_init             (Eigen::VectorXd) Initial velocity of a link at the start of the segment.
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration.
                 * @param link_name             (std::string) Name of the link.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile base velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                */                      
                virtual void QPIKCubic(const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name,
                                       VectorXd& opt_qdot_mobile,
                                       VectorXd& opt_qdot_manipulator);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired velocity of a link by solving inverse dynamics QP.
                 * @param xddot_target           (Eigen::VectorXd) Desired acceleration of a link.
                 * @param link_name              (std::string) Name of the link.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                */      
                virtual void QPID(const VectorXd& xddot_target,
                                  const std::string& link_name,
                                  VectorXd& opt_qddot_mobile,
                                  VectorXd& opt_torque_manipulator);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired position & velocity of a link by solving inverse dynamics QP.
                 * @param x_target               (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target            (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name              (std::string) Name of the link.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                */                  
                virtual void QPIDStep(const Affine3d& x_target, 
                                      const VectorXd& xdot_target,
                                      const std::string& link_name,
                                      VectorXd& opt_qddot_mobile,
                                      VectorXd& opt_torque_manipulator);

                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute mobile base accelerations and manipulator joint torques using QP to follow the resulting trajectory.
                 * @param x_target               (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target            (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                 (Eigen::Affine3d) Initial position of a link at the start of the segment.
                 * @param xdot_init              (Eigen::VectorXd) Initial velocity of a link at the start of the segment.
                 * @param current_time           (double) Current time.
                 * @param init_time              (double) Start time of the segment.
                 * @param duration               (double) Time duration.
                 * @param link_name              (std::string) Name of the link.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                */
                virtual void QPIDCubic(const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name,
                                       VectorXd& opt_qddot_mobile,
                                       VectorXd& opt_torque_manipulator);
   
                
            protected:
                double dt_;                                                 // Control time step in seconds
                int dof_;                                                   // Total degrees of freedom
                int mani_dof_;                                              // Manipulator DOF
                int mobi_dof_;                                              // Mobile robot DOF
                int actuator_dof_;                                          // Number of actuated joints
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data.

                // Task space gains
                VectorXd Kp_task_;
                VectorXd Kv_task_;

                // Joint space gains
                VectorXd Kp_mani_joint_;
                VectorXd Kv_mani_joint_;

                // QP solvers
                std::unique_ptr<MobileManipulator::QPIK> QP_moma_IK_;
                std::unique_ptr<MobileManipulator::QPID> QP_moma_ID_;
        };
    } // namespace MobileManipulator
} // namespace drc