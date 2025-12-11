#pragma once
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"
#include "dyros_robot_controller/mobile/robot_controller.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"
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
        class RobotController : public Mobile::RobotController
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param dt         (double) Control loop time step in seconds.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                */
                RobotController(const double& dt, std::shared_ptr<MobileManipulator::RobotData> robot_data);
                
                /**
                 * @brief Set joint space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as mani_dof.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as mani_dof.
                */                
                virtual void setManipulatorJointGain(const VectorXd& Kp, 
                                                     const VectorXd& Kv);
                /**
                 * @brief Set joint space P gains for the robot.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as mani_dof.
                */                                     
                virtual void setManipulatorJointKpGain(const VectorXd& Kp);
                /**
                 * @brief Set joint space D gains for the robot.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as mani_dof.
                */ 
                virtual void setManipulatorJointKvGain(const VectorXd& Kv);
                /**
                 * @brief Set task space PD gains for the robot per links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Proportional gains.
                 * @param link_Kv (std::map<std::string, Vector6d>) Derivative gains.
                */
                virtual void setTaskGain(const std::map<std::string, Vector6d>& link_Kp, 
                                         const std::map<std::string, Vector6d>& link_Kv);
                /**
                 * @brief Set task space P gains for the robot per links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Proportional gains.
                 */                         
                virtual void setTaskKpGain(const std::map<std::string, Vector6d>& link_Kp);
                /**
                 * @brief Set task space D gains for the robot per links.
                 * @param link_Kv (std::map<std::string, Vector6d>) Derivative gains.
                 */
                virtual void setTaskKvGain(const std::map<std::string, Vector6d>& link_Kv);      
                /**
                 * @brief Set the wight vector for  the cost terms of the QPIK
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as actuator_dof.
                 */
                // TODO: add document to notion
                void setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_damping);
                /**
                 * @brief Set the wight vector for  the cost terms of the QPID
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as actuator_dof.
                 */
                // TODO: add document to notion
                void setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_vel_damping, const VectorXd& w_acc_damping);

                // TODO: modify comments, add bindings and python func, add python comment, add to notion
                // ================================== Mobile Functions ===================================
                /**
                 * @brief Compute wheel velocities from desired base velocity using inverse kinematics.
                 *
                 * @param base_vel (Eigen::Vector3d) Desired base velocity [vx, vy, wz].
                 * @return (Eigen::VectorXd) Computed wheel velocities [rad/s], size = number of wheels.
                */
                virtual VectorXd computeMobileWheelVel(const Vector3d& base_vel);

                /**
                 * @brief Compute inverse kinematics Jacobian of base mobile (maps base velocity to wheel velocity).
                 *
                 * @return (Eigen::MatrixXd) Jacobian matrix of size [wheel_num x 3].
                */
                virtual MatrixXd computeMobileIKJacobian();

                /**
                 * @brief Generate base velocity command with optional constraints (e.g., saturation).
                 *
                 * @param desired_base_vel (Eigen::Vector3d) Desired base velocity [vx, vy, wz].
                 * @return (Eigen::VectorXd) Wheel velocity command (possibly saturated), size = number of wheels.
                */
                virtual VectorXd MobileVelocityCommand(const Vector3d& desired_base_vel);

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
                 * @param use_mass          (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                */                             
                // TODO: add document to notion that add use_mass                       
                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& qddot_mani_target, const bool use_mass = true);
                /**
                 * @brief Computes joint torques to achieve desired manipulator joint positions & velocities using PD control law.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities.
                 * @param use_mass          (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */
                // TODO: add document to notion that add use_mass
                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& q_mani_target,
                                                                const VectorXd& qdot_mani_target,
                                                                const bool use_mass = true);

                /**
                 * @brief Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration, then compute manipulator joint torques to follow the resulting trajectory.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_mani_init       (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_mani_init    (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segemnt.
                 * @param current_time      (double) Current time.
                 * @param init_time         (double) Start time of the segment.
                 * @param duration          (double) Time duration.
                 * @param use_mass    (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */          
                // TODO: add document to notion that add use_mass                                      
                virtual VectorXd moveManipulatorJointTorqueCubic(const VectorXd& q_mani_target,
                                                                 const VectorXd& qdot_mani_target,
                                                                 const VectorXd& q_mani_init,
                                                                 const VectorXd& qdot_mani_init,
                                                                 const double& current_time,
                                                                 const double& init_time,
                                                                 const double& duration,
                                                                 const bool use_mass = true);

                // ================================ Task space Functions ================================
                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */                                                  
                virtual void QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  VectorXd& opt_qdot_mobile,
                                  VectorXd& opt_qdot_manipulator,
                                  const bool time_verbose=false);

                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */                    
                virtual void QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      VectorXd& opt_qdot_mobile,
                                      VectorXd& opt_qdot_manipulator,
                                      const bool time_verbose=false);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute velocities for mobile base and manipulator joints using QP to follow the resulting trajectory.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */                      
                virtual void QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       VectorXd& opt_qdot_mobile,
                                       VectorXd& opt_qdot_manipulator,
                                       const bool time_verbose=false);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */      
                virtual void QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  VectorXd& opt_qddot_mobile,
                                  VectorXd& opt_torque_manipulator,
                                  const bool time_verbose=false);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */                  
                virtual void QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      VectorXd& opt_qddot_mobile,
                                      VectorXd& opt_torque_manipulator,
                                      const bool time_verbose=false);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using QP to follow the resulting trajectory.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param init_time              (double) Start time of the segment.
                 * @param duration               (double) Time duration.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                */
                virtual void QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       VectorXd& opt_qddot_mobile,
                                       VectorXd& opt_torque_manipulator,
                                       const bool time_verbose=false);

   
                
            protected:
                double dt_;                                                 // Control time step in seconds
                int dof_;                                                   // Total degrees of freedom
                int mani_dof_;                                              // Manipulator DOF
                int mobi_dof_;                                              // Mobile robot DOF
                int actuator_dof_;                                          // Number of actuated joints
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data.

                // Task space gains per links
                std::map<std::string, Vector6d> link_Kp_task_;
                std::map<std::string, Vector6d> link_Kv_task_;

                // Joint space gains
                VectorXd Kp_mani_joint_;
                VectorXd Kv_mani_joint_;

                // QP solvers
                std::unique_ptr<MobileManipulator::QPIK> QP_moma_IK_;
                std::unique_ptr<MobileManipulator::QPID> QP_moma_ID_;

                virtual void QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                  VectorXd& opt_qdot_mobile,
                                  VectorXd& opt_qdot_manipulator,
                                  const bool time_verbose=false);

                virtual void QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                  VectorXd& opt_qddot_mobile,
                                  VectorXd& opt_torque_manipulator,
                                  const bool time_verbose=false);
        };
    } // namespace MobileManipulator
} // namespace drc