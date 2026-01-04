#pragma once
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/QP_IK.h"
#include "dyros_robot_controller/manipulator/QP_ID.h"

namespace drc
{
    /**
     * @brief Controller-side base class for single manipulator controller.
     * 
     * This class consists of functions that compute manipulator control inputs and helpers that generate smooth trajectories.
     * Joint space functions compute control inputs to track desired joint positions, velocities, or accelerations.
     * Task space functions compute control inputs to track desired position, velocity, or acceleration of a link.
     */
    namespace Manipulator
    {
        class RobotController
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param dt         (double) Control loop time step in seconds.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 */
                RobotController(const double& dt,  std::shared_ptr<Manipulator::RobotData> robot_data);
                
                /**
                 * @brief Set joint space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as dof.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as dof.
                 */                
                virtual void setJointGain(const VectorXd& Kp, 
                                          const VectorXd& Kv);
                /**
                 * @brief Set joint space P gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as dof.
                 */                            
                virtual void setJointKpGain(const VectorXd& Kp);
                /**
                 * @brief Set joint space D gains for the manipulator.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as dof.
                 */ 
                virtual void setJointKvGain(const VectorXd& Kv);
                /**
                 * @brief Set task space PD gains for the manipulator per links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Proportional gains.
                 * @param link_Kv (std::map<std::string, Vector6d>) Derivative gains.
                 */
                virtual void setTaskGain(const std::map<std::string, Vector6d>& link_Kp, 
                                         const std::map<std::string, Vector6d>& link_Kv);
                /**
                 * @brief Set task space P gains for the manipulator per links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Proportional gains.
                 */                          
                virtual void setTaskKpGain(const std::map<std::string, Vector6d>& link_Kp);
                /**
                 * @brief Set task space D gains for the manipulator per links.
                 * @param link_Kv (std::map<std::string, Vector6d>) Derivative gains.
                 */ 
                virtual void setTaskKvGain(const std::map<std::string, Vector6d>& link_Kv);      
                
                /**
                 * @brief Set the wight vector for  the cost terms of the QPIK
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_damping);
                /**
                 * @brief Set the wight vector for  the cost terms of the QPID
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_vel_damping, const VectorXd& w_acc_damping);
                
                
                // ================================ Joint space Functions ================================
                /**
                 * @brief Perform cubic interpolation between the initial and desired joint configurations over the given duration.
                 * @param q_target      (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_target   (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_init        (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_init     (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segment.
                 * @param current_time  (double) Current time.
                 * @param init_time     (double) Start time of the segment.
                 * @param duration      (double) Time duration
                 * @return (Eigen::VectorXd) Desired joint positions.
                 */
                virtual VectorXd moveJointPositionCubic(const VectorXd& q_target,
                                                        const VectorXd& qdot_target,
                                                        const VectorXd& q_init,
                                                        const VectorXd& qdot_init,
                                                        const double& current_time,
                                                        const double& init_time,
                                                        const double& duration);
                /**
                 * @brief Perform cubic interpolation between the initial and desired joint configurations over the given duration.
                 * @param q_target      (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_target   (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_init        (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_init     (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segment.
                 * @param current_time  (double) Current time.
                 * @param init_time     (double) Start time of the segment.
                 * @param duration      (double) Time duration
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */                                        
                virtual VectorXd moveJointVelocityCubic(const VectorXd& q_target,
                                                        const VectorXd& qdot_target,
                                                        const VectorXd& q_init,
                                                        const VectorXd& qdot_init,
                                                        const double& current_time,
                                                        const double& init_time,
                                                        const double& duration);
                /**
                 * @brief Computes joint torques to achieve desired joint accelerations using dynamics.
                 * @param qddot_target (Eigen::VectorXd) Desired manipulator joint accelerations.
                 * @param use_mass     (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */
                // TODO: add document to notion that add use_mass
                virtual VectorXd moveJointTorqueStep(const VectorXd& qddot_target, const bool use_mass = true);
                /**
                 * @brief Computes joint torques to achieve desired joint positions & velocities using PD control law.
                 * @param q_target    (Eigen::VectorXd) Desired manipulator joint positions.
                 * @param qdot_target (Eigen::VectorXd) Desired manipulator joint velocities.
                 * @param use_mass    (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */
                // TODO: add document to notion that add use_mass
                virtual VectorXd moveJointTorqueStep(const VectorXd& q_target,
                                                     const VectorXd& qdot_target,
                                                     const bool use_mass = true);
                /**
                 * @brief Perform cubic interpolation between the initial and desired joint configurations over the given duration, then compute joint torques to follow the resulting trajectory.
                 * @param q_target      (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_target   (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_init        (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_init     (Eigen::VectorXd) Initial manipulator joint velocites at the start of the segment.
                 * @param current_time  (double) Current time.
                 * @param init_time     (double) Start time of the segment.
                 * @param duration      (double) Time duration
                 * @param use_mass      (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                          
                // TODO: add document to notion that add use_mass           
                virtual VectorXd moveJointTorqueCubic(const VectorXd& q_target,
                                                      const VectorXd& qdot_target,
                                                      const VectorXd& q_init,
                                                      const VectorXd& qdot_init,
                                                      const double& current_time,
                                                      const double& init_time,
                                                      const double& duration,
                                                      const bool use_mass = true);
                                                
                // ================================ Task space Functions ================================
                /**
                 * @brief Computes joint velocities to achieve desired velocity of a link by using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param null_qdot      (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      const VectorXd& null_qdot);
                /**
                 * @brief Computes joint velocities to achieve desired velocity of a link by using closed-loop inverse kinematics.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param null_qdot      (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIK(const std::map<std::string, TaskSpaceData>& link_task_data);
                /**
                 * @brief Computes joint velocity to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param null_qdot      (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                          const VectorXd& null_qdot);
                /**
                 * @brief Computes joint velocity to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration, then compute joint velocities with null_qdot to follow the resulting trajectory.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time   (double) Current time.
                 * @param init_time      (double) Start time of the segment.
                 * @param duration       (double) Time duration
                 * @param null_qdot      (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const VectorXd& null_qdot);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time   (double) Current time.
                 * @param init_time      (double) Start time of the segment.
                 * @param duration       (double) Time duration
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration);
                /**
                 * @brief Computes joint torque to achieve desired acceleration (xddot_desired) of a link using operational space control, projecting null_torque into null space to exploit redundancy.
                 * @param link_task_data            (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param null_torque               (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                           
                virtual VectorXd OSF(const std::map<std::string, TaskSpaceData>& link_task_data, 
                                     const VectorXd& null_torque);
                /**
                 * @brief Computes joint torque to achieve desired acceleration (xddot_desired) of a link using operational space control.
                 * @param link_task_data            (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */ 
                virtual VectorXd OSF(const std::map<std::string, TaskSpaceData>& link_task_data);
                /**
                 * @brief Computes joint torque to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space control, projecting null_torque into null space to exploit redundancy.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Desired position of a link; it must include (x_desired, xdot_desired).
                 * @param null_torque           (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                     
                virtual VectorXd OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                         const VectorXd& null_torque);
                /**
                 * @brief Computes joint torque to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space control.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Desired position of a link; it must include (x_desired, xdot_desired).
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                     
                virtual VectorXd OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration, then compute joint torques with null_torque to follow the resulting trajectory.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @param null_torque           (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                         
                virtual VectorXd OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration,
                                          const VectorXd& null_torque);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) and velocity (xdot_desired) over the given duration.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */       
                virtual VectorXd OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration);
                /**
                 * @brief Computes joint velocities to achieve desired velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data  (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */                          
                virtual VectorXd QPIK(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose=false);
                /**
                 * @brief Computes joint velocities to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint velocities.
                */                      
                virtual VectorXd QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose=false);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute joint velocities using QP to follow the resulting trajectory.
                 * @param link_task_data       (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time         (double) Current time.
                 * @param init_time            (double) Start time of the segment.
                 * @param duration             (double) Time duration
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint velocities.
                */                           
                virtual VectorXd QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const bool time_verbose=false);
                /**
                 * @brief Computes joint torques to achieve desired acceleration (xddot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                            
                virtual VectorXd QPID(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose=false);
                /**
                 * @brief Computes joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                        
                virtual VectorXd QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose=false);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute joint torques using QP to follow the resulting trajectory.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param init_time              (double) Start time of the segment.
                 * @param duration               (double) Time duration
                 * @param time_verbose  (bool) If true, print the computation time for QP. 
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                             
                virtual VectorXd QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const bool time_verbose=false);

            protected:
                double dt_;                                          // Control time step in seconds.
                int dof_;                                            // Total degrees of freedom.
                std::shared_ptr<Manipulator::RobotData> robot_data_; // Shared pointer to the robot data class.

                // Task space gains
                std::map<std::string, Vector6d> link_Kp_task_;
                std::map<std::string, Vector6d> link_Kv_task_;

                // Joint space gains
                VectorXd Kp_joint_;
                VectorXd Kv_joint_;

                // QP solvers
                std::unique_ptr<Manipulator::QPIK> QP_mani_IK_;
                std::unique_ptr<Manipulator::QPID> QP_mani_ID_;

                virtual VectorXd CLIK(const std::map<std::string, Vector6d>& link_xdot_target, const VectorXd& null_qdot);
                virtual VectorXd OSF(const std::map<std::string, Vector6d>& link_xddot_target, const VectorXd& null_torque);
                virtual VectorXd QPIK(const std::map<std::string, Vector6d>& link_xdot_target, const bool time_verbose=false);
                virtual VectorXd QPID(const std::map<std::string, Vector6d>& link_xddot_target, const bool time_verbose=false);
        };
    } // namespace Manipulator
} // namespace drc