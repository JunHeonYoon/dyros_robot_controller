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
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData> )
                 *                   Shared pointer to the RobotData class.
                 */
                RobotController(const double& dt,
                                std::shared_ptr<Manipulator::RobotData> robot_data);
                
                /**
                 * @brief Set joint space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                 */                
                virtual void setJointGain(const VectorXd& Kp, 
                                          const VectorXd& Kv);
                /**
                 * @brief Set joint space P gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 */                            
                virtual void setJointKpGain(const VectorXd& Kp);
                /**
                 * @brief Set joint space D gains for the manipulator.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                 */ 
                virtual void setJointKvGain(const VectorXd& Kv);
                /**
                 * @brief Set task space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                 */
                virtual void setTaskGain(const VectorXd& Kp, 
                                         const VectorXd& Kv);
                /**
                 * @brief Set task space P gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains.
                 */                          
                virtual void setTaskKpGain(const VectorXd& Kp);
                /**
                 * @brief Set task space D gains for the manipulator.
                 * @param Kv (Eigen::VectorXd) Derivative gains.
                 */ 
                virtual void setTaskKvGain(const VectorXd& Kv);
                
                /**
                 * @brief Set the wight vector for  the cost terms of the QPIK
                 * @param w_tracking (Eigen::VectorXd) Weight for task velocity tracking; its size must be 6.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setQPIKGain(const VectorXd& w_tracking, const VectorXd& w_damping);
                /**
                 * @brief Set the wight vector for  the cost terms of the QPID
                 * @param w_tracking    (Eigen::VectorXd) Weight for task acceleration tracking; its size must be 6.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setQPIDGain(const VectorXd& w_tracking, const VectorXd& w_vel_damping, const VectorXd& w_acc_damping);
                
                
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
                 * @brief Computes joint velocity to achieve desired position & velocity of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param null_qdot             (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const VectorXd& null_qdot,
                                          const std::string& link_name);
                /**
                 * @brief Computes joint velocity to achieve desired position & velocity of a link using closed-loop inverse kinematics.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */                          
                virtual VectorXd CLIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);
                
                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint velocities with null_qdot to follow the resulting trajectory.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                (Eigen::Affine3d) Initial position of a link at the start of the segment.
                 * @param xdot_init             (Eigen::VectorXd) Initial velocity of a link at the start of the segment.
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @param null_qdot             (Eigen::VectorXd) Desired joint velocity to be projected on null space.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */
                virtual VectorXd CLIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const VectorXd& null_qdot,
                                           const std::string& link_name);
                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint velocities without null_qdot to follow the resulting trajectory.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                (Eigen::Affine3d) Initial position of a link at the start of the segment.
                 * @param xdot_init             (Eigen::VectorXd) Initial velocity of a link at the start of the segment.
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */                           
                virtual VectorXd CLIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);

                /**
                 * @brief Computes joint torque to achieve desired acceleration of a link using operational space control, projecting null_torque into null space to exploit redundancy.
                 * @param xddot_target              (Eigen::VectorXd) Desired acceleration of a link.
                 * @param null_torque               (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @param link_name                 (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                           
                virtual VectorXd OSF(const VectorXd& xddot_target, 
                                     const VectorXd& null_torque,
                                     const std::string& link_name);
                /**
                 * @brief Computes joint torque to achieve desired acceleration of a link using operational space control.
                 * @param xddot_target              (Eigen::VectorXd) Desired acceleration of a link.
                 * @param link_name                 (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                     
                virtual VectorXd OSF(const VectorXd& xddot_target,
                                     const std::string& link_name);
                
                /**
                 * @brief Computes joint torque to achieve desired position & velocity of a link using operational space control, projecting null_torque into null space to exploit redundancy.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param null_torque           (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                     
                virtual VectorXd OSFStep(const Affine3d& x_target, 
                                         const VectorXd& xdot_target,
                                         const VectorXd& null_torque,
                                         const std::string& link_name);
                /**
                 * @brief Computes joint torque to achieve desired position & velocity of a link using operational space control.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                         
                virtual VectorXd OSFStep(const Affine3d& x_target, 
                                         const VectorXd& xdot_target,
                                         const std::string& link_name);

                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint torques with null_torque to follow the resulting trajectory.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                (Eigen::Affine3d) Initial position of a link at init_time.
                 * @param xdot_init             (Eigen::VectorXd) Initial velocity of a link at init_time.
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @param null_torque           (Eigen::VectorXd) Desired joint torque to be projected on null space.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                         
                virtual VectorXd OSFCubic(const Affine3d& x_target,
                                          const VectorXd& xdot_target,
                                          const Affine3d& x_init,
                                          const VectorXd& xdot_init,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration,
                                          const VectorXd& null_torque,
                                          const std::string& link_name);
                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose and velocity over the given duration, then compute joint torques without null_torque to follow the resulting trajectory.
                 * @param x_target              (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                (Eigen::Affine3d) Initial position of a link at init_time.
                 * @param xdot_init             (Eigen::VectorXd) Initial velocity of a link at init_time.
                 * @param current_time          (double) Current time.
                 * @param init_time             (double) Start time of the segment.
                 * @param duration              (double) Time duration
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                 */                          
                virtual VectorXd OSFCubic(const Affine3d& x_target,
                                          const VectorXd& xdot_target,
                                          const Affine3d& x_init,
                                          const VectorXd& xdot_init,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration,
                                          const std::string& link_name);

                /**
                 * @brief Computes joint velocities to achieve desired velocity of a link by solving inverse kinematics QP.
                 * @param xdot_target           (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name             (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                 */                          
                virtual VectorXd QPIK(const VectorXd& xdot_target,
                                      const std::string& link_name);

                /**
                 * @brief Computes joint velocities to achieve desired position & velocity of a link by solving inverse kinematics QP.
                 * @param x_target             (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target          (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name            (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                */                      
                virtual VectorXd QPIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);

                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute joint velocities using QP to follow the resulting trajectory.
                 * @param x_target             (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target          (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init               (Eigen::Affine3d) Initial position of a link at init_time.
                 * @param xdot_init            (Eigen::VectorXd) Initial velocity of a link at init_time.
                 * @param current_time         (double) Current time.
                 * @param init_time            (double) Start time of the segment.
                 * @param duration             (double) Time duration
                 * @param link_name            (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint velocities.
                */                           
                virtual VectorXd QPIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);

                /**
                 * @brief Computes joint torques to achieve desired acceleration of a link by solving inverse dynamics QP.
                 * @param xddot_target           (Eigen::VectorXd) Desired acceleration of a link.
                 * @param link_name              (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                            
                virtual VectorXd QPID(const VectorXd& xddot_target,
                                      const std::string& link_name);

                /**
                 * @brief Computes joint torques to achieve desired position & velocity of a link by solving inverse dynamics QP.
                 * @param x_target               (Eigen::Affine3d) Desired position of a link.
                 * @param xdot_target            (Eigen::VectorXd) Desired velocity of a link.
                 * @param link_name              (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                        
                virtual VectorXd QPIDStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);

                /**
                 * @brief Perform cubic interpolation between the initial and desired link pose & velocity over the given duration, then compute joint torques using QP to follow the resulting trajectory.
                 * @param x_target               (Eigen::Affine3d) Desired position of a link at the end of the segment.
                 * @param xdot_target            (Eigen::VectorXd) Desired velocity of a link at the end of the segment.
                 * @param x_init                 (Eigen::Affine3d) Initial position of a link at the start of the segment.
                 * @param xdot_init              (Eigen::VectorXd) Initial velocity of a link at the start of the segment.
                 * @param current_time           (double) Current time.
                 * @param init_time              (double) Start time of the segment.
                 * @param duration               (double) Time duration
                 * @param link_name              (std::string) Name of the link.
                 * @return (Eigen::VectorXd) Desired joint torques.
                */                             
                virtual VectorXd QPIDCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);


            protected:
                double dt_;                                             // Control time step in seconds.
                int dof_;                                               // Total degrees of freedom.
                std::shared_ptr<Manipulator::RobotData> robot_data_;    // Shared pointer to the robot data class.

                // Task space gains
                VectorXd Kp_task_;
                VectorXd Kv_task_;

                // Joint space gains
                VectorXd Kp_joint_;
                VectorXd Kv_joint_;

                // QP solvers
                std::unique_ptr<Manipulator::QPIK> QP_mani_IK_;
                std::unique_ptr<Manipulator::QPID> QP_mani_ID_;

        };
    } // namespace Manipulator
} // namespace drc