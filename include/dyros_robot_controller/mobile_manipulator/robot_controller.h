#pragma once
#include <string>
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
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                */
                RobotController(std::shared_ptr<MobileManipulator::RobotData> robot_data);
                
                /**
                 * @brief Set joint space PD gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as mani_dof.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as mani_dof.
                */                
                virtual void setManipulatorJointGain(const Eigen::Ref<const VectorXd>& Kp, 
                                                     const Eigen::Ref<const VectorXd>& Kv);
                /**
                 * @brief Set joint space P gains for the manipulator.
                 * @param Kp (Eigen::VectorXd) Proportional gains; its size must same as mani_dof.
                */
                virtual void setManipulatorJointKpGain(const Eigen::Ref<const VectorXd>& Kp);
                /**
                 * @brief Set joint space D gains for the manipulator.
                 * @param Kv (Eigen::VectorXd) Derivative gains; its size must same as mani_dof.
                */
                virtual void setManipulatorJointKvGain(const Eigen::Ref<const VectorXd>& Kv);

                /**
                 * @brief Set IK task-space proportional gains for specified links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Link-name to 6D task gain map.
                 * Invalid link names are ignored with a warning.
                 */
                virtual void setIKGain(const std::map<std::string, Vector6d>& link_Kp);

                /**
                 * @brief Set the same IK task-space proportional gain for all link frames.
                 * @param Kp (Vector6d) 6D task-space proportional gain applied to every link.
                 */
                virtual void setIKGain(const Vector6d& Kp);

                /**
                 * @brief Set ID task-space proportional/derivative gains for specified links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Link-name to 6D proportional gain map.
                 * @param link_Kv (std::map<std::string, Vector6d>) Link-name to 6D derivative gain map.
                 */
                virtual void setIDGain(const std::map<std::string, Vector6d>& link_Kp,
                                       const std::map<std::string, Vector6d>& link_Kv);

                /**
                 * @brief Set the same ID task-space proportional/derivative gains for all link frames.
                 * @param Kp (Vector6d) 6D proportional gain applied to every link.
                 * @param Kv (Vector6d) 6D derivative gain applied to every link.
                 */
                virtual void setIDGain(const Vector6d& Kp,
                                       const Vector6d& Kv);

                /**
                 * @brief Set ID task-space proportional gains for specified links.
                 * @param link_Kp (std::map<std::string, Vector6d>) Link-name to 6D proportional gain map.
                 */
                virtual void setIDKpGain(const std::map<std::string, Vector6d>& link_Kp);

                /**
                 * @brief Set the same ID task-space proportional gain for all link frames.
                 * @param Kp (Vector6d) 6D proportional gain applied to every link.
                 */
                virtual void setIDKpGain(const Vector6d& Kp);

                /**
                 * @brief Set ID task-space derivative gains for specified links.
                 * @param link_Kv (std::map<std::string, Vector6d>) Link-name to 6D derivative gain map.
                 */
                virtual void setIDKvGain(const std::map<std::string, Vector6d>& link_Kv);

                /**
                 * @brief Set the same ID task-space derivative gain for all link frames.
                 * @param Kv (Vector6d) 6D derivative gain applied to every link.
                 */
                virtual void setIDKvGain(const Vector6d& Kv);

                /**
                 * @brief Set QPIK task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task velocity tracking for every link.
                 */
                void setQPIKTrackingGain(const Vector6d& w_tracking);

                /**
                 * @brief Set QPIK task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 */
                void setQPIKTrackingGain(const std::map<std::string, Vector6d>& link_w_tracking);

                /**
                 * @brief Set QPIK manipulator joint velocity damping weights only.
                 * @param w_mani_vel_damping (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 */
                void setQPIKManiJointVelGain(const Eigen::Ref<const VectorXd>& w_mani_vel_damping);

                /**
                 * @brief Set QPIK manipulator joint acceleration damping weights only.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 */
                void setQPIKManiJointAccGain(const Eigen::Ref<const VectorXd>& w_mani_acc_damping);

                /**
                 * @brief Set QPIK mobile base velocity damping weights only.
                 * @param w_base_vel_damping (Eigen::Vector3d) Weight for mobile base velocity damping.
                 */
                void setQPIKBaseVelGain(const Eigen::Vector3d& w_base_vel_damping);

                /**
                 * @brief Set QPIK mobile base acceleration damping weights only.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIKBaseAccGain(const Eigen::Vector3d& w_base_acc_damping);

                /**
                 * @brief Set the weight vector for the cost terms of the QPIK.
                 * @param w_tracking (Vector6d) Weight for task velocity tracking for every link.
                 * @param w_mani_vel_damping  (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 * @param w_mani_acc_damping  (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 * @param w_base_vel_damping  (Eigen::Vector3d) Weight for mobile base velocity damping.
                 * @param w_base_acc_damping  (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIKGain(const Vector6d& w_tracking,
                                 const Eigen::Ref<const VectorXd>& w_mani_vel_damping,
                                 const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                 const Eigen::Vector3d& w_base_vel_damping,
                                 const Eigen::Vector3d& w_base_acc_damping);

                /**
                 * @brief Set the weight vector for the cost terms of the QPIK.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 * @param w_mani_vel_damping  (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 * @param w_mani_acc_damping  (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 * @param w_base_vel_damping  (Eigen::Vector3d) Weight for mobile base velocity damping.
                 * @param w_base_acc_damping  (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking,
                                 const Eigen::Ref<const VectorXd>& w_mani_vel_damping,
                                 const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                 const Eigen::Vector3d& w_base_vel_damping,
                                 const Eigen::Vector3d& w_base_acc_damping);
                                 

                /**
                 * @brief Set QPID task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task acceleration tracking for every link.
                 */
                void setQPIDTrackingGain(const Vector6d& w_tracking);

                /**
                 * @brief Set QPID task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task acceleration tracking per links.
                 */
                void setQPIDTrackingGain(const std::map<std::string, Vector6d>& link_w_tracking);

                /**
                 * @brief Set QPID manipulator joint velocity damping weights only.
                 * @param w_mani_vel_damping (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 */
                void setQPIDManiJointVelGain(const Eigen::Ref<const VectorXd>& w_mani_vel_damping);

                /**
                 * @brief Set QPID manipulator joint acceleration damping weights only.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 */
                void setQPIDManiJointAccGain(const Eigen::Ref<const VectorXd>& w_mani_acc_damping);

                /**
                 * @brief Set QPID mobile base velocity damping weights only.
                 * @param w_base_vel_damping (Eigen::Vector3d) Weight for mobile base velocity damping.
                 */
                void setQPIDBaseVelGain(const Eigen::Vector3d& w_base_vel_damping);

                /**
                 * @brief Set QPID mobile base acceleration damping weights only.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIDBaseAccGain(const Eigen::Vector3d& w_base_acc_damping);
               
                /**
                 * @brief Set the weight vector for the cost terms of the QPID.
                 * @param w_tracking (Vector6d) Weight for task acceleration tracking for every link.
                 * @param w_mani_vel_damping (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 * @param w_base_vel_damping (Eigen::Vector3d) Weight for mobile base velocity damping.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIDGain(const Vector6d& w_tracking,
                                 const Eigen::Ref<const VectorXd>& w_mani_vel_damping,
                                 const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                 const Eigen::Vector3d& w_base_vel_damping,
                                 const Eigen::Vector3d& w_base_acc_damping);

                /**
                 * @brief Set the weight vector for the cost terms of the QPID.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task acceleration tracking per links.
                 * @param w_mani_vel_damping (Eigen::VectorXd) Weight for manipulator joint velocity damping; its size must same as mani_dof.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 * @param w_base_vel_damping (Eigen::Vector3d) Weight for mobile base velocity damping.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking,
                                 const Eigen::Ref<const VectorXd>& w_mani_vel_damping, 
                                 const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                 const Eigen::Vector3d& w_base_vel_damping,
                                 const Eigen::Vector3d& w_base_acc_damping);
                

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
                virtual VectorXd moveManipulatorJointPositionCubic(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                   const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                   const Eigen::Ref<const VectorXd>& q_mani_init,
                                                                   const Eigen::Ref<const VectorXd>& qdot_mani_init,
                                                                   const double& current_time,
                                                                   const double& init_time,
                                                                   const double& duration);
                /**
                 * @brief Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration.
                 * @param q_target      (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_target   (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_init        (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_init     (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segment.
                 * @param current_time  (double) Current time.
                 * @param init_time     (double) Start time of the segment.
                 * @param duration      (double) Time duration
                 * @return (Eigen::VectorXd) Desired manipulator joint velocities.
                 */                                        
                virtual VectorXd moveManipulatorJointVelocityCubic(const Eigen::Ref<const VectorXd>& q_target,
                                                                   const Eigen::Ref<const VectorXd>& qdot_target,
                                                                   const Eigen::Ref<const VectorXd>& q_init,
                                                                   const Eigen::Ref<const VectorXd>& qdot_init,
                                                                   const double& current_time,
                                                                   const double& init_time,
                                                                   const double& duration);
                /**
                 * @brief Computes joint torques to achieve desired manipulator joint accelerations using equations of motion about manipulator.
                 * @param qddot_mani_target (Eigen::VectorXd) Desired joint accelerations.
                 * @param use_mass          (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                */                             
                virtual VectorXd moveManipulatorJointTorqueStep(const Eigen::Ref<const VectorXd>& qddot_mani_target, const bool use_mass = true);
                /**
                 * @brief Computes joint torques to achieve desired manipulator joint positions & velocities using PD control law.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities.
                 * @param use_mass          (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */
                virtual VectorXd moveManipulatorJointTorqueStep(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                const bool use_mass = true);

                /**
                 * @brief Perform cubic interpolation between the initial and desired manipulator joint configurations over the given duration, then compute manipulator joint torques to follow the resulting trajectory.
                 * @param q_mani_target     (Eigen::VectorXd) Desired manipulator joint positions at the end of the segment.
                 * @param qdot_mani_target  (Eigen::VectorXd) Desired manipulator joint velocities at the end of the segment.
                 * @param q_mani_init       (Eigen::VectorXd) Initial manipulator joint positions at the start of the segment.
                 * @param qdot_mani_init    (Eigen::VectorXd) Initial manipulator joint velocities at the start of the segment.
                 * @param current_time      (double) Current time.
                 * @param init_time         (double) Start time of the segment.
                 * @param duration          (double) Time duration.
                 * @param use_mass    (bool) Whether use mass matrix.
                 * @return (Eigen::VectorXd) Desired manipulator joint torques.
                 */          
                virtual VectorXd moveManipulatorJointTorqueCubic(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                 const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                 const Eigen::Ref<const VectorXd>& q_mani_init,
                                                                 const Eigen::Ref<const VectorXd>& qdot_mani_init,
                                                                 const double& current_time,
                                                                 const double& init_time,
                                                                 const double& duration,
                                                                 const bool use_mass = true);

                // ================================ Task space Functions ================================
                /**
                 * @brief Computes mobile base and manipulator joint velocities to achieve desired velocity of a link using closed-loop inverse kinematics, projecting null_qdot into null space to exploit redundancy.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param null_qdot             (Eigen::VectorXd) Desired actuated joint velocity to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  const Eigen::Ref<const VectorXd>& null_qdot);
                /**
                 * @brief Computes mobile base and manipulator joint velocities to achieve desired velocity of a link using closed-loop inverse kinematics.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator);

                /**
                 * @brief Computes mobile base and manipulator joint velocities to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics, projecting null_qdot into null space.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param null_qdot             (Eigen::VectorXd) Desired actuated joint velocity to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                      const Eigen::Ref<const VectorXd>& null_qdot);
                /**
                 * @brief Computes mobile base and manipulator joint velocities to achieve desired position (x_desired) & velocity (xdot_desired) of a link using closed-loop inverse kinematics.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base and manipulator joint velocities using CLIK with null_qdot.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param duration              (double) Time duration.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param null_qdot             (Eigen::VectorXd) Desired actuated joint velocity to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       const Eigen::Ref<const VectorXd>& null_qdot);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base and manipulator joint velocities using CLIK.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param duration              (double) Time duration.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link using operational space formulation, projecting null_torque into null space.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param null_torque            (Eigen::VectorXd) Desired actuated joint torque to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                 Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                 Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                 const Eigen::Ref<const VectorXd>& null_torque);
                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link using operational space formulation.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                 Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                 Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space formulation, projecting null_torque into null space.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param null_torque            (Eigen::VectorXd) Desired actuated joint torque to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                     Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                     Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                     const Eigen::Ref<const VectorXd>& null_torque);
                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link using operational space formulation.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                     Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                     Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using OSF with null_torque.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param duration               (double) Time duration.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param null_torque            (Eigen::VectorXd) Desired actuated joint torque to be projected on null space; size must be (mobi_dof + mani_dof).
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      const double& current_time,
                                      const double& duration,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                      const Eigen::Ref<const VectorXd>& null_torque);
                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using OSF.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param duration               (double) Time duration.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @return (bool) True if sizes are valid and computation succeeded.
                 */
                virtual bool OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      const double& current_time,
                                      const double& duration,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator);

                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */                                                  
                virtual bool QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPIK.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xdot_desired.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  const bool time_verbose=false);

                /**
                 * @brief Computes velocities for mobile base and manipulator joints to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse kinematics QP.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */                    
                virtual bool QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                      std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPIKStep.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                      const bool time_verbose=false);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute velocities for mobile base and manipulator joints using QP to follow the resulting trajectory.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param duration              (double) Time duration.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */                      
                virtual bool QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPIKCubic.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time          (double) Current time.
                 * @param duration              (double) Time duration.
                 * @param opt_qdot_mobile       (Eigen::VectorXd) Output optimal mobile wheel velocities.
                 * @param opt_qdot_manipulator  (Eigen::VectorXd) Output optimal manipulator joint velocities.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       const bool time_verbose=false);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired acceleration (xddot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */      
                virtual bool QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPID.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include xddot_desired.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile wheel accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  const bool time_verbose=false);

                /**
                 * @brief Computes mobile base accelerations and manipulator joint torques to achieve desired position (x_desired) & velocity (xdot_desired) of a link by solving inverse dynamics QP.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */                  
                virtual bool QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                      std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPIDStep.
                 * @param link_task_data         (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_desired, xdot_desired).
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                      const bool time_verbose=false);

                /**
                 * @brief Perform cubic interpolation between the initial (x_init, xdot_init) and desired link pose (x_desired) & velocity (xdot_desired) over the given duration, then compute mobile base accelerations and manipulator joint torques using QP to follow the resulting trajectory.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param duration               (double) Time duration.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (std::string&) Output formatted computation time information for QP.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                       std::string& time_verbose);
                /**
                 * @brief Compatibility overload of QPIDCubic.
                 * @param link_task_data        (std::map<std::string, TaskSpaceData>) Task space data per links; it must include (x_init, xdot_init, x_desired, xdot_desired).
                 * @param current_time           (double) Current time.
                 * @param duration               (double) Time duration.
                 * @param opt_qddot_mobile       (Eigen::VectorXd) Output optimal mobile base accelerations.
                 * @param opt_torque_manipulator (Eigen::VectorXd) Output optimal manipulator joint torques.
                 * @param time_verbose  (bool) If true, print the formatted computation time information to std::cout.
                 * @return (bool) True if the problem was solved successfully.
                */
                virtual bool QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                       const bool time_verbose=false);

   
                
                int getManipulatorDof() const { return mani_dof_; }
                int getMobileDof() const { return mobi_dof_; }

            protected:
                double dt_;                                                 // Control time step in seconds
                int dof_;                                                   // Total degrees of freedom
                int mani_dof_;                                              // Manipulator DOF
                int mobi_dof_;                                              // Mobile robot DOF
                int actuator_dof_;                                          // Number of actuated joints
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data.

                // Task-space gains
                std::map<std::string, Vector6d> link_IK_Kp_task_;
                std::map<std::string, Vector6d> link_ID_Kp_task_;
                std::map<std::string, Vector6d> link_ID_Kv_task_;

                // Joint space gains
                VectorXd Kp_mani_joint_;
                VectorXd Kv_mani_joint_;

                // QP solvers
                std::unique_ptr<MobileManipulator::QPIK> QP_moma_IK_;
                std::unique_ptr<MobileManipulator::QPID> QP_moma_ID_;

                /**
                 * @brief Internal CLIK overload that receives desired task velocities directly.
                 */
                virtual bool CLIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  const Eigen::Ref<const VectorXd>& null_qdot);

                /**
                 * @brief Internal QPIK overload that stores QP timing information in a string.
                 */
                virtual bool QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  std::string& time_verbose);
                /**
                 * @brief Internal OSF overload that receives desired task accelerations directly.
                 */
                virtual bool OSF(const std::map<std::string, Vector6d>& link_xddot_target,
                                 Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                 Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                 const Eigen::Ref<const VectorXd>& null_torque);
                /**
                 * @brief Internal compatibility overload of QPIK with legacy bool argument.
                 */
                virtual bool QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                  const bool time_verbose=false);

                /**
                 * @brief Internal QPID overload that stores QP timing information in a string.
                 */
                virtual bool QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  std::string& time_verbose);
                /**
                 * @brief Internal compatibility overload of QPID with legacy bool argument.
                 */
                virtual bool QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  const bool time_verbose=false);
        };
    } // namespace MobileManipulator
} // namespace drc
