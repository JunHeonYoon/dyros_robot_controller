#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Class for solving inverse dynamics QP problems for mobile manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse dynamics problems for mobile manipulators using Quadratic Programming.
         */
        class QPID : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                // TODO: add document to notion that add dt
                QPID(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);
                 /**
                 * @brief Set the wight vector for the cost terms
                 * @param w_tracking (Eigen::VectorXd) Weight for task acceleration tracking; its size must be 6.
                 * @param w_vel_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as actuator dof.
                 * @param w_acc_damping  (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as actuator dof.
                 */
                // TODO: add document to notion
                void setWeight(const VectorXd w_tracking, const VectorXd w_vel_damping, const VectorXd w_acc_damping);
                /**
                 * @brief Set the desired task space acceleration for the link.
                 * @param xddot_desired (Eigen::VectorXd) Desired task space acceleration.
                 * @param link_name     (std::string) Name of the link.
                 */
                void setDesiredTaskAcc(const VectorXd &xddot_desired, const std::string &link_name);
                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP.
                 * @param opt_etadot   (Eigen::VectorXd) Optimal joint acceleration.
                 * @param opt_torque   (Eigen::VectorXd) Optimal joint torque.
                 * @param time_status  (TimeDuration) Time durations structure for the QP solving process
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(VectorXd &opt_etadot, VectorXd &opt_torque, QP::TimeDuration &time_status);
    
            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int eta_dot_start; // qddot_actuated 
                    int torque_start;  // torque_actuated
                    int slack_q_mani_min_start;
                    int slack_q_mani_max_start;
                    int slack_qdot_mani_min_start;
                    int slack_qdot_mani_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;
    
                    int eta_dot_size;
                    int torque_size;
                    int slack_q_mani_min_size;
                    int slack_q_mani_max_size;
                    int slack_qdot_mani_min_size;
                    int slack_qdot_mani_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // equality
                    int con_dyn_start; // dynamics constraint
    
                    int con_dyn_size;
                    
                    // inequality
                    int con_q_mani_min_start;    // min q_manipulator
                    int con_q_mani_max_start;    // max q_manipulator
                    int con_qdot_mani_min_start; // min qdot_manipulator
                    int con_qdot_mani_max_start; // max qdot_manipulator
                    int con_sing_start;          // singularity
                    int con_sel_col_start;       // self collision
    
                    int con_q_mani_min_size;    // min q_manipulator size
                    int con_q_mani_max_size;    // max q_manipulator size
                    int con_qdot_mani_min_size; // min qdot_manipulator size
                    int con_qdot_mani_max_size; // max qdot_manipulator size
                    int con_sing_size;          // singularity size
                    int con_sel_col_size;       // self collision size
                }si_index_;
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.
                double dt_; // control time step size
                
                int actuator_dof_;              // Number of actuators in the mobile manipulator
                int mani_dof_;                  // Number of joints in the manipulator
                int mobi_dof_;                  // Number of degrees of freedom in the mobile base
                
                VectorXd xddot_desired_;        // Desired task acceleration
                std::string link_name_;         // Name of the link

                VectorXd w_tracking_;     // weight for tracking;             || x_ddot_des - J_tilda*eta_dot - J_tilda_dot*eta ||
                VectorXd w_vel_damping_;  // weight for velocity damping;     || eta_dot*dt + eta ||^2
                VectorXd w_acc_damping_;  // weight for acceleration damping; || eta_ddot ||^2

                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 * 
                 *       min     || x_ddot_des - J_tilda*eta_dot - J_tilda_dot*eta ||_W1^2 + || eta_dot ||_W2^2 + || eta_dot*dt + eta ||_W3^2
                 *  [eta_dot, torque]
                 *
                 * =>      min        1/2 * [ eta_dot ].T * [ 2*J_tilda.T*W1*J_tilda + 2*W2 + 2*dt*dt*W3  0 ] * [ eta_dot ] + [ -2*J_tilda.T*W1*(x_ddot_des - J_tilda_dot*eta) + 2*dt*eta].T * [ eta_dot ]
                 *  [eta_dot, torque]       [ torque  ]     [                      0                      0 ]   [ torque  ]   [                                0                         ]     [ torque  ]
                 */
                void setCost() override;
                /**
                 * @brief Not yet implemented.
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit manipulator joint angles and velocities and avoid singularity and self collision.
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which forces joint accelerations and torques to match the equations of motion.
                 * 
                 * subject to M_tilda * eta_dot + g_tilda = torque
                 *
                 * => subject to [ M_tilda -I ][ eta_dot ] = [ -g_tilda ]
                 *                             [ torque  ]
                 */
                void setEqConstraint() override;
        };
    } // namespace MobileManipulator
} // namespace drc
