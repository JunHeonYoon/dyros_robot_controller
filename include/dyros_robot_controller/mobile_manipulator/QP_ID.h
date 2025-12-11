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
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 * @param w_vel_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as actuator dof.
                 * @param w_acc_damping  (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as actuator dof.
                 */
                // TODO: add document to notion
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking, const VectorXd w_vel_damping, const VectorXd w_acc_damping);
                /**
                 * @brief Set the desired task space acceleration for the link.
                 * @param link_xddot_desired (std::map<std::string, Vector6d>) Desired task space acceleration (6D twist) per links.
                 * @param link_name     (std::string) Name of the link.
                 */
                // TODO: add document to notion
                void setDesiredTaskAcc(const std::map<std::string, Vector6d> &link_xddot_desired);
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
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_; // Shared pointer to the robot data class.
                double dt_;                                                // control time step size
                int actuator_dof_;                                         // Number of actuators in the mobile manipulator
                int mani_dof_;                                             // Number of joints in the manipulator
                int mobi_dof_;                                             // Number of degrees of freedom in the mobile base
                
                std::map<std::string, Vector6d> link_xddot_desired_; // Desired task acceleration per links
                std::map<std::string, Vector6d> link_w_tracking_; // weight for task acceleration tracking per links; || x_i_ddot_des - J_i_tilda*eta_dot - J_i_tilda_dot*eta ||
                VectorXd w_vel_damping_;                          // weight for velocity damping;                     || eta_dot*dt + eta ||
                VectorXd w_acc_damping_;                          // weight for acceleration damping;                 || eta_ddot ||

                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *         min       || x_i_ddot_des - J_i_tilda*eta_dot - J_i_tilda_dot*eta ||_Wi^2 + || q_ddot ||_W2^2 + || q_ddot*dt + eta ||_W3^2 + 1000*s 
                 *  [eta_dot, torque, s]
                 *
                 * =>      min         1/2 * [ eta_dot ].T * [ 2*J_i_tilda.T*Wi_i*J + 2*W2 + 2*dt*dt*W3  0  0 ] * [ eta_dot  ] + [ -2*J_i_tilda.T*Wi*(x_i_ddot_des - J_i_tilda_dot*eta) + 2*dt*eta ].T * [ eta_dot  ]
                 *  [eta_dot, torque, s]     [  torque ]     [                     0                     0  0 ]   [ torque ]     [                               0                                 ]     [   torque ]
                 *                           [    s    ]     [                     0                     0  0 ]   [   s    ]     [                              1000                               ]     [    s     ]
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint to keep all slack variables non-negative.
                 * 
                 *      subject to [ -inf ] <= [ eta_dot ] <= [ inf ]
                 *                 [ -inf ]    [  torque ]    [ inf ]
                 *                 [   0  ]    [    s    ]    [ inf ]
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit manipulator joint angles and velocities, avoid singularity and self collision by 1st or 2nd-order CBF.
                 * 
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 2nd-order CBF condition with slack: hddot(x) >= -2*a*hdot(x) -a*a*h(x) - s
                 * 
                 *  1. (2nd-order CBF)
                 *     Manipulator joint angle limit: h_p_min(q_mani) = q_mani - q_mani_min >= 0  -> hdot_p_min(q_mani) = qdot_mani   -> hddot_p_min(q_mani) = qddot_mani
                 *                                    h_p_max(q_mani) = q_mani_max - q_mani >= 0  -> hdot_p_max(q_mani) = -qdot_mani  -> hddot_p_max(q_mani) = -qddot_mani
                 *     
                 *     => subject to [  I_mani  0  I ] * [ eta_dot ] >= [ -2*a*qdot_mani -a*a*(q_mani - q_mani_min) ]
                 *                   [ -I_mani  0  I ]   [  torque ]    [  2*a*qdot_mani -a*a*(q_mani_max - q_mani) ]
                 *                                       [    s    ]
                 * 
                 *  2.(1st-order CBF)
                 *     Manipulator joint velocity limit: h_v_min(qdot_mani) = qdot_mani - qdot_mani_min >= 0  -> hdot_v_min(qdot_mani) = qddot_mani 
                 *                                       h_v_max(qdot_mani) = qdot_mani_max - qdot_mani >= 0  -> hdot_v_max(qdot_mani) = -qddot_mani
                 *     
                 *     => subject to [  I_mani  0  I ] * [ eta_dot ] >= [ -a*(qdot_mani - qdot_mani_min) ]
                 *                   [ -I_mani  0  I ]   [  torque ]    [ -a*(qdot_mani_max - qdot_mani) ]
                 *                                       [    s    ]
                 * 
                 *  3. (2nd-order CBF)
                 *     Singluarity avoidance: h_sing(q_mani) = manipulability(q_mani) - eps_sing_min >= 0 -> hdot_sing = ∇_(q_mani) manipulability.T * qdot_mani  -> hddot_sing = (∇_(q_mani) manipulability.T)dot * qdot_mani +  ∇_(q) manipulability.T * qddot_mani
                 * 
                 *     => subject to [ ∇_(q_mani) manipulability.T  0  I ] * [ eta_dot ] >= [ -(∇_(q_mani) manipulability.T)dot*qdot_mani - 2*a*∇_(q_mani) manipulability.T * qdot_mani -a*a*(manipulability - eps_sing_min) ]
                 *                                                           [  torque ]
                 *                                                           [    s    ]
                 *  4. (2nd-order CBF)
                 *      Self collision avoidance: h_selcol(q_mani) = self_dist(q_mani) - eps_selcol_min >= 0 -> hdot_selcol = ∇_(q_mani) self_dist^T * qdot_mani  -> hddot_selcol = (∇_(q_mani) self_dist.T)dot * qdot_mani +  ∇_(q) self_dist.T * qddot_mani
                 *      
                 *     => subject to [ ∇_(q_mani) self_dist.T  0  I ] * [ eta_dot ] >= [ -(∇_(q_mani) self_dist.T)dot*qdot_mani - 2*a*∇_(q_mani) self_dist.T * qdot_mani -a*a*(self_dist - eps_selcol_min) ]
                 *                                                      [  torque ]
                 *                                                      [    s    ]
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which forces joint accelerations and torques to match the equations of motion.
                 * 
                 * subject to M_tilda * eta_dot + g_tilda = torque
                 *
                 * => subject to [ M_tilda -I 0 ][ eta_dot ] = [ -g_tilda ]
                 *                               [ torque  ]
                 *                               [    s    ]
                 */
                void setEqConstraint() override;
        };
    } // namespace MobileManipulator
} // namespace drc
