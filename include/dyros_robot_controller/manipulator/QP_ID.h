#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Class for solving inverse dynamics QP problems for manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse dynamics problems for manipulators using Quadratic Programming.
         */
        class QPID : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                // TODO: add document to notion that add dt
                QPID(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt);
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
                 * @param opt_qddot   (Eigen::VectorXd) Optimal joint accelerations.
                 * @param opt_torque  (Eigen::VectorXd) Optimal joint torques.
                 * @param time_status (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(VectorXd &opt_qddot, VectorXd &opt_torque, QP::TimeDuration &time_status);

            private:
                /**
                 * @brief Struct to hold the indicies of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qddot_start; 
                    int torque_start;
                    int slack_q_min_start;
                    int slack_q_max_start;
                    int slack_qdot_min_start;
                    int slack_qdot_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;

                    int qddot_size;
                    int torque_size;
                    int slack_q_min_size;
                    int slack_q_max_size;
                    int slack_qdot_min_size;
                    int slack_qdot_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // equality
                    int con_dyn_start; // dynamics constraint

                    int con_dyn_size;
                    
                    // inequality
                    int con_q_min_start;    // min joint angle
                    int con_q_max_start;    // max joint angle
                    int con_qdot_min_start; // min joint velocity
                    int con_qdot_max_start; // max joint velocity
                    int con_sing_start;     // singularity
                    int con_sel_col_start;  // self collision

                    int con_q_min_size;    // min joint angle size
                    int con_q_max_size;    // max joint angle size
                    int con_qdot_min_size; // min joint velocity size
                    int con_qdot_max_size; // max joint velocity size
                    int con_sing_size;     // singularity size
                    int con_sel_col_size;  // self collision size

                }si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_; // Shared pointer to the robot data class.
                double dt_;                                          // control time step size
                int joint_dof_;                                      // Number of joints in the manipulator

                std::map<std::string, Vector6d> link_xddot_desired_; // Desired task acceleration per links
                std::map<std::string, Vector6d> link_w_tracking_;    // weight for task acceleration tracking per links; || x_i_ddot_des - J_i*qddot - J_i_dot*qdot ||
                VectorXd w_vel_damping_;                             // weight for joint velocity damping;               || q_ddot*dt + qdot ||
                VectorXd w_acc_damping_;                             // weight for joint acceleration damping;           || q_ddot ||
                
                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *         min       || x_i_ddot_des - J_i*qddot - J_i_dot*qdot ||_Wi^2 + || q_ddot ||_W2^2 + || q_ddot*dt + qdot ||_W3^2 + 1000*s 
                 *  [qddot, torque, s]
                 *
                 * =>      min         1/2 * [ qddot  ].T * [ 2*J_i.T*Wi_i*J + 2*W2 + 2*dt*dt*W3  0  0 ] * [ qddot  ] + [ -2*J_i.T*Wi*(x_i_ddot_des - J_i_dot*qdot) + 2*dt*qdot ].T * [ qddot  ]
                 *  [qddot, torque, s]       [ torque ]     [                     0               0  0 ]   [ torque ]   [                           0                           ]     [ torque ]
                 *                           [   s    ]     [                     0               0  0 ]   [   s    ]   [                          1000                         ]     [   s    ]
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint to keep all slack variables non-negative.
                 * 
                 *      subject to [ -inf ] <= [ qddot  ] <= [ inf ]
                 *                 [ -inf ]    [ torque ]    [ inf ]
                 *                 [   0  ]    [    s   ]    [ inf ]
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit manipulator joint angles and velocities, avoid singularity and self collision by 1st or 2nd-order CBF.
                 * 
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 2nd-order CBF condition with slack: hddot(x) >= -2*a*hdot(x) -a*a*h(x) - s
                 * 
                 *  1. (2nd-order CBF)
                 *     Manipulator joint angle limit: h_p_min(q) = q - q_min >= 0  -> hdot_p_min(q) = qdot   -> hddot_p_min(q) = qddot
                 *                                    h_p_max(q) = q_max - q >= 0  -> hdot_p_max(q) = -qdot  -> hddot_p_max(q) = -qddot
                 *     
                 *     => subject to [  I  0  I ] * [ qddot  ] >= [ -2*a*qdot -a*a*(q - q_min) ]
                 *                   [ -I  0  I ]   [ torque ]    [  2*a*qdot -a*a*(q_max - q) ]
                 *                                  [    s   ]
                 * 
                 *  2.(1st-order CBF)
                 *     Manipulator joint velocity limit: h_v_min(qdot) = qdot - qdot_min >= 0  -> hdot_v_min(qdot) = qddot 
                 *                                       h_v_max(qdot) = qdot_max - qdot >= 0  -> hdot_v_max(qdot) = -qddot
                 *     
                 *     => subject to [  I  0  I ] * [ qddot  ] >= [ -a*(qdot - qdot_min) ]
                 *                   [ -I  0  I ]   [ torque ]    [ -a*(qdot_max - qdot) ]
                 *                                  [    s   ]
                 * 
                 *  3. (2nd-order CBF)
                 *     Singluarity avoidance: h_sing(q) = manipulability(q) - eps_sing_min >= 0 -> hdot_sing = ∇_(q) manipulability.T * qdot  -> hddot_sing = (∇_(q) manipulability.T)dot * qdot +  ∇_(q) manipulability.T * qddot
                 * 
                 *     => subject to [ ∇_(q) manipulability.T  0  I ] * [ qddot  ] >= [ -(∇_(q) manipulability.T)dot*qdot - 2*a*∇_(q) manipulability.T * qdot -a*a*(manipulability - eps_sing_min) ]
                 *                                                      [ torque ]
                 *                                                      [    s   ]
                 *  4. (2nd-order CBF)
                 *      Self collision avoidance: h_selcol(q) = self_dist(q) - eps_selcol_min >= 0 -> hdot_selcol = ∇_q self_dist^T * qdot  -> hddot_selcol = (∇_(q) self_dist.T)dot * qdot +  ∇_(q) self_dist.T * qddot
                 *      
                 *     => subject to [ ∇_(q) self_dist.T  0  I ] * [ qddot  ] >= [ -(∇_(q) self_dist.T)dot*qdot - 2*a*∇_(q) self_dist.T * qdot -a*a*(self_dist - eps_selcol_min) ]
                 *                                                 [ torque ]
                 *                                                 [    s   ]
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraint which forces joint accelerations and torques to match the equations of dynamics.
                 *
                 * subject to M * qddot + g = torque
                 *
                 * => subject to [ M -I 0 ][ qddot  ] = [ -g ]
                 *                         [ torque ]
                 *                         [   s    ]
                 */
                void setEqConstraint() override;
        };
    } // namespace QP
} // namespacce drc
