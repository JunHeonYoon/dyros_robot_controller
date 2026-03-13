#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Class for solving inverse kinematics QP problems for manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse kinematics problems for manipulators using Quadratic Programming.
         */
        class QPIK : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                QPIK(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 */
                void setTrackingWeight(const Vector6d w_tracking);
                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
                 */
                void setTrackingWeight(const std::map<std::string, Vector6d> link_w_tracking) { link_w_tracking_ = link_w_tracking; }
                /**
                 * @brief Set joint velocity damping weights only.
                 * @param w_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }
                /**
                 * @brief Set joint acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setWeight(const Vector6d w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping);
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping);

                /**
                 * @brief Set the desired task space velocity for the link.
                 * @param link_xdot_desired (std::map<std::string, Vector6d>) Desired task space velocity (6D twist) per links.
                 */ 
                void setDesiredTaskVel(const std::map<std::string, Vector6d> &link_xdot_desired);
                /**
                 * @brief Get the optimal joint velocity by solving QP.
                 * @param opt_qdot    (Eigen::VectorXd) Optimal joint velocity.
                 * @param time_status (TimeDuration) Time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJointVel(Eigen::Ref<Eigen::VectorXd> opt_qdot, QP::TimeDuration &time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qdot_start; 
                    int slack_q_min_start;
                    int slack_q_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;

                    int qdot_size;
                    int slack_q_min_size;
                    int slack_q_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;

                    // inequality
                    int con_q_min_start;
                    int con_q_max_start;
                    int con_sing_start;    // singularity
                    int con_sel_col_start; // self collision

                    int con_q_min_size;
                    int con_q_max_size;
                    int con_sing_size;
                    int con_sel_col_size;
                }si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                           // control time step size
                int joint_dof_;                                       // Number of joints in the manipulator

                std::map<std::string, Vector6d> link_xdot_desired_; // Desired task velocity per links
                std::map<std::string, Vector6d> link_w_tracking_;   // weight for task velocity tracking per links; ||x_i_dot_des - J_i*q_dot||
                VectorXd w_vel_damping_;                            // weight for joint velocity damping;           || q_dot ||
                VectorXd w_acc_damping_;                            // weight for joint acceleration damping;       || (q_dot - q_dot_now) / dt ||
                

                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 * 
                 *       min      || x_i_dot_des - J_i*q_dot ||_W1^2 + || q_dot ||_W2^2 + || (q_dot - q_dot_now) / dt ||_W3^2 + 1000*s
                 *     [qdot,s]
                 * 
                 * =>    min     1/2 [ qdot ]^T * [ 2*J_i.T*W1*J_i + 2*W2 + 2/dt^2*W3     0 ] * [ qdot ] + [ -2*J_i.T*W1*x_i_dot_des - 2/dt^2*W3*q_dot_now ].T * [ qdot ]
                 *     [qdot,s]      [   s  ]     [                   0                     0 ]   [   s  ]   [                      1000                      ]     [  s   ]
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint which limits manipulator joint velocities and keeps all slack variables non-negative.
                 * 
                 *     subject to [ qdot_min ] <= [ qdot ] <= [ qdot_max ]
                 *                [    0     ]    [   s  ]    [   inf    ]
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which manipulator limit joint angles and avoid singularity and self collision by 1st-order CBF.
                 * 
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 
                 *  1.
                 *     Manipulator joint angle limit: h_min(q) = q - q_min >= 0  -> hdot_min(q) = qdot
                 *                                    h_max(q) = q_max - q >= 0  -> hdot_max(q) = -qdot
                 *     
                 *     => subject to [  I  I ] * [ qdot ] >= [ -a*(q - q_min) ]
                 *                   [ -I  I ]   [  s   ]    [ -a*(q_max - q) ]
                 *  2.
                 *     Singluarity avoidance: h_sing(q) = manipulability(q) - eps_sing_min >= 0 -> hdot_sing = ∇_(q) manipulability.T * qdot
                 * 
                 *     => subject to [ ∇_(q) manipulability.T  I ] * [ qdot ] >= [ -a*(manipulability - eps_sing_min) ]
                 *                                                   [  s   ]
                 *  3. 
                 *      Self collision avoidance: h_selcol(q) = self_dist(q) - eps_selcol_min >= 0 -> hdot_selcol = ∇_q self_dist^T * qdot
                 *      
                 *     => subject to [ ∇_(q) self_dist.T  I ] * [ qdot ] >= [ -a*(self_dist - eps_selcol_min) ]
                 *                                              [  s   ]
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Not implemented.
                 */
                void setEqConstraint() override;
        };
    } // namespace Manipulator
} // namespace drc
