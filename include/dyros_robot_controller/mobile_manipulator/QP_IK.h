#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Class for solving inverse kinematics QP problems for mobile manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse kinematics problems for mobile manipulators using Quadratic Programming.
         */
        class QPIK : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                // TODO: add document to notion that add dt
                QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);
                /**
                 * @brief Set the wight vector for the cost terms
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as actuator dof.
                 */
                // TODO: add document to notion
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking, const VectorXd w_damping);
                /**
                 * @brief Set the desired task space velocity for the link.
                 * @param link_xdot_desired (std::map<std::string, Vector6d>) Desired task space velocity (6D twist) per links.
                 */
                // TODO: add document to notion
                void setDesiredTaskVel(const std::map<std::string, Vector6d> &link_xdot_desired);
                /**
                 * @brief Get the optimal joint velocity by solving QP.
                 * @param opt_qdot    (Eigen::VectorXd) Optimal joint velocity.
                 * @param time_status (TimeDuration) Time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJointVel(VectorXd &opt_qdot, QP::TimeDuration &time_status);
    
            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int eta_start; // qdot_actuated
                    int slack_q_mani_min_start;
                    int slack_q_mani_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;
    
                    int eta_size;
                    int slack_q_mani_min_size;
                    int slack_q_mani_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // inequality
                    int con_q_mani_min_start;    // min q_manipulator
                    int con_q_mani_max_start;    // max q_manipulator
                    int con_sing_start;          // singularity
                    int con_sel_col_start;       // self collision
    
                    int con_q_mani_min_size;    // min q_manipulator size
                    int con_q_mani_max_size;    // max q_manipulator size
                    int con_sing_size;          // singularity size
                    int con_sel_col_size;       // self collision size
                }si_index_;
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int actuator_dof_;                                           // Number of actuators in the mobile manipulator
                int mani_dof_;                                               // Number of joints in the manipulator
                int mobi_dof_;                                               // Number of degrees of freedom in the mobile base

                std::map<std::string, Vector6d> link_xdot_desired_; // Desired task velocity per links
                std::map<std::string, Vector6d> link_w_tracking_;   // weight for task velocity tracking per links; || x_i_dot_des - J_i_tilda*eta ||
                VectorXd w_damping_;                                // weight for joint velocity damping;           || eta ||
                
                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 * 
                 *       min     || x_i_dot_des - J_i_tilda*eta ||_Wi^2 + || eta ||_W2^2 + 1000*s
                 *     [eta,s]
                 *
                 * =>    min     1/2 [ eta ]^T * [ 2*J_i_tilda.T*Wi*J_i_tilda + 2*W2  0 ] * [ eta ] + [ -2*J_i_tilda.T*Wi*x_i_dot_des ].T * [ eta ]
                 *     [eta,s]       [  s  ]     [                 0                  0 ]   [  s  ]   [             1000              ]     [  s  ]
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint which limits manipulator joint velocities and keeps all slack variables non-negative.
                 * 
                 *     subject to [ qdot_mani_min ] <= [ eta ] <= [ qdot_mani_max ]
                 *                [       0       ]    [  s  ]    [      inf      ]
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which manipulator limit joint angles and avoid singularity and self collision by 1st-order CBF.
                 * 
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 
                 *  1.
                 *     Manipulator joint angle limit: h_min(q) = q_mani - q_mani_min >= 0  -> hdot_min(q) = qdot_mani
                 *                                    h_max(q) = q_mani_max - q_mani >= 0  -> hdot_max(q) = -qdot_mani
                 *     
                 *     => subject to [  I_mani  I ] * [ eta ] >= [ -a*(q_mani - q_mani_min) ]
                 *                   [ -I_mani  I ]   [  s  ]    [ -a*(q_mani_max - q_mani) ]
                 *  2.
                 *     Singluarity avoidance: h_sing(q_mani) = manipulability(q_mani) - eps_sing_min >= 0 -> hdot_sing = ∇_(q_mani) manipulability.T * qdot_mani
                 * 
                 *     => subject to [ ∇_(q_mani) manipulability.T  I ] * [ eta ] >= [ -a*(manipulability - eps_sing_min) ]
                 *                                                        [  s  ]
                 *  3. 
                 *      Self collision avoidance: h_selcol(q_mani) = self_dist(q_mani) - eps_selcol_min >= 0 -> hdot_col = ∇_q self_dist^T * qdot_mani
                 *      
                 *     => subject to [ ∇_(q_mani) self_dist.T  I ] * [ eta ] >= [ -a*(self_dist - eps_selcol_min) ]
                 *                                                   [  s  ]
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Not implemented.
                 */
                void setEqConstraint() override;
        };
    } // namespace MobileManipulator
} // namespace drc