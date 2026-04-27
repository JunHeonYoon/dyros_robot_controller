// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
//
// Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
// Seoul National University, under a research agreement with ETRI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
                QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);

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
                 * @brief Set null space velocity tracking weights only.
                 * @param w_null_joint_vel (Eigen::VectorXd) Weight for null space velocity tracking toward null_eta_desired; its size must same as actuator_dof.
                 */
                void setNullJointVelWeight(const Eigen::Ref<const VectorXd>& w_null_joint_vel) { w_null_joint_vel_ = w_null_joint_vel; }

                /**
                 * @brief Set the desired actuated velocity for null space tracking.
                 *        When set, the null space cost becomes ||N_tilda*(eta - null_eta_desired)||_W_null^2,
                 *        where N_tilda = I - J_tilda†J_tilda is the task null space projector.
                 * @param null_eta_desired (Eigen::VectorXd) Desired actuated velocity [qdot_mani; v_mobi]; its size must same as actuator_dof.
                 */
                void setDesiredNullJointVel(const Eigen::Ref<const VectorXd>& null_eta_desired) { null_eta_desired_ = null_eta_desired; }

                /**
                 * @brief Set manipulator joint acceleration damping weights only.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping; its size must same as mani_dof.
                 */
                void setManiJointAccWeight(const Eigen::Ref<const VectorXd>& w_mani_acc_damping) { w_mani_acc_damping_ = w_mani_acc_damping; }

                /**
                 * @brief Set mobile base acceleration damping weights only.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setBaseAccWeight(const Eigen::Vector3d& w_base_acc_damping) { w_base_acc_damping_ = w_base_acc_damping; }

                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 * @param w_null_joint_vel (Eigen::VectorXd) Weight for null space velocity tracking toward null_eta_desired; its size must same as actuator_dof.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping toward qdot_now; its size must same as mani_dof.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setWeight(const Vector6d& w_tracking,
                               const Eigen::Ref<const VectorXd>& w_null_joint_vel,
                               const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                               const Eigen::Vector3d& w_base_acc_damping);

                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 * @param w_null_joint_vel (Eigen::VectorXd) Weight for null space velocity tracking toward null_eta_desired; its size must same as actuator_dof.
                 * @param w_mani_acc_damping (Eigen::VectorXd) Weight for manipulator joint acceleration damping toward qdot_now; its size must same as mani_dof.
                 * @param w_base_acc_damping (Eigen::Vector3d) Weight for mobile base acceleration damping.
                 */
                void setWeight(const std::map<std::string, Vector6d>& link_w_tracking,
                               const Eigen::Ref<const VectorXd>& w_null_joint_vel,
                               const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                               const Eigen::Vector3d& w_base_acc_damping);
                
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
                    int con_base_vel_start;      // mobile base velocity
                    int con_base_acc_start;      // mobile base acceleration
    
                    int con_q_mani_min_size;    // min q_manipulator size
                    int con_q_mani_max_size;    // max q_manipulator size
                    int con_sing_size;          // singularity size
                    int con_sel_col_size;       // self collision size
                    int con_base_vel_size;      // mobile base velocity size
                    int con_base_acc_size;      // mobile base acceleration size
                }si_index_;
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int actuator_dof_;                                           // Number of actuators in the mobile manipulator
                int mani_dof_;                                               // Number of joints in the manipulator
                int mobi_dof_;                                               // Number of degrees of freedom in the mobile base

                std::map<std::string, Vector6d> link_xdot_desired_; // Desired task velocity per links
                std::map<std::string, Vector6d> link_w_tracking_;   // weight for task velocity tracking per links; || x_i_dot_des - J_i_tilda*eta ||
                VectorXd null_eta_desired_;                         // desired actuated velocity [qdot_mani; v_mobi] for null space tracking (default: zero)
                VectorXd w_null_joint_vel_;                               // weight for null space velocity tracking;      || N_tilda*(eta - null_eta_desired) ||
                VectorXd w_mani_acc_damping_;                       // weight for manipulator joint acceleration damping; || (eta_mani - eta_mani_now) / dt ||
                Vector3d w_base_acc_damping_;                       // weight for mobile base acceleration damping;  || (v_base - v_base_now) / dt ||

                // self-collision CBF gradient exponential filter
                // smooths discontinuous jumps when the closest collision pair changes
                VectorXd col_grad_filtered_;
                bool     col_grad_initialized_  = false;
                double   col_grad_filter_alpha_ = 0.2;  // filter coefficient (0~1): larger = faster tracking, less smoothing

                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *       min     Σ_i || x_i_dot_des - J_i_tilda*eta ||_Wi^2
                 *     [eta,s]     + || N_tilda*(eta - null_eta_des) ||_W_null^2
                 *                 + || (eta_mani - eta_mani_now) / dt ||_W3^2
                 *                 + || (v_base - v_base_now) / dt ||_W5^2
                 *                 + 1000*s
                 *
                 *        where N_tilda = I - J_tilda†J_tilda  (actuated null space projector),  NWN = N_tilda^T * W_null * N_tilda
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
