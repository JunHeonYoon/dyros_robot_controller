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
                QPID(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 */
                void setTrackingWeight(const Vector6d w_tracking);

                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 */
                void setTrackingWeight(const std::map<std::string, Vector6d> link_w_tracking) { link_w_tracking_ = link_w_tracking; }

                /**
                 * @brief Set actuator-space velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set actuator-space acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }
                /**
                 * @brief Set null space torque tracking weights only.
                 * @param w_null_torque (Eigen::VectorXd) Diagonal weight for actuator-space null torque tracking; its size must same as actuator_dof.
                 */
                void setNullTorqueWeight(const Eigen::Ref<const VectorXd>& w_null_torque) { w_null_torque_ = w_null_torque; }

                /**
                 * @brief Set the weight vectors for the cost terms.
                 * @param w_tracking (Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 * @param w_null_torque (Eigen::VectorXd) Diagonal weight for null space torque tracking; its size must same as actuator_dof.
                 */
                void setWeight(const Vector6d w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping,
                               const Eigen::Ref<const VectorXd>& w_null_torque);
                
                /**
                 * @brief Set the weight vectors for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 * @param w_null_torque (Eigen::VectorXd) Diagonal weight for null space torque tracking; its size must same as actuator_dof.
                 */
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping,
                               const Eigen::Ref<const VectorXd>& w_null_torque);

                /**
                 * @brief Set the desired task space acceleration for each link.
                 * @param link_xddot_desired (std::map<std::string, Vector6d>) Desired task space acceleration (6D twist) per links.
                 */
                void setDesiredTaskAcc(const std::map<std::string, Vector6d> &link_xddot_desired);

                /**
                 * @brief Set the desired null space torque.
                 * @param null_torque (Eigen::VectorXd) Desired joint torque to track in the null space; its size must same as actuator_dof.
                 */
                void setNullTorque(const Eigen::Ref<const VectorXd>& null_torque) { null_torque_ = null_torque; }


                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP.
                 * @param opt_etadot   (Eigen::VectorXd) Optimal joint acceleration.
                 * @param opt_torque   (Eigen::VectorXd) Optimal joint torque.
                 * @param time_status  (TimeDuration) Time durations structure for the QP solving process
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<Eigen::VectorXd> opt_etadot, Eigen::Ref<Eigen::VectorXd> opt_torque, QP::TimeDuration &time_status);
    
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
                    int slack_base_vel_min_start;
                    int slack_base_vel_max_start;
                    int slack_base_acc_min_start;
                    int slack_base_acc_max_start;
    
                    int eta_dot_size;
                    int torque_size;
                    int slack_q_mani_min_size;
                    int slack_q_mani_max_size;
                    int slack_qdot_mani_min_size;
                    int slack_qdot_mani_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    int slack_base_vel_min_size;
                    int slack_base_vel_max_size;
                    int slack_base_acc_min_size;
                    int slack_base_acc_max_size;
                    
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
                    int con_base_vel_min_start;  // mobile base velocity min
                    int con_base_vel_max_start;  // mobile base velocity max
                    int con_base_acc_min_start;  // mobile base acceleration min
                    int con_base_acc_max_start;  // mobile base acceleration max
    
                    int con_q_mani_min_size;    // min q_manipulator size
                    int con_q_mani_max_size;    // max q_manipulator size
                    int con_qdot_mani_min_size; // min qdot_manipulator size
                    int con_qdot_mani_max_size; // max qdot_manipulator size
                    int con_sing_size;          // singularity size
                    int con_sel_col_size;       // self collision size
                    int con_base_vel_min_size;  // mobile base velocity min size
                    int con_base_vel_max_size;  // mobile base velocity max size
                    int con_base_acc_min_size;  // mobile base acceleration min size
                    int con_base_acc_max_size;  // mobile base acceleration max size
                }si_index_;
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_; // Shared pointer to the robot data class.
                double dt_;                                                // control time step size
                int actuator_dof_;                                         // Number of actuators in the mobile manipulator
                int mani_dof_;                                             // Number of joints in the manipulator
                int mobi_dof_;                                             // Number of degrees of freedom in the mobile base
                
                std::map<std::string, Vector6d> link_xddot_desired_; // Desired task acceleration per links
                std::map<std::string, Vector6d> link_w_tracking_; // weight for task acceleration tracking per links; || x_i_ddot_des - J_i_tilda*eta_dot - J_i_tilda_dot*eta ||
                VectorXd w_vel_damping_;                          // weight for actuator velocity damping;            || eta_dot*dt + eta ||
                VectorXd w_acc_damping_;                          // weight for actuator acceleration damping;        || eta_dot ||
                VectorXd w_null_torque_;                          // weight for actuator-space null torque tracking;  || N*(torque - null_torque) ||
                VectorXd null_torque_;                            // desired total null space torque

                // self-collision CBF gradient exponential filter
                // smooths discontinuous jumps when the closest collision pair changes
                VectorXd col_grad_filtered_;
                VectorXd col_grad_dot_filtered_;
                bool     col_grad_initialized_  = false;
                double   col_grad_filter_alpha_ = 0.2;  // filter coefficient (0~1): larger = faster tracking, less smoothing

                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *       min      Σ_i || x_i_ddot_des - J_i_tilda*eta_dot - J_i_tilda_dot*eta ||_W1_i^2
                 * [eta_dot,tau,s] + || eta_dot ||_W2^2
                 *                 + || dt*eta_dot + eta ||_W3^2
                 *                 + || N*(tau - null_tau_des) ||_W4^2
                 *                 + 1000*s
                 *        where N = I - J_tilda^T*Λ*J_tilda*M_tilda^-1,  Λ = (J_tilda*M_tilda^-1*J_tilda^T)^+,
                 *              W4 = diag(w_null_torque),  NWN = N^T*W4*N
                 *
                 * =>    min        1/2 [ eta_dot ]^T * [ 2*Σ(J_i_tilda.T*W1_i*J_i_tilda) + damping terms     0       0 ] * [ eta_dot ]
                 * [eta_dot,tau,s]      [   tau   ]     [                         0                              2*NWN     0 ]   [   tau   ]
                 *                      [    s    ]     [                         0                                0       0 ]   [    s    ]
                 *                    + [ -2*Σ(J_i_tilda.T*W1_i*(x_i_ddot_des - J_i_tilda_dot*eta)) + damping terms ].T * [ eta_dot ]
                 *                      [                          -2*NWN*null_tau_des                                   ]     [   tau   ]
                 *                      [                                 1000                                           ]     [    s    ]
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
