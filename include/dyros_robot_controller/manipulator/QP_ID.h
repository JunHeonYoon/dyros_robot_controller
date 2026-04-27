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
                QPID(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt);
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
                 * @brief Set joint velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }
                /**
                 * @brief Set joint acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }
                /**
                 * @brief Set the desired null space torque tracking weights only.
                 * @param w_null_torque (Eigen::VectorXd) Weight for null space torque tracking; its size must be same as dof.
                 */
                void setNullTorqueWeight(const Eigen::Ref<const VectorXd>& w_null_torque) { w_null_torque_ = w_null_torque; }
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 * @param w_null_torque (Eigen::VectorXd) Diagonal weight for null space torque tracking; its size must be same as dof.
                 */
                void setWeight(const Vector6d w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping,
                               const Eigen::Ref<const VectorXd>& w_null_torque);
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 * @param w_null_torque (Eigen::VectorXd) Diagonal weight for null space torque tracking; its size must be same as dof.
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
                 *        This is the desired total null-space torque.
                 * @param null_torque (Eigen::VectorXd) Desired joint torque to track in the null space; its size must same as dof.
                 */
                void setNullTorque(const Eigen::Ref<const VectorXd>& null_torque) { null_torque_ = null_torque; }
                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP.
                 * @param opt_qddot   (Eigen::VectorXd) Optimal joint accelerations.
                 * @param opt_torque  (Eigen::VectorXd) Optimal joint torques.
                 * @param time_status (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<Eigen::VectorXd> opt_qddot, Eigen::Ref<Eigen::VectorXd> opt_torque, QP::TimeDuration &time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
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
                VectorXd w_null_torque_;                             // weight for null space torque tracking;           || N*(torque - null_torque) ||

                VectorXd null_torque_;                               // desired total null space torque

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
                 *       min      Σ_i || x_i_ddot_des - J_i*qddot - J_i_dot*qdot ||_W1_i^2
                 * [qddot,tau,s]    + || qddot ||_W2^2
                 *                  + || dt*qddot + qdot ||_W3^2
                 *                  + || N*(tau - null_tau_des) ||_W4^2
                 *                  + 100*s
                 *        where N = I - J^T*Λ*J*M^-1  (dynamic-consistent torque null projector, J = [J_1; ...; J_n]),  Λ = (J*M^-1*J^T)^+,  W4 = diag(w_null_torque),  NWN = N^T*W4*N
                 *
                 * =>    min       1/2 [ qddot ]^T * [ 2*Σ(J_i.T*W1_i*J_i) + 2*W2 + 2*dt^2*W3     0       0 ] * [ qddot ] + [ -2*Σ(J_i.T*W1_i*(x_i_ddot_des - J_i_dot*qdot)) + 2*dt*W3*qdot ].T * [ qddot ]
                 * [qddot,tau,s]       [  tau  ]     [                         0                 2*NWN    0 ]   [  tau  ]   [                     -2*NWN*null_tau_des                       ]     [  tau  ]
                 *                     [   s   ]     [                         0                  0       0 ]   [   s   ]   [                             100                               ]     [   s   ]
                 */
                void setCost() override;
                /**
                 * @brief Set variable bounds.
                 * 
                 *      subject to [ -inf    ] <= [ qddot  ] <= [ inf     ]
                 *                 [ tau_min ]    [ torque ]    [ tau_max ]
                 *                 [   0     ]    [    s   ]    [ inf     ]
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set inequality constraints for joint limits and self collision.
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
                 *  3. Singularity avoidance:
                 *     A CBF row is allocated for this term, but it is currently inactive in the implementation.
                 *
                 *  4. (2nd-order CBF, active only when a valid distance gradient is available)
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
                 * subject to M * qddot + nle = torque
                 *
                 * => subject to [ M -I 0 ][ qddot  ] = [ -nle ]
                 *                         [ torque ]
                 *                         [   s    ]
                 */
                void setEqConstraint() override;
        };
    } // namespace Manipulator
} // namespace drc
