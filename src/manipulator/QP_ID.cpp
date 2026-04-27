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

#include "dyros_robot_controller/manipulator/QP_ID.h"
#include <cmath>

namespace drc
{
    namespace Manipulator
    {
        QPID::QPID(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt)
        : QP::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            joint_dof_ = robot_data_->getDof();
       
            si_index_.qddot_size          = joint_dof_;
            si_index_.torque_size         = joint_dof_;
            si_index_.slack_q_min_size    = joint_dof_;
            si_index_.slack_q_max_size    = joint_dof_;
            si_index_.slack_qdot_min_size = joint_dof_;
            si_index_.slack_qdot_max_size = joint_dof_;
            si_index_.slack_sing_size     = 1;
            si_index_.slack_sel_col_size  = 1;
            si_index_.con_dyn_size        = joint_dof_;
            si_index_.con_q_min_size      = joint_dof_;
            si_index_.con_q_max_size      = joint_dof_;
            si_index_.con_qdot_min_size   = joint_dof_;
            si_index_.con_qdot_max_size   = joint_dof_;
            si_index_.con_sing_size       = 1;
            si_index_.con_sel_col_size    = 1;

            const int nx = si_index_.qddot_size + 
                           si_index_.torque_size +
                           si_index_.slack_q_min_size +
                           si_index_.slack_q_max_size +
                           si_index_.slack_qdot_min_size +
                           si_index_.slack_qdot_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_min_size +
                              si_index_.con_q_max_size +
                              si_index_.con_qdot_min_size +
                              si_index_.con_qdot_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size ;
            const int neq = si_index_.con_dyn_size;
    
            QPBase::setQPsize(nx, nbound, nineq, neq);
    
            si_index_.qddot_start          = 0;
            si_index_.torque_start         = si_index_.qddot_start          + si_index_.qddot_size;
            si_index_.slack_q_min_start    = si_index_.torque_start         + si_index_.torque_size;
            si_index_.slack_q_max_start    = si_index_.slack_q_min_start    + si_index_.slack_q_min_size;
            si_index_.slack_qdot_min_start = si_index_.slack_q_max_start    + si_index_.slack_q_max_size;
            si_index_.slack_qdot_max_start = si_index_.slack_qdot_min_start + si_index_.slack_qdot_min_size;
            si_index_.slack_sing_start     = si_index_.slack_qdot_max_start + si_index_.slack_qdot_max_size;
            si_index_.slack_sel_col_start  = si_index_.slack_sing_start     + si_index_.slack_sing_size;
            si_index_.con_dyn_start        = 0;
            si_index_.con_q_min_start      = 0;
            si_index_.con_q_max_start      = si_index_.con_q_min_start    + si_index_.con_q_min_size;
            si_index_.con_qdot_min_start   = si_index_.con_q_max_start    + si_index_.con_q_max_size;
            si_index_.con_qdot_max_start   = si_index_.con_qdot_min_start + si_index_.con_qdot_min_size;
            si_index_.con_sing_start       = si_index_.con_qdot_max_start + si_index_.con_qdot_max_size;
            si_index_.con_sel_col_start    = si_index_.con_sing_start     + si_index_.con_sing_size;
    
            w_vel_damping_.setOnes(joint_dof_);
            w_acc_damping_.setOnes(joint_dof_);
            null_torque_.setZero(joint_dof_);
            w_null_torque_.setZero(joint_dof_);
        }

        void QPID::setTrackingWeight(const Vector6d w_tracking)
        {
            std::map<std::string, Vector6d> link_w_tracking;
            for(const auto& link_name : robot_data_->getLinkFrameVector())
            {
                link_w_tracking[link_name] = w_tracking;
            }
            link_w_tracking_ = link_w_tracking;
        }

        void QPID::setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_vel_damping,
                             const Eigen::Ref<const VectorXd>& w_acc_damping,
                             const Eigen::Ref<const VectorXd>& w_null_torque)
        {
            link_w_tracking_ = link_w_tracking;
            w_vel_damping_ = w_vel_damping;
            w_acc_damping_ = w_acc_damping;
            w_null_torque_ = w_null_torque;
        }

        void QPID::setWeight(const Vector6d w_tracking,
                             const Eigen::Ref<const VectorXd>& w_vel_damping,
                             const Eigen::Ref<const VectorXd>& w_acc_damping,
                             const Eigen::Ref<const VectorXd>& w_null_torque)
        {
            setTrackingWeight(w_tracking);
            w_vel_damping_ = w_vel_damping;
            w_acc_damping_ = w_acc_damping;
            w_null_torque_ = w_null_torque;
        }

        void QPID::setDesiredTaskAcc(const std::map<std::string, Vector6d> &link_xddot_desired)
        {
            link_xddot_desired_ = link_xddot_desired;
        }
    
        bool QPID::getOptJoint(Eigen::Ref<Eigen::VectorXd> opt_qddot, Eigen::Ref<Eigen::VectorXd> opt_torque, QP::TimeDuration &time_status)
        {
            if(opt_qddot.size() != joint_dof_ || opt_torque.size() != joint_dof_)
            {
                std::cerr << "Size of opt_qddot(" << opt_qddot.size() << ") or opt_torque(" << opt_torque.size() << ") are not same as joint_dof_(" << joint_dof_ << ")" << std::endl;
                time_status.setZero();
                return false;
            }
            MatrixXd sol;
            if(!solveQP(sol, time_status))
            {
                std::cerr << "QP ID failed to compute optimal joint torque." << std::endl;
                opt_qddot.setZero();
                opt_torque.setZero();
                time_status.setZero();
                return false;
            }
            else
            {
                opt_qddot = sol.block(si_index_.qddot_start,0,si_index_.qddot_size,1);
                opt_torque = sol.block(si_index_.torque_start,0,si_index_.torque_size,1);
                return true;
            }
        }
    
        void QPID::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);
            
            const VectorXd qdot = robot_data_->getJointVelocity();
            MatrixXd J_total(6 * link_xddot_desired_.size(), joint_dof_);
            J_total.setZero();
            int row = 0;

            // for task space acceleration tracking; accumulate J_total for null space computation
            for(const auto& [link_name, xddot_desired] : link_xddot_desired_)
            {
                const MatrixXd J_i = robot_data_->getJacobian(link_name);
                const MatrixXd J_i_dot = robot_data_->getJacobianTimeVariation(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,si_index_.qddot_size,si_index_.qddot_size) += 2.0 * J_i.transpose() * w_tracking.asDiagonal() * J_i;
                q_ds_.segment(si_index_.qddot_start,si_index_.qddot_size) += -2.0 * J_i.transpose() * w_tracking.asDiagonal() * (xddot_desired - J_i_dot * qdot);

                J_total.block(row, 0, 6, joint_dof_) = J_i;
                row += 6;
            }

            // for joint velocity/acceleration damping
            P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,si_index_.qddot_size,si_index_.qddot_size) += 2.0 * w_acc_damping_.asDiagonal().toDenseMatrix() + 
                                                                                                                  2.0 * dt_ * dt_ * w_vel_damping_.asDiagonal().toDenseMatrix();
            q_ds_.segment(si_index_.qddot_start,si_index_.qddot_size) += 2.0 * dt_ * w_vel_damping_.asDiagonal() * qdot;

            if(w_null_torque_.cwiseAbs().maxCoeff() > 0.0)
            {
                // Null space projector N = I - J^T*Λ*J*M^-1,  Λ = (J*M^-1*J^T)^+
                const MatrixXd M_inv = robot_data_->getMassMatrixInv();
                MatrixXd N = MatrixXd::Identity(joint_dof_, joint_dof_);

                if(J_total.rows() > 0)
                {
                    const MatrixXd J_total_T = J_total.transpose();
                    const MatrixXd M_task_total = DyrosMath::PinvSVD(J_total * M_inv * J_total_T);
                    const MatrixXd J_total_T_pinv = M_task_total * J_total * M_inv;
                    N -= J_total_T * J_total_T_pinv;
                }

                // for null space tracking: || N*(torque - null_torque) ||_W_null^2
                const MatrixXd W_null = w_null_torque_.asDiagonal();
                const MatrixXd NWN = N.transpose() * W_null * N;
                P_ds_.block(si_index_.torque_start, si_index_.torque_start, si_index_.torque_size, si_index_.torque_size) += 2.0 * NWN;
                q_ds_.segment(si_index_.torque_start, si_index_.torque_size) += -2.0 * NWN * null_torque_;
            }

            // for slack
            q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 100.0); 
            q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 100.0); 
            q_ds_.segment(si_index_.slack_qdot_min_start,si_index_.slack_qdot_min_size) = VectorXd::Constant(si_index_.slack_qdot_min_size, 100.0); 
            q_ds_.segment(si_index_.slack_qdot_max_start,si_index_.slack_qdot_max_size) = VectorXd::Constant(si_index_.slack_qdot_max_size, 100.0); 
            q_ds_(si_index_.slack_sing_start) = 100.0;
            q_ds_(si_index_.slack_sel_col_start) = 100.0;
            
        }
    
        void QPID::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);

            const auto torque_limit = robot_data_->getJointEffortLimit();
            
            // for torque limit
            l_bound_ds_.segment(si_index_.torque_start, si_index_.torque_size) = torque_limit.first;
            u_bound_ds_.segment(si_index_.torque_start, si_index_.torque_size) = torque_limit.second;

            // for slack
            l_bound_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size).setZero();
            l_bound_ds_.segment(si_index_.slack_qdot_min_start,si_index_.slack_qdot_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_qdot_max_start,si_index_.slack_qdot_max_size).setZero();
            l_bound_ds_.segment(si_index_.slack_sing_start,si_index_.slack_sing_size).setZero();
            l_bound_ds_.segment(si_index_.slack_sel_col_start,si_index_.slack_sel_col_size).setZero();
        }
    
        void QPID::setIneqConstraint()    
        {
            A_ineq_ds_.setZero(nineqc_, nx_);
            l_ineq_ds_.setConstant(nineqc_,-OSQP_INFTY);
            u_ineq_ds_.setConstant(nineqc_,OSQP_INFTY);
    
            const double alpha = 10.;
    
            // Manipulator Joint Angle Limit (CBF)
            const auto q_lim = robot_data_->getJointPositionLimit();
            const VectorXd q_min_raw = q_lim.first;
            const VectorXd q_max_raw = q_lim.second;
            const VectorXd q_min =
                (q_min_raw.array() < 0.0)
                    .select(q_min_raw.array() * 0.9, q_min_raw.array() * 1.9)
                    .matrix();
            const VectorXd q_max =
                (q_max_raw.array() > 0.0)
                    .select(q_max_raw.array() * 0.9, q_max_raw.array() * 1.9)
                    .matrix();
            // const auto q_lim = robot_data_->getJointPositionLimit();
            // const VectorXd q_min = q_lim.first;
            // const VectorXd q_max = q_lim.second;
    
            const VectorXd q = robot_data_->getJointPosition();
            const VectorXd qdot = robot_data_->getJointVelocity();
                
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qddot_start, si_index_.con_q_min_size, si_index_.qddot_size) = MatrixXd::Identity(si_index_.con_q_min_size,si_index_.qddot_size);
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, si_index_.con_q_min_size, si_index_.slack_q_min_size) = MatrixXd::Identity(si_index_.con_q_min_size,si_index_.slack_q_min_size);
            l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = -(alpha+alpha)*qdot - alpha*alpha*(q - q_min);
            
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qddot_start, si_index_.con_q_max_size, si_index_.qddot_size) = -MatrixXd::Identity(si_index_.con_q_max_size,si_index_.qddot_size);
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, si_index_.con_q_max_size, si_index_.slack_q_max_size) = MatrixXd::Identity(si_index_.con_q_max_size,si_index_.slack_q_max_size);
            l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_max_size) = +(alpha+alpha)*qdot - alpha*alpha*(q_max - q);
    
            // Manipulator Joint Velocity Limit (CBF)
            const auto qdot_lim = robot_data_->getJointVelocityLimit();
            const VectorXd qdot_min_raw = qdot_lim.first;
            const VectorXd qdot_max_raw = qdot_lim.second;
            const VectorXd qdot_min =
                (qdot_min_raw.array() < 0.0)
                    .select(qdot_min_raw.array() * 0.9, qdot_min_raw.array() * 1.9)
                    .matrix();
            const VectorXd qdot_max =
                (qdot_max_raw.array() > 0.0)
                    .select(qdot_max_raw.array() * 0.9, qdot_max_raw.array() * 1.9)
                    .matrix();

            // const auto qdot_lim = robot_data_->getJointVelocityLimit();
            // const VectorXd qdot_min = qdot_lim.first;
            // const VectorXd qdot_max = qdot_lim.second;
    
            A_ineq_ds_.block(si_index_.con_qdot_min_start, si_index_.qddot_start, si_index_.con_qdot_min_size, si_index_.qddot_size) = MatrixXd::Identity(si_index_.con_qdot_min_size, si_index_.qddot_size);
            A_ineq_ds_.block(si_index_.con_qdot_min_start, si_index_.slack_qdot_min_start, si_index_.con_qdot_min_size, si_index_.slack_qdot_min_size) = MatrixXd::Identity(si_index_.con_qdot_min_size, si_index_.slack_qdot_min_size);
            l_ineq_ds_.segment(si_index_.con_qdot_min_start, si_index_.con_qdot_min_size) = - alpha*(qdot - qdot_min);
            
            A_ineq_ds_.block(si_index_.con_qdot_max_start, si_index_.qddot_start, si_index_.con_qdot_max_size, si_index_.qddot_size) = -MatrixXd::Identity(si_index_.con_qdot_max_size, si_index_.qddot_size);
            A_ineq_ds_.block(si_index_.con_qdot_max_start, si_index_.slack_qdot_max_start, si_index_.con_qdot_max_size, si_index_.slack_qdot_max_size) = MatrixXd::Identity(si_index_.con_qdot_max_size, si_index_.slack_qdot_max_size);
            l_ineq_ds_.segment(si_index_.con_qdot_max_start, si_index_.con_qdot_max_size) = - alpha*(qdot_max - qdot);
    
            // singularity avoidance (CBF)
            // Manipulator::ManipulabilityResult mani_data = robot_data_->getManipulability(true, true, link_name_);
    
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.qddot_start, si_index_.con_sing_size, si_index_.qddot_size) = mani_data.grad.transpose();
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.slack_sing_start, si_index_.con_sing_size, si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = -mani_data.grad_dot.dot(qdot) - (alpha + alpha)*mani_data.grad.dot(qdot) - alpha*alpha*(mani_data.manipulability -0.01);
            
            // self collision avoidance (CBF)
            const Manipulator::MinDistResult min_dist_data = robot_data_->getMinDistance(true, true, false);
            const bool valid_col_grad =
                std::isfinite(min_dist_data.distance) &&
                min_dist_data.grad.size() == joint_dof_ &&
                min_dist_data.grad_dot.size() == joint_dof_ &&
                min_dist_data.grad.allFinite() &&
                min_dist_data.grad_dot.allFinite() &&
                min_dist_data.grad.squaredNorm() > 1e-12;

            if (valid_col_grad)
            {
                // Skip the CBF row when the distance Jacobian becomes undefined.
                if (!col_grad_initialized_) {
                    col_grad_filtered_     = min_dist_data.grad;
                    col_grad_dot_filtered_ = min_dist_data.grad_dot;
                    col_grad_initialized_  = true;
                } else {
                    col_grad_filtered_     = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                                           + col_grad_filter_alpha_ * min_dist_data.grad;
                    col_grad_dot_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_dot_filtered_
                                           + col_grad_filter_alpha_ * min_dist_data.grad_dot;
                }

                A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qddot_start, si_index_.con_sel_col_size, si_index_.qddot_size) = col_grad_filtered_.transpose();
                A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
                l_ineq_ds_(si_index_.con_sel_col_start) = -col_grad_dot_filtered_.dot(qdot) - (alpha + alpha)*col_grad_filtered_.dot(qdot) - alpha*alpha*(min_dist_data.distance - 0.01);
            }
            else
            {
                col_grad_initialized_ = false;
            }
        }
    
        void QPID::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);

            // for dynamics
            const MatrixXd M  = robot_data_->getMassMatrix();
            const MatrixXd nle = robot_data_->getNonlinearEffects();
    
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.qddot_start, si_index_.con_dyn_size, si_index_.qddot_size) = M;
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.torque_start, si_index_.con_dyn_size, si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size, si_index_.torque_size);
    
            b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -nle;
        }
    } // namespace Manipulator
} // namespace drc
