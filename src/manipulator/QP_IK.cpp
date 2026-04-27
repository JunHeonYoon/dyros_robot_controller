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

#include "dyros_robot_controller/manipulator/QP_IK.h"
#include <cmath>

namespace drc
{
    namespace Manipulator
    {
        QPIK::QPIK(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt)
        : QP::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            joint_dof_ = robot_data_->getDof();
    
            si_index_.qdot_size            = joint_dof_;
            si_index_.slack_q_min_size     = joint_dof_;
            si_index_.slack_q_max_size     = joint_dof_;
            si_index_.slack_sing_size      = 1;
            si_index_.slack_sel_col_size   = 1;
            si_index_.con_q_min_size       = joint_dof_;
            si_index_.con_q_max_size       = joint_dof_;
            si_index_.con_sing_size        = 1;
            si_index_.con_sel_col_size     = 1;
            
            const int nx = si_index_.qdot_size +
                           si_index_.slack_q_min_size +
                           si_index_.slack_q_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_min_size +
                              si_index_.con_q_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size;
            const int neq = 0;
            
            QPBase::setQPsize(nx, nbound, nineq, neq);
            
            si_index_.qdot_start          = 0;
            si_index_.slack_q_min_start   = si_index_.qdot_start        + si_index_.qdot_size;
            si_index_.slack_q_max_start   = si_index_.slack_q_min_start + si_index_.slack_q_min_size;;
            si_index_.slack_sing_start    = si_index_.slack_q_max_start + si_index_.slack_q_max_size;;
            si_index_.slack_sel_col_start = si_index_.slack_sing_start  + si_index_.slack_sing_size;;
            si_index_.con_q_min_start     = 0;
            si_index_.con_q_max_start     = si_index_.con_q_min_start + si_index_.con_q_min_size;
            si_index_.con_sing_start      = si_index_.con_q_max_start + si_index_.con_q_max_size;
            si_index_.con_sel_col_start   = si_index_.con_sing_start  + si_index_.con_sing_size;

            qdot_desired_.setZero(joint_dof_);
            w_joint_vel_.setOnes(joint_dof_);
            w_acc_damping_.setOnes(joint_dof_);

        }

        void QPIK::setTrackingWeight(const Vector6d w_tracking)
        {
            std::map<std::string, Vector6d> link_w_tracking;
            for(const auto& link_name : robot_data_->getLinkFrameVector())
            {
                link_w_tracking[link_name] = w_tracking;
            }
            link_w_tracking_ = link_w_tracking;
        }

        void QPIK::setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_vel_damping,
                             const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_joint_vel_ = w_vel_damping;
            w_acc_damping_ = w_acc_damping;
        }

        void QPIK::setWeight(const Vector6d w_tracking,
                             const Eigen::Ref<const VectorXd>& w_vel_damping,
                             const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            setTrackingWeight(w_tracking);
            w_joint_vel_ = w_vel_damping;
            w_acc_damping_ = w_acc_damping;
        }

        void QPIK::setDesiredTaskVel(const std::map<std::string, Vector6d> &link_xdot_desired)
        {
            link_xdot_desired_ = link_xdot_desired;
        }
    
        bool QPIK::getOptJointVel(Eigen::Ref<Eigen::VectorXd> opt_qdot, QP::TimeDuration &time_status)
        {
            if(opt_qdot.size() != joint_dof_)
            {
                std::cerr << "Size of opt_qdot(" << opt_qdot.size() << ") is not same as joint_dof_(" << joint_dof_ << ")" << std::endl;
                time_status.setZero();
                return false;
            }
            MatrixXd sol;
            if(!solveQP(sol, time_status))
            {
                std::cerr << "QP IK failed to compute optimal joint velocity." << std::endl;
                opt_qdot.setZero();
                time_status.setZero();
                return false;
            }
            else
            {
                opt_qdot = sol.block(si_index_.qdot_start,0,si_index_.qdot_size,1);
                return true;
            }
        }
    
        void QPIK::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);
            const VectorXd qdot_now = robot_data_->getJointVelocity();

            // for task space velocity tracking
            for(const auto& [link_name, xdot_desired] : link_xdot_desired_)
            {
                MatrixXd J_i = robot_data_->getJacobian(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) += 2.0 * J_i.transpose() * w_tracking.asDiagonal() * J_i;
                q_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) += -2.0 * J_i.transpose() * w_tracking.asDiagonal() * xdot_desired;
            }
            
            // for joint velocity tracking: || q_dot - qdot_desired ||_W2^2
            P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) += 2.0 * w_joint_vel_.asDiagonal();
            q_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) += -2.0 * w_joint_vel_.asDiagonal() * qdot_desired_;

            // for joint acceleration damping: || (q_dot - q_dot_now) / dt ||_W3^2
            const double dt_sq_inv = 1.0 / (dt_ * dt_);
            P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) += 2.0 * dt_sq_inv * w_acc_damping_.asDiagonal();
            q_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) += -2.0 * dt_sq_inv * w_acc_damping_.asDiagonal() * qdot_now;
            
            // for slack
            q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 100.0);
            q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 100.0);
            q_ds_(si_index_.slack_sing_start) = 100.0;
            q_ds_(si_index_.slack_sel_col_start) = 100.0;
        }
    
        void QPIK::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);

            // Manipulator Joint Velocity Limit
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

            l_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = qdot_min;
            u_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = qdot_max;
            
            // l_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().first;
            // u_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().second;

            // for slack
            l_bound_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size).setZero();
            l_bound_ds_(si_index_.slack_sing_start) = 0.0;
            l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
        }
    
        void QPIK::setIneqConstraint()    
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

            // const VectorXd q_min = robot_data_->getJointPositionLimit().first;
            // const VectorXd q_max = robot_data_->getJointPositionLimit().second;
           
            const VectorXd q = robot_data_->getJointPosition();
                
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qdot_start, si_index_.con_q_min_size, si_index_.qdot_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, si_index_.con_q_min_size, si_index_.slack_q_min_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.slack_q_min_size);
            l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = - alpha*(q - q_min);
            
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qdot_start, si_index_.con_q_max_size, si_index_.qdot_size) = -MatrixXd::Identity(si_index_.con_q_max_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, si_index_.con_q_max_size, si_index_.slack_q_max_size) = MatrixXd::Identity(si_index_.con_q_max_size, si_index_.slack_q_max_size);
            l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_max_size) = - alpha*(q_max - q);
    
            // singularity avoidance (CBF)
            // Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);
    
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.qdot_start, si_index_.con_sing_size, si_index_.qdot_size) = mani_result.grad.transpose();
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.slack_sing_start, si_index_.con_sing_size, si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);
    
            // self collision avoidance (CBF)
            const Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
            const bool valid_col_grad =
                std::isfinite(min_dist_res.distance) &&
                min_dist_res.grad.size() == joint_dof_ &&
                min_dist_res.grad.allFinite() &&
                min_dist_res.grad.squaredNorm() > 1e-12;

            if (valid_col_grad)
            {
                // Skip the CBF row when the distance Jacobian becomes undefined.
                if (!col_grad_initialized_) {
                    col_grad_filtered_    = min_dist_res.grad;
                    col_grad_initialized_ = true;
                } else {
                    col_grad_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                                       + col_grad_filter_alpha_ * min_dist_res.grad;
                }

                A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qdot_start, si_index_.con_sel_col_size, si_index_.qdot_size) = col_grad_filtered_.transpose();
                A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
                l_ineq_ds_(si_index_.con_sel_col_start) = -alpha * (min_dist_res.distance - 0.01);
            }
            else
            {
                col_grad_initialized_ = false;
            }
        }
    
        void QPIK::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);
        }
    } // namespace Manipulator
} // namespace drc
