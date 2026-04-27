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

#include "dyros_robot_controller/mobile_manipulator/QP_IK.h"
#include <cmath>

namespace drc
{
    namespace MobileManipulator
    {
        QPIK::QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt)
        : QP::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            actuator_dof_ = robot_data_->getActuatorDof();
            mani_dof_ = robot_data_->getManipulatorDof();
            mobi_dof_ = robot_data_->getMobileDof();
    
            si_index_.eta_size              = actuator_dof_;
            si_index_.slack_q_mani_min_size = mani_dof_;
            si_index_.slack_q_mani_max_size = mani_dof_;
            si_index_.slack_sing_size       = 1;
            si_index_.slack_sel_col_size    = 1;
            si_index_.con_q_mani_min_size   = mani_dof_;
            si_index_.con_q_mani_max_size   = mani_dof_;
            si_index_.con_sing_size         = 1;
            si_index_.con_sel_col_size      = 1;
            si_index_.con_base_vel_size     = 3;
            si_index_.con_base_acc_size     = 3;
    
            const int nx = si_index_.eta_size +
                           si_index_.slack_q_mani_min_size +
                           si_index_.slack_q_mani_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_mani_min_size +
                              si_index_.con_q_mani_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size +
                              si_index_.con_base_vel_size +
                              si_index_.con_base_acc_size;
            const int neq = 0;
    
            QPBase::setQPsize(nx, nbound, nineq, neq);
    
            si_index_.eta_start               = 0;
            si_index_.slack_q_mani_min_start  = si_index_.eta_start              + si_index_.eta_size;
            si_index_.slack_q_mani_max_start  = si_index_.slack_q_mani_min_start + si_index_.slack_q_mani_min_size;
            si_index_.slack_sing_start        = si_index_.slack_q_mani_max_start + si_index_.slack_q_mani_max_size;
            si_index_.slack_sel_col_start     = si_index_.slack_sing_start       + si_index_.slack_sing_size;
            si_index_.con_q_mani_min_start    = 0;
            si_index_.con_q_mani_max_start    = si_index_.con_q_mani_min_start + si_index_.con_q_mani_min_size;
            si_index_.con_sing_start          = si_index_.con_q_mani_max_start + si_index_.con_q_mani_max_size;
            si_index_.con_sel_col_start       = si_index_.con_sing_start       + si_index_.con_sing_size;
            si_index_.con_base_vel_start      = si_index_.con_sel_col_start    + si_index_.con_sel_col_size;
            si_index_.con_base_acc_start      = si_index_.con_base_vel_start   + si_index_.con_base_vel_size;

            null_eta_desired_.setZero(actuator_dof_);
            w_null_joint_vel_.setOnes(actuator_dof_);
            w_mani_acc_damping_.setOnes(mani_dof_);
            w_base_acc_damping_.setOnes();
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

        void QPIK::setWeight(const std::map<std::string, Vector6d>& link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_null_vel,
                             const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                             const Eigen::Vector3d& w_base_acc_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_null_joint_vel_ = w_null_vel;
            w_mani_acc_damping_ = w_mani_acc_damping;
            w_base_acc_damping_ = w_base_acc_damping;
        }

        void QPIK::setWeight(const Vector6d& w_tracking,
                             const Eigen::Ref<const VectorXd>& w_null_vel,
                             const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                             const Eigen::Vector3d& w_base_acc_damping)
        {
            setTrackingWeight(w_tracking);
            w_null_joint_vel_ = w_null_vel;
            w_mani_acc_damping_ = w_mani_acc_damping;
            w_base_acc_damping_ = w_base_acc_damping;
        }

        void QPIK::setDesiredTaskVel(const std::map<std::string, Vector6d> &link_xdot_desired)
        {
            link_xdot_desired_ = link_xdot_desired;
        }

        bool QPIK::getOptJointVel(Eigen::Ref<Eigen::VectorXd> opt_eta, QP::TimeDuration &time_status)
        {
            if(opt_eta.size() != actuator_dof_)
            {
                std::cerr << "Size of opt_eta(" << opt_eta.size() << ") is not same as actuator_dof_(" << actuator_dof_ << ")" << std::endl;
                time_status.setZero();
                return false;
            }
            MatrixXd sol;
            if(!solveQP(sol, time_status))
            {
                std::cerr << "QP IK failed to compute optimal joint velocity." << std::endl;
                opt_eta.setZero();
                time_status.setZero();
                return false;
            }
            else
            {
                opt_eta = sol.block(si_index_.eta_start,0,si_index_.eta_size,1);
                return true;
            }
        }
        
        void QPIK::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);

            // for task space velocity tracking; accumulate J_total_tilda for null space computation
            MatrixXd J_total_tilda(6 * link_xdot_desired_.size(), actuator_dof_);
            J_total_tilda.setZero();
            int row = 0;

            for(const auto& [link_name, xdot_desired] : link_xdot_desired_)
            {
                const MatrixXd J_i_tilda = robot_data_->getJacobianActuated(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.eta_start,si_index_.eta_start,si_index_.eta_size,si_index_.eta_size) += 2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * J_i_tilda;
                q_ds_.segment(si_index_.eta_start,si_index_.eta_size) += -2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * xdot_desired;

                J_total_tilda.block(row, 0, 6, actuator_dof_) = J_i_tilda;
                row += 6;
            }

            const int mani_start = robot_data_->getActuatorIndex().mani_start;
            const int mobi_start = robot_data_->getActuatorIndex().mobi_start;
            const double dt_sq_inv = 1.0 / (dt_ * dt_);

            // Null space projector in actuated space: N_tilda = I - J_tilda†J_tilda
            const MatrixXd N_tilda = MatrixXd::Identity(actuator_dof_, actuator_dof_)
                                   - DyrosMath::PinvSVD(J_total_tilda) * J_total_tilda;

            // for null space tracking (mani + mobile unified): || N_tilda*(eta - null_eta_desired) ||²_{W_null}
            const MatrixXd NWN = N_tilda.transpose() * w_null_joint_vel_.asDiagonal() * N_tilda;
            P_ds_.block(si_index_.eta_start, si_index_.eta_start, actuator_dof_, actuator_dof_) += 2.0 * NWN;
            q_ds_.segment(si_index_.eta_start, actuator_dof_) += -2.0 * NWN * null_eta_desired_;

            // for manipulator joint acceleration damping: || (eta_mani - eta_mani_now) / dt ||_W3^2
            P_ds_.block(si_index_.eta_start+mani_start,
                        si_index_.eta_start+mani_start,
                        mani_dof_,
                        mani_dof_) += 2.0 * dt_sq_inv * w_mani_acc_damping_.asDiagonal();
            q_ds_.segment(si_index_.eta_start+mani_start, mani_dof_) +=
                -2.0 * dt_sq_inv * w_mani_acc_damping_.asDiagonal() * robot_data_->getManiJointVelocity();

            // for mobile base acceleration damping: || (v_base - v_base_now) / dt ||_W5^2
            const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
            const MatrixXd J_mobile_T = J_mobile.transpose();
            const Matrix3d w_base_acc = w_base_acc_damping_.asDiagonal();
            const Vector3d base_vel_now = robot_data_->getMobileBaseVel().head<3>();
            P_ds_.block(si_index_.eta_start+mobi_start,
                        si_index_.eta_start+mobi_start,
                        mobi_dof_,
                        mobi_dof_) += 2.0 * dt_sq_inv * J_mobile_T * w_base_acc * J_mobile;
            q_ds_.segment(si_index_.eta_start+mobi_start,
                          mobi_dof_) += -2.0 * dt_sq_inv * J_mobile_T * w_base_acc * base_vel_now;

            // for slack
            q_ds_.segment(si_index_.slack_q_mani_min_start,si_index_.slack_q_mani_min_size) = VectorXd::Constant(si_index_.slack_q_mani_min_size, 1000.0);
            q_ds_.segment(si_index_.slack_q_mani_max_start,si_index_.slack_q_mani_max_size) = VectorXd::Constant(si_index_.slack_q_mani_max_size, 1000.0);
            q_ds_(si_index_.slack_sing_start) = 1000.0;
            q_ds_(si_index_.slack_sel_col_start) = 1000.0;
        }
    
        void QPIK::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);
            
            // Manipulator Joint Velocity Limit
            const auto qdot_lim = robot_data_->getJointVelocityLimit();
            const VectorXd qdot_mani_min_raw = qdot_lim.first.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd qdot_mani_max_raw = qdot_lim.second.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd qdot_mani_min =
                (qdot_mani_min_raw.array() < 0.0)
                    .select(qdot_mani_min_raw.array() * 0.9, qdot_mani_min_raw.array() * 1.9)
                    .matrix();
            const VectorXd qdot_mani_max =
                (qdot_mani_max_raw.array() > 0.0)
                    .select(qdot_mani_max_raw.array() * 0.9, qdot_mani_max_raw.array() * 1.9)
                    .matrix();

            l_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                                mani_dof_) = qdot_mani_min;
                                // mani_dof_) = robot_data_->getJointVelocityLimit().first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            u_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                                mani_dof_) = qdot_mani_max;
                                // mani_dof_) = robot_data_->getJointVelocityLimit().second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);

            // for slack
            l_bound_ds_.segment(si_index_.slack_q_mani_min_start,si_index_.slack_q_mani_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_mani_max_start,si_index_.slack_q_mani_max_size).setZero();
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
            const VectorXd q_mani_min_raw = q_lim.first.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd q_mani_max_raw = q_lim.second.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd q_mani_min =
                (q_mani_min_raw.array() < 0.0)
                    .select(q_mani_min_raw.array() * 0.9, q_mani_min_raw.array() * 1.9)
                    .matrix();
            const VectorXd q_mani_max =
                (q_mani_max_raw.array() > 0.0)
                    .select(q_mani_max_raw.array() * 0.9, q_mani_max_raw.array() * 1.9)
                    .matrix();
            // const VectorXd q_mani_min = robot_data_->getJointPositionLimit().first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            // const VectorXd q_mani_max = robot_data_->getJointPositionLimit().second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
           
            const VectorXd q_mani = robot_data_->getJointPosition().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            
                
            A_ineq_ds_.block(si_index_.con_q_mani_min_start,
                             si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                             si_index_.con_q_mani_min_size, 
                             mani_dof_) = MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_q_mani_min_start,
                             si_index_.slack_q_mani_min_start,
                             si_index_.con_q_mani_min_size, 
                             si_index_.slack_q_mani_min_size) = MatrixXd::Identity(si_index_.con_q_mani_min_size, si_index_.slack_q_mani_min_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_min_start, 
                               si_index_.con_q_mani_min_size) = - alpha*(q_mani - q_mani_min);
    
            A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                             si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                             si_index_.con_q_mani_max_size, 
                             mani_dof_) = -MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                             si_index_.slack_q_mani_max_start,
                             si_index_.con_q_mani_max_size, 
                             si_index_.slack_q_mani_max_size) = MatrixXd::Identity(si_index_.con_q_mani_max_size, si_index_.slack_q_mani_max_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_max_start, 
                               si_index_.con_q_mani_max_size) = - alpha*(q_mani_max - q_mani);
    
            // singularity avoidance (CBF)
            // Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);
    
            // A_ineq_ds_.block(si_index_.con_sing_start, 
            //                  si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
            //                  si_index_.con_sing_size, 
            //                  mani_dof_) = mani_result.grad.transpose();
            // A_ineq_ds_.block(si_index_.con_sing_start, 
            //                  si_index_.slack_sing_start,
            //                  si_index_.con_sing_size, 
            //                  si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);
    
            // self collision avoidance (CBF)
            const Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
            const int mani_start = robot_data_->getJointIndex().mani_start;
            const bool valid_col_grad =
                std::isfinite(min_dist_res.distance) &&
                min_dist_res.grad.size() >= mani_start + mani_dof_;

            if (valid_col_grad)
            {
                const VectorXd col_grad = min_dist_res.grad.segment(mani_start, mani_dof_);

                if (col_grad.allFinite() && col_grad.squaredNorm() > 1e-12)
                {
                    // Skip the CBF row when the distance Jacobian becomes undefined.
                    if (!col_grad_initialized_) {
                        col_grad_filtered_    = col_grad;
                        col_grad_initialized_ = true;
                    } else {
                        col_grad_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                                           + col_grad_filter_alpha_ * col_grad;
                    }

                    A_ineq_ds_.block(si_index_.con_sel_col_start,
                                     si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                                     si_index_.con_sel_col_size,
                                     mani_dof_) = col_grad_filtered_.transpose();
                    A_ineq_ds_.block(si_index_.con_sel_col_start,
                                     si_index_.slack_sel_col_start,
                                     si_index_.con_sel_col_size,
                                     si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
                    l_ineq_ds_(si_index_.con_sel_col_start) = -alpha * (min_dist_res.distance - 0.01);
                }
                else
                {
                    col_grad_initialized_ = false;
                }
            }
            else
            {
                col_grad_initialized_ = false;
            }

            // Mobile base velocity limit
            const auto& param = robot_data_->getKineParam();
            const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
            const int mobi_start = robot_data_->getActuatorIndex().mobi_start;

            A_ineq_ds_.block(si_index_.con_base_vel_start,
                             si_index_.eta_start + mobi_start,
                             si_index_.con_base_vel_size,
                             mobi_dof_) = J_mobile;
            Vector3d vel_limit;
            vel_limit << param.max_lin_speed, param.max_lin_speed, param.max_ang_speed;
            l_ineq_ds_.segment(si_index_.con_base_vel_start, si_index_.con_base_vel_size) = -vel_limit;
            u_ineq_ds_.segment(si_index_.con_base_vel_start, si_index_.con_base_vel_size) = vel_limit;
            
            // // Mobile base acceleration Limit
            // A_ineq_ds_.block(si_index_.con_base_acc_start,
            //                  si_index_.eta_start + mobi_start,
            //                  si_index_.con_base_acc_size,
            //                  mobi_dof_) = J_mobile;
            // const Vector3d base_vel = robot_data_->getMobileBaseVel();
            // Vector3d acc_limit;
            // acc_limit << param.max_lin_acc, param.max_lin_acc, param.max_ang_acc;
            // const double inv_alpha = 1.0 / alpha;
            // const Vector3d acc_delta = acc_limit * inv_alpha;
            // l_ineq_ds_.segment(si_index_.con_base_acc_start, si_index_.con_base_acc_size) = base_vel - acc_delta;
            // u_ineq_ds_.segment(si_index_.con_base_acc_start, si_index_.con_base_acc_size) = base_vel + acc_delta;
        }
    
        void QPIK::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);
        }
    } // namespace MobileManipulator
} // namespace drc
