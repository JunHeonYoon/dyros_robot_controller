#include "dyros_robot_controller/manipulator/QP_ID.h"

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

        void QPID::setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_vel_damping,
                             const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_vel_damping_ = w_vel_damping;
            w_acc_damping_ = w_acc_damping;
        }
    
        void QPID::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);
            
            // for task space acceleration tracking
            const VectorXd qdot = robot_data_->getJointVelocity();
            for(const auto& [link_name, xddot_desired] : link_xddot_desired_)
            {
                const MatrixXd J_i = robot_data_->getJacobian(link_name);
                const MatrixXd J_i_dot = robot_data_->getJacobianTimeVariation(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,si_index_.qddot_size,si_index_.qddot_size) += 2.0 * J_i.transpose() * w_tracking.asDiagonal() * J_i;
                q_ds_.segment(si_index_.qddot_start,si_index_.qddot_size) += -2.0 * J_i.transpose() * w_tracking.asDiagonal() * (xddot_desired - J_i_dot * qdot);
            }

            // for joint velocity/acceleration damping
            P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,si_index_.qddot_size,si_index_.qddot_size) += 2.0 * w_acc_damping_.asDiagonal().toDenseMatrix();// + 
                                                                                                                //   2.0 * dt_ * dt_ * w_vel_damping_.asDiagonal().toDenseMatrix();
            q_ds_.segment(si_index_.qddot_start,si_index_.qddot_size) += 2.0 * dt_ * qdot;

            // for slack
            q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 1000.0); 
            q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 1000.0); 
            q_ds_.segment(si_index_.slack_qdot_min_start,si_index_.slack_qdot_min_size) = VectorXd::Constant(si_index_.slack_qdot_min_size, 1000.0); 
            q_ds_.segment(si_index_.slack_qdot_max_start,si_index_.slack_qdot_max_size) = VectorXd::Constant(si_index_.slack_qdot_max_size, 1000.0); 
            q_ds_(si_index_.slack_sing_start) = 1000.0;
            q_ds_(si_index_.slack_sel_col_start) = 1000.0;
            
        }
    
        void QPID::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);

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
    
            const double alpha = 50.;
    
            // Manipulator Joint Angle Limit (CBF)
            const auto q_lim = robot_data_->getJointPositionLimit();
            const VectorXd q_min = q_lim.first;
            const VectorXd q_max = q_lim.second;
    
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
            const VectorXd qdot_min = qdot_lim.first;
            const VectorXd qdot_max = qdot_lim.second;
    
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
            
            A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qddot_start, si_index_.con_sel_col_size, si_index_.qddot_size) = min_dist_data.grad.transpose();
            A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
            l_ineq_ds_(si_index_.con_sel_col_start) = -min_dist_data.grad_dot.dot(qdot) - (alpha + alpha)*min_dist_data.grad.dot(qdot) - alpha*alpha*(min_dist_data.distance -0.01);
        }
    
        void QPID::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);

            // for dynamics
            const MatrixXd M  = robot_data_->getMassMatrix();
            const MatrixXd g = robot_data_->getGravity();
    
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.qddot_start, si_index_.con_dyn_size, si_index_.qddot_size) = M;
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.torque_start, si_index_.con_dyn_size, si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size, si_index_.torque_size);
    
            b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -g;
        }
    } // namespace Manipulator
} // namespace drc
