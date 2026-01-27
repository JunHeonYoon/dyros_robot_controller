#include "dyros_robot_controller/manipulator/QP_IK.h"

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

            w_damping_.setOnes(joint_dof_);

        }
    
        void QPIK::setDesiredTaskVel(const std::map<std::string, Vector6d> &link_xdot_desired)
        {
            link_xdot_desired_ = link_xdot_desired;
        }
    
        bool QPIK::getOptJointVel(Eigen::Ref<Eigen::VectorXd> opt_qdot, QP::TimeDuration &time_status)
        {
            if(opt_qdot.size() != joint_dof_)
            {
                time_status.setZero();
                return false;
            }
            MatrixXd sol;
            if(!solveQP(sol, time_status))
            {
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

        void QPIK::setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_damping_ = w_damping;
        }
    
        void QPIK::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);

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
            
            // for joint velocity damping
            P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) += 2.0 * w_damping_.asDiagonal();
            
            // for slack
            q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 1000.0);
            q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 1000.0);
            q_ds_(si_index_.slack_sing_start) = 1000.0;
            q_ds_(si_index_.slack_sel_col_start) = 1000.0;
        }
    
        void QPIK::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);

            // Manipulator Joint Velocity Limit
            l_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().first;
            u_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().second;

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

            const double alpha = 50.;
    
            // Manipulator Joint Angle Limit (CBF)
            const VectorXd q_min = robot_data_->getJointPositionLimit().first;
            const VectorXd q_max = robot_data_->getJointPositionLimit().second;
           
            const VectorXd q = robot_data_->getJointPosition();
                
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qdot_start, si_index_.con_q_min_size, si_index_.qdot_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, si_index_.con_q_min_size, si_index_.slack_q_min_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.slack_q_min_size);
            l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = - alpha*(q - q_min);
            
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qdot_start, si_index_.con_q_max_size, si_index_.qdot_size) = -MatrixXd::Identity(si_index_.con_q_max_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, si_index_.con_q_max_size, si_index_.slack_q_max_size) = MatrixXd::Identity(si_index_.con_q_max_size, si_index_.slack_q_max_size);
            l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_min_size) = - alpha*(q_max - q);
    
            // singularity avoidance (CBF)
            // Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);
    
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.qdot_start, si_index_.con_sing_size, si_index_.qdot_size) = mani_result.grad.transpose();
            // A_ineq_ds_.block(si_index_.con_sing_start, si_index_.slack_sing_start, si_index_.con_sing_size, si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);
    
            // self collision avoidance (CBF)
            const Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
    
            A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qdot_start, si_index_.con_sel_col_size, si_index_.qdot_size) = min_dist_res.grad.transpose();
            A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
            l_ineq_ds_(si_index_.con_sel_col_start) = - alpha*(min_dist_res.distance -0.01);
        }
    
        void QPIK::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);
        }
    } // namespace Manipulator
} // namespace drc
