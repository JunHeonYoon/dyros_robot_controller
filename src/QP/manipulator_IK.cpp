#include "QP/manipulator_IK.h"

namespace QP
{
    ManipulatorIK::ManipulatorIK(std::shared_ptr<RobotData::Manipulator::ManipulatorBase> robot_data)
    : QPBase(), robot_data_(robot_data)
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
        si_index_.con_q_min_start   = 0;
        si_index_.con_q_max_start   = si_index_.con_q_min_start + si_index_.con_q_min_size;
        si_index_.con_sing_start    = si_index_.con_q_max_start + si_index_.con_q_max_size;
        si_index_.con_sel_col_start = si_index_.con_sing_start  + si_index_.con_sing_size;
    }

    void ManipulatorIK::setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name)
    {
        xdot_desired_ = xdot_desired;
        link_name_ = link_name;
    }

    bool ManipulatorIK::getOptJointVel(VectorXd &opt_qdot, TimeDuration &time_status)
    {
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

    void ManipulatorIK::setCost()
    {
        /*
              min     || x_dot_des - J*q_dot ||_2^2 + W * || q_dot ||_2^2 
              qdot

        =>    min     1/2 * qdot.T * (2*J.T*J + W) * qdot + (-2*J.T*x_dot_des).T * qdot
              qdot
        */
        MatrixXd J = robot_data_->getJacobian(link_name_);

        P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) = 2.0 * J.transpose() * J;
        P_ds_.block(si_index_.qdot_start,si_index_.qdot_start,si_index_.qdot_size,si_index_.qdot_size) += 1.0*MatrixXd::Identity(si_index_.qdot_size,si_index_.qdot_size);
        q_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = -2.0 * J.transpose() * xdot_desired_;
        q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 1000.0);
        q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 1000.0);
        q_ds_(si_index_.slack_sing_start) = 1000.0;
        q_ds_(si_index_.slack_sel_col_start) = 1000.0;
    }

    void ManipulatorIK::setBoundConstraint()    
    {
        l_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().first;
        l_bound_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size).setZero();
        l_bound_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size).setZero();
        l_bound_ds_(si_index_.slack_sing_start) = 0.0;
        l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
        u_bound_ds_.segment(si_index_.qdot_start,si_index_.qdot_size) = robot_data_->getJointVelocityLimit().second;
    }

    void ManipulatorIK::setIneqConstraint()    
    {
        double alpha = 50.;

        // Manipulator Joint Angle Limit (CBF)
        VectorXd q_min = robot_data_->getJointPositionLimit().first;
        VectorXd q_max = robot_data_->getJointPositionLimit().second;
       
        VectorXd q = robot_data_->getJointPosition();
            
        A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qdot_start, si_index_.con_q_min_size, si_index_.qdot_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.qdot_size);
        A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, si_index_.con_q_min_size, si_index_.slack_q_min_size) = MatrixXd::Identity(si_index_.con_q_min_size, si_index_.slack_q_min_size);
        l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = - alpha*(q - q_min);
        
        A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qdot_start, si_index_.con_q_max_size, si_index_.qdot_size) = -MatrixXd::Identity(si_index_.con_q_max_size, si_index_.qdot_size);
        A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, si_index_.con_q_max_size, si_index_.slack_q_max_size) = MatrixXd::Identity(si_index_.con_q_max_size, si_index_.slack_q_max_size);
        l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_min_size) = - alpha*(q_max - q);

        // singularity avoidance (CBF)
        RobotData::Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);

        A_ineq_ds_.block(si_index_.con_sing_start, si_index_.qdot_start, si_index_.con_sing_size, si_index_.qdot_size) = mani_result.grad.transpose();
        A_ineq_ds_.block(si_index_.con_sing_start, si_index_.slack_sing_start, si_index_.con_sing_size, si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
        l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);

        // self collision avoidance (CBF)
        VectorXd min_dist_grad;
        RobotData::Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);

        A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qdot_start, si_index_.con_sel_col_size, si_index_.qdot_size) = min_dist_res.grad.transpose();
        A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
        l_ineq_ds_(si_index_.con_sel_col_start) = - alpha*(min_dist_res.distance -0.05);
    }

    void ManipulatorIK::setEqConstraint()    
    {
    }
}