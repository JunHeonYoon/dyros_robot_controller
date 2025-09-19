#include "QP/manipulator_ID.h"

namespace QP
{
    ManipulatorID::ManipulatorID(std::shared_ptr<RobotData::Manipulator::ManipulatorBase> robot_data)
    : QPBase(), robot_data_(robot_data)
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

        si_index_.con_dyn_size = joint_dof_;

        si_index_.con_q_min_size    = joint_dof_;
        si_index_.con_q_max_size    = joint_dof_;
        si_index_.con_qdot_min_size = joint_dof_;
        si_index_.con_qdot_max_size = joint_dof_;
        si_index_.con_sing_size     = 1;
        si_index_.con_sel_col_size  = 1;

        si_index_.qddot_start          = 0;
        si_index_.torque_start         = si_index_.qddot_start          + si_index_.qddot_size;
        si_index_.slack_q_min_start    = si_index_.torque_start         + si_index_.torque_size;
        si_index_.slack_q_max_start    = si_index_.slack_q_min_start    + si_index_.slack_q_min_size;
        si_index_.slack_qdot_min_start = si_index_.slack_q_max_start    + si_index_.slack_q_max_size;
        si_index_.slack_qdot_max_start = si_index_.slack_qdot_min_start + si_index_.slack_qdot_min_size;
        si_index_.slack_sing_start     = si_index_.slack_qdot_max_start + si_index_.slack_qdot_max_size;
        si_index_.slack_sel_col_start  = si_index_.slack_sing_start     + si_index_.slack_sing_size;

        si_index_.con_dyn_start = 0;

        si_index_.con_q_min_start    = 0;
        si_index_.con_q_max_start    = si_index_.con_q_min_start    + si_index_.con_q_min_size;
        si_index_.con_qdot_min_start = si_index_.con_q_max_start    + si_index_.con_q_max_size;
        si_index_.con_qdot_max_start = si_index_.con_qdot_min_start + si_index_.con_qdot_min_size;
        si_index_.con_sing_start     = si_index_.con_qdot_max_start + si_index_.con_qdot_max_size;
        si_index_.con_sel_col_start  = si_index_.con_sing_start     + si_index_.con_sing_size;

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
    }

    void ManipulatorID::setDesiredTaskAcc(const VectorXd &xddot_desired, const std::string &link_name)
    {
        xddot_desired_ = xddot_desired;
        link_name_ = link_name;
    }

    bool ManipulatorID::getOptJoint(VectorXd &opt_qddot, VectorXd &opt_torque, TimeDuration &time_status)
    {
        MatrixXd sol;
        if(!solveQP(sol, time_status))
        {
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

    void ManipulatorID::setCost()
    {
        /*
             min       || x_ddot_des - J*qddot - Jdot * qdot ||_2^2
        qddot, torque

        =>      min        1/2 * [ qddot  ]^T * [2 * J.T * J  0] * [ qddot  ] + [-2 * J.T * (x_ddot_des - Jdot * qdot)].T * [ qddot  ]
          [qddot, torque]        [ torque ]     [     0       0]   [ torque ]   [                 0                   ]     [ torque ]
        */
        MatrixXd J = robot_data_->getJacobian(link_name_);
        MatrixXd Jdot = robot_data_->getJacobianTimeVariation(link_name_);
        VectorXd qdot = robot_data_->getJointVelocity();

        P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,si_index_.qddot_size,si_index_.qddot_size) = 2.0 * J.transpose() * J;
        q_ds_.segment(si_index_.qddot_start,si_index_.qddot_size) = -2.0 * J.transpose() * (xddot_desired_ - Jdot * qdot);
        q_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size) = VectorXd::Constant(si_index_.slack_q_min_size, 1000.0); 
        q_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size) = VectorXd::Constant(si_index_.slack_q_max_size, 1000.0); 
        q_ds_.segment(si_index_.slack_qdot_min_start,si_index_.slack_qdot_min_size) = VectorXd::Constant(si_index_.slack_qdot_min_size, 1000.0); 
        q_ds_.segment(si_index_.slack_qdot_max_start,si_index_.slack_qdot_max_size) = VectorXd::Constant(si_index_.slack_qdot_max_size, 1000.0); 
        q_ds_(si_index_.slack_sing_start) = 1000.0;
        q_ds_(si_index_.slack_sel_col_start) = 1000.0;
        
    //    P_ds_.block(si_index_.qddot_start,si_index_.qddot_start,joint_dof_,joint_dof_) += 0.1*MatrixXd::Identity(joint_dof_,joint_dof_);
    }

    void ManipulatorID::setBoundConstraint()    
    {
        l_bound_ds_.segment(si_index_.slack_q_min_start,si_index_.slack_q_min_size).setZero();
        l_bound_ds_.segment(si_index_.slack_q_max_start,si_index_.slack_q_max_size).setZero();
        l_bound_ds_.segment(si_index_.slack_qdot_min_start,si_index_.slack_qdot_min_size).setZero();
        l_bound_ds_.segment(si_index_.slack_qdot_max_start,si_index_.slack_qdot_max_size).setZero();
        l_bound_ds_.segment(si_index_.slack_sing_start,si_index_.slack_sing_size).setZero();
        l_bound_ds_.segment(si_index_.slack_sel_col_start,si_index_.slack_sel_col_size).setZero();
    }

    void ManipulatorID::setIneqConstraint()    
    {

        double alpha = 50.;

        // Manipulator Joint Angle Limit (CBF)
        VectorXd q_min(joint_dof_), q_max(joint_dof_);
        auto q_lim = robot_data_->getJointPositionLimit();
        q_min = q_lim.first;
        q_max = q_lim.second;

        VectorXd q = robot_data_->getJointPosition();
        VectorXd qdot = robot_data_->getJointVelocity();
            
        A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qddot_start, si_index_.con_q_min_size, si_index_.qddot_size) = MatrixXd::Identity(si_index_.con_q_min_size,si_index_.qddot_size);
        A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, si_index_.con_q_min_size, si_index_.slack_q_min_size) = MatrixXd::Identity(si_index_.con_q_min_size,si_index_.slack_q_min_size);
        l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = -(alpha+alpha)*qdot - alpha*alpha*(q - q_min);
        
        A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qddot_start, si_index_.con_q_max_size, si_index_.qddot_size) = -MatrixXd::Identity(si_index_.con_q_max_size,si_index_.qddot_size);
        A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, si_index_.con_q_max_size, si_index_.slack_q_max_size) = MatrixXd::Identity(si_index_.con_q_max_size,si_index_.slack_q_max_size);
        l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_max_size) = +(alpha+alpha)*qdot - alpha*alpha*(q_max - q);

        // Manipulator Joint Velocity Limit (CBF)
        VectorXd qdot_min(joint_dof_), qdot_max(joint_dof_);
        auto qdot_lim = robot_data_->getJointVelocityLimit();
        qdot_min = qdot_lim.first;
        qdot_max = qdot_lim.second;

        A_ineq_ds_.block(si_index_.con_qdot_min_start, si_index_.qddot_start, si_index_.con_qdot_min_size, si_index_.qddot_size) = MatrixXd::Identity(si_index_.con_qdot_min_size, si_index_.qddot_size);
        A_ineq_ds_.block(si_index_.con_qdot_min_start, si_index_.slack_qdot_min_start, si_index_.con_qdot_min_size, si_index_.slack_qdot_min_size) = MatrixXd::Identity(si_index_.con_qdot_min_size, si_index_.slack_qdot_min_size);
        l_ineq_ds_.segment(si_index_.con_qdot_min_start, si_index_.con_qdot_min_size) = - alpha*(qdot - qdot_min);
        
        A_ineq_ds_.block(si_index_.con_qdot_max_start, si_index_.qddot_start, si_index_.con_qdot_max_size, si_index_.qddot_size) = -MatrixXd::Identity(si_index_.con_qdot_max_size, si_index_.qddot_size);
        A_ineq_ds_.block(si_index_.con_qdot_max_start, si_index_.slack_qdot_max_start, si_index_.con_qdot_max_size, si_index_.slack_qdot_max_size) = MatrixXd::Identity(si_index_.con_qdot_max_size, si_index_.slack_qdot_max_size);
        l_ineq_ds_.segment(si_index_.con_qdot_max_start, si_index_.con_qdot_max_size) = - alpha*(qdot_max - qdot);

        // singularity avoidance (CBF)
        RobotData::Manipulator::ManipulabilityResult mani_data = robot_data_->getManipulability(true, true, link_name_);

        A_ineq_ds_.block(si_index_.con_sing_start, si_index_.qddot_start, si_index_.con_sing_size, si_index_.qddot_size) = mani_data.grad.transpose();
        A_ineq_ds_.block(si_index_.con_sing_start, si_index_.slack_sing_start, si_index_.con_sing_size, si_index_.slack_sing_size) = MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
        l_ineq_ds_(si_index_.con_sing_start) = -mani_data.grad_dot.dot(qdot) - (alpha + alpha)*mani_data.grad.dot(qdot) - alpha*alpha*(mani_data.manipulability -0.01);
        
        // self collision avoidance (CBF)
        RobotData::Manipulator::MinDistResult min_dist_data = robot_data_->getMinDistance(true, true, false);
        
        A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.qddot_start, si_index_.con_sel_col_size, si_index_.qddot_size) = min_dist_data.grad.transpose();
        A_ineq_ds_.block(si_index_.con_sel_col_start, si_index_.slack_sel_col_start, si_index_.con_sel_col_size, si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
        l_ineq_ds_(si_index_.con_sel_col_start) = -min_dist_data.grad_dot.dot(qdot) - (alpha + alpha)*min_dist_data.grad.dot(qdot) - alpha*alpha*(min_dist_data.distance -0.05);
    }

    void ManipulatorID::setEqConstraint()    
    {
        /*
        subject to M * qddot + g = torque

        => subject to [M -I][ qddot  ] = [-g]
                            [ torque ]
        */
        MatrixXd M  = robot_data_->getMassMatrix();
        MatrixXd g = robot_data_->getGravity();

        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.qddot_start, si_index_.con_dyn_size, si_index_.qddot_size) = M;
        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.torque_start, si_index_.con_dyn_size, si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size, si_index_.torque_size);

        b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -g;
    }
}