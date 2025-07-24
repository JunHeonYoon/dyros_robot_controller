#include "QP/mobile_manipulator_IK.h"

namespace QP
{
    MobileManipulatorIK::MobileManipulatorIK(std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data)
    : QPBase(), robot_data_(robot_data)
    {
        actuator_dof_ = robot_data_->getActuatordDof();
        mani_dof_ = robot_data_->getManipulatorDof();
        mobi_dof_ = robot_data_->getMobileDof();

        si_index_.eta_size            = actuator_dof_;
        si_index_.con_q_mani_min_size = mani_dof_;
        si_index_.con_q_mani_max_size = mani_dof_;
        si_index_.con_sing_size       = 1;
        si_index_.con_sel_col_size    = 1;

        const int nx = si_index_.eta_size; // qdot_actuated
        const int nbound = nx;
        const int nineq = si_index_.con_q_mani_min_size +
                          si_index_.con_q_mani_max_size +
                          si_index_.con_sing_size +
                          si_index_.con_sel_col_size ;
        const int neq = 0;

        QPBase::setQPsize(nx, nbound, nineq, neq);

        si_index_.eta_start               = 0;
        si_index_.con_q_mani_min_start    = 0;
        si_index_.con_q_mani_max_start    = si_index_.con_q_mani_min_start + si_index_.con_q_mani_min_size;
        si_index_.con_sing_start          = si_index_.con_q_mani_max_start + si_index_.con_q_mani_max_size;
        si_index_.con_sel_col_start       = si_index_.con_sing_start       + si_index_.con_sing_size;
    }

    void MobileManipulatorIK::setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name)
    {
        xdot_desired_ = xdot_desired;
        link_name_ = link_name;
    }

    bool MobileManipulatorIK::getOptJointVel(VectorXd &opt_eta, TimeDuration &time_status)
    {
        MatrixXd sol;
        if(!solveQP(sol, time_status))
        {
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

    void MobileManipulatorIK::setCost()
    {
        /*
              min     || x_dot_des - J_tilda*eta ||_2^2 + W * || eta ||_2^2
              eta

        =>    min     1/2 * eta.T * (2*J_tilda.T*J_tilda + W) * eta + (-2*J_tilda.T*x_dot_des).T * eta
              eta
        */
       MatrixXd J_tilda = robot_data_->getJacobianActuated(link_name_);

       P_ds_.block(si_index_.eta_start,si_index_.eta_start,si_index_.eta_size,si_index_.eta_size) = 2.0 * J_tilda.transpose() * J_tilda;
       P_ds_.block(si_index_.eta_start,si_index_.eta_start,si_index_.eta_size,si_index_.eta_size) += 0.01*MatrixXd::Identity(si_index_.eta_size,si_index_.eta_size);
       q_ds_.segment(si_index_.eta_start,si_index_.eta_size) = -2.0 * J_tilda.transpose() * xdot_desired_;
    }

    void MobileManipulatorIK::setBoundConstraint()    
    {
        // TODO: Bound constraint makes the robot weired
        // Manipulator Joint Velocity Limit
        // l_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
        //                     mani_dof_) = robot_data_->getJointVelocityLimit().first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
        // u_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
        //                     mani_dof_) = robot_data_->getJointVelocityLimit().second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
    }

    void MobileManipulatorIK::setIneqConstraint()    
    {
        double alpha = 50.;

        // Manipulator Joint Angle Limit (CBF)
        VectorXd q_mani_min = robot_data_->getJointPositionLimit().first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
        VectorXd q_mani_max = robot_data_->getJointPositionLimit().second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
       
        VectorXd q_mani = robot_data_->getJointPosition().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            
        A_ineq_ds_.block(si_index_.con_q_mani_min_start,
                         si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                         si_index_.con_q_mani_min_size, 
                         mani_dof_) = MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
        l_ineq_ds_.segment(si_index_.con_q_mani_min_start, 
                           si_index_.con_q_mani_min_size) = - alpha*(q_mani - q_mani_min);

        A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                         si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                         si_index_.con_q_mani_max_size, 
                         mani_dof_) = -MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
        l_ineq_ds_.segment(si_index_.con_q_mani_max_start, 
                           si_index_.con_q_mani_max_size) = - alpha*(q_mani_max - q_mani);

        // singularity avoidance (CBF)
        RobotData::Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);

        A_ineq_ds_.block(si_index_.con_sing_start, 
                         si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                         si_index_.con_sing_size, 
                         mani_dof_) = mani_result.grad.transpose();
        l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);

        // self collision avoidance (CBF)
        VectorXd min_dist_grad;
        RobotData::Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
        min_dist_res.grad = min_dist_res.grad.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
        
        A_ineq_ds_.block(si_index_.con_sel_col_start, 
                         si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                         si_index_.con_sel_col_size, 
                         mani_dof_) = min_dist_res.grad.transpose();
        l_ineq_ds_(si_index_.con_sel_col_start) = - alpha*(min_dist_res.distance -0.05);
    }

    void MobileManipulatorIK::setEqConstraint()    
    {

    }
}