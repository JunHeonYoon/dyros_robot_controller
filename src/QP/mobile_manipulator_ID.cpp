#include "QP/mobile_manipulator_ID.h"

namespace QP
{
    MobileManipulatorID::MobileManipulatorID(std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data)
    : QPBase(), robot_data_(robot_data)
    {
        actuator_dof_ = robot_data_->getActuatordDof();
        mani_dof_ = robot_data_->getManipulatorDof();
        mobi_dof_ = robot_data_->getMobileDof();

        si_index_.eta_dot_size = actuator_dof_;
        si_index_.torque_size = actuator_dof_;

        si_index_.con_dyn_size = actuator_dof_;

        si_index_.con_q_mani_min_size = mani_dof_;
        si_index_.con_q_mani_max_size = mani_dof_;
        si_index_.con_qdot_mani_min_size = mani_dof_;
        si_index_.con_qdot_mani_max_size = mani_dof_;
        si_index_.con_sing_size = 1;
        si_index_.con_sel_col_size = 1;

        const int nx = si_index_.eta_dot_size + si_index_.torque_size; // qddot_actuated + torque_actuated
        const int nbound = 0;
        const int nineq = si_index_.con_q_mani_min_size +
                          si_index_.con_q_mani_max_size +
                          si_index_.con_qdot_mani_min_size +
                          si_index_.con_qdot_mani_max_size +
                          si_index_.con_sing_size +
                          si_index_.con_sel_col_size ;
        const int neq = si_index_.con_dyn_size;

        QPBase::setQPsize(nx, nbound, nineq, neq);

        si_index_.eta_dot_start = 0;
        si_index_.torque_start  = si_index_.eta_dot_start + si_index_.eta_dot_size;

        si_index_.con_dyn_start = 0;

        si_index_.con_q_mani_min_start    = 0;
        si_index_.con_q_mani_max_start    = si_index_.con_q_mani_min_start    + si_index_.con_q_mani_min_size;
        si_index_.con_qdot_mani_min_start = si_index_.con_q_mani_max_start    + si_index_.con_q_mani_max_size;
        si_index_.con_qdot_mani_max_start = si_index_.con_qdot_mani_min_start + si_index_.con_qdot_mani_min_size;
        si_index_.con_sing_start          = si_index_.con_qdot_mani_max_start + si_index_.con_qdot_mani_max_size;
        si_index_.con_sel_col_start       = si_index_.con_sing_start          + si_index_.con_sing_size;
    }

    void MobileManipulatorID::setDesiredTaskAcc(const VectorXd &xddot_desired, const std::string &link_name)
    {
        xddot_desired_ = xddot_desired;
        link_name_ = link_name;
    }

    bool MobileManipulatorID::getOptJoint(VectorXd &opt_etadot, VectorXd &opt_torque, TimeDuration &time_status)
    {
        MatrixXd sol;
        if(!solveQP(sol, time_status))
        {
            opt_etadot.setZero();
            opt_torque.setZero();
            time_status.setZero();
            return false;
        }
        else
        {
            opt_etadot = sol.block(si_index_.eta_dot_start,0,si_index_.eta_dot_size,1);
            opt_torque = sol.block(si_index_.torque_start,0,si_index_.torque_size,1);
            return true;
        }
    }

    void MobileManipulatorID::setCost()
    {
        /*
              min     || x_ddot_des - J_tilda*eta_dot - J_tilda_dot * eta ||_2^2
        eta_dot, torque

        =>      min        1/2 * [ eta_dot ].T * [ 2*J_tilda.T*J_tilda  0 ] * [ eta_dot ] + [ -2 * J_tilda.T * (x_ddot_des - J_tilda_dot * eta)].T * [ eta_dot ]
         [eta_dot, torque]       [ torque  ]     [          0           0 ]   [ torque  ]   [                    0                             ]     [ torque  ]
        */
       MatrixXd J_tilda = robot_data_->getJacobianActuated(link_name_);
       MatrixXd J_tilda_dot = robot_data_->getJacobianActuatedTimeVariation(link_name_);
       VectorXd eta = robot_data_->getJointVelocityActuated();

       P_ds_.block(si_index_.eta_dot_start,si_index_.eta_dot_start,si_index_.eta_dot_size,si_index_.eta_dot_size) = 2.0 * J_tilda.transpose() * J_tilda;
       q_ds_.segment(si_index_.eta_dot_start,si_index_.eta_dot_size) = -2.0 * J_tilda.transpose() * (xddot_desired_ - J_tilda_dot * eta);
       
    }

    void MobileManipulatorID::setBoundConstraint()    
    {
        // Not yet implemented
    }

    void MobileManipulatorID::setIneqConstraint()    
    {

        double alpha = 50.;

        // Manipulator Joint Angle Limit
        VectorXd q_mani_min(mani_dof_), q_mani_max(mani_dof_);
        auto q_lim = robot_data_->getJointPositionLimit();
        q_mani_min = q_lim.first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
        q_mani_max = q_lim.second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);

        VectorXd q_actuated = robot_data_->getJointPositionActuated();
        VectorXd qdot_actuated = robot_data_->getJointVelocityActuated();
    
        VectorXd q_mani = q_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
        VectorXd qdot_mani = qdot_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
        
        A_ineq_ds_.block(si_index_.con_q_mani_min_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                         si_index_.con_q_mani_min_size, 
                         mani_dof_) = MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
        l_ineq_ds_.segment(si_index_.con_q_mani_min_start, 
                           si_index_.con_q_mani_min_size) = -(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani - q_mani_min);

        A_ineq_ds_.block(si_index_.con_q_mani_max_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                         si_index_.con_q_mani_max_size, 
                         mani_dof_) = -MatrixXd::Identity(si_index_.con_q_mani_max_size, mani_dof_);        
        l_ineq_ds_.segment(si_index_.con_q_mani_max_start, 
                           si_index_.con_q_mani_max_size) = +(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani_max - q_mani);

        // Manipulator Joint Velocity Limit
        VectorXd qdot_mani_min(mani_dof_), qdot_mani_max(mani_dof_);
        auto qdot_lim = robot_data_->getJointVelocityLimit();
        qdot_mani_min = qdot_lim.first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
        qdot_mani_max = qdot_lim.second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);

        A_ineq_ds_.block(si_index_.con_qdot_mani_min_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                         si_index_.con_qdot_mani_min_size, 
                         mani_dof_) = MatrixXd::Identity(si_index_.con_qdot_mani_min_size, mani_dof_);
        l_ineq_ds_.segment(si_index_.con_qdot_mani_min_start, 
                           si_index_.con_qdot_mani_min_size) = -alpha*(qdot_mani - qdot_mani_min);

        A_ineq_ds_.block(si_index_.con_qdot_mani_max_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                         si_index_.con_qdot_mani_max_size, 
                         mani_dof_) = -MatrixXd::Identity(si_index_.con_qdot_mani_max_size, mani_dof_);
        l_ineq_ds_.segment(si_index_.con_qdot_mani_max_start, si_index_.con_qdot_mani_max_size) = -alpha*(qdot_mani_max - qdot_mani);

        // singularity avoidance
        RobotData::Manipulator::ManipulabilityResult mani_data = robot_data_->getManipulability(true, true, link_name_);

        A_ineq_ds_.block(si_index_.con_sing_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                         si_index_.con_sing_size, 
                         mani_dof_) = mani_data.grad.transpose();
        l_ineq_ds_(si_index_.con_sing_start) = -mani_data.grad_dot.dot(qdot_mani) - (alpha + alpha)*mani_data.grad.dot(qdot_mani) - alpha*alpha*(mani_data.manipulability -0.01);

        // self collision avoidance
        RobotData::Manipulator::MinDistResult min_dist_data = robot_data_->getMinDistance(true, true, false);
        min_dist_data.grad = min_dist_data.grad.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
        min_dist_data.grad_dot = min_dist_data.grad_dot.segment(robot_data_->getJointIndex().mani_start, mani_dof_);

        A_ineq_ds_.block(si_index_.con_sel_col_start, 
                         si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start,  
                         si_index_.con_sel_col_size, 
                         mani_dof_) = min_dist_data.grad.transpose();
        l_ineq_ds_(si_index_.con_sel_col_start) = -min_dist_data.grad_dot.dot(qdot_mani) - (alpha + alpha)*min_dist_data.grad.dot(qdot_mani) - alpha*alpha*(min_dist_data.distance -0.05);
    }

    void MobileManipulatorID::setEqConstraint()    
    {
        /*
        subject to M_tilda * eta_dot + g_tilda = torque

        => subject to [ M_tilda -I ][ eta_dot ] = [ -g_tilda ]
                                    [ torque  ]
        */
        MatrixXd M_tilda  = robot_data_->getMassMatrixActuated();
        MatrixXd g_tilda = robot_data_->getGravityActuated();

        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.eta_dot_start,si_index_.con_dyn_size,si_index_.eta_dot_size) = M_tilda;
        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.torque_start,si_index_.con_dyn_size,si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size,si_index_.torque_size);

        b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -g_tilda;
    }
}