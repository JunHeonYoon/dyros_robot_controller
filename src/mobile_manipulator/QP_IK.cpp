#include "dyros_robot_controller/mobile_manipulator/QP_IK.h"

namespace drc
{
    namespace MobileManipulator
    {
        QPIK::QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data)
        : QP::QPBase(), robot_data_(robot_data)
        {
            actuator_dof_ = robot_data_->getActuatordDof();
            mani_dof_ = robot_data_->getManipulatorDof();
            mobi_dof_ = robot_data_->getMobileDof();
    
            si_index_.eta_size              = actuator_dof_;
            si_index_.slack_q_mani_min_size = mani_dof_;
            si_index_.slack_q_mani_max_size = mani_dof_;
            si_index_.slack_sing_size       = 1;
            si_index_.slack_sel_col_size    = 1;
            si_index_.con_q_mani_min_size = mani_dof_;
            si_index_.con_q_mani_max_size = mani_dof_;
            si_index_.con_sing_size       = 1;
            si_index_.con_sel_col_size    = 1;
    
            const int nx = si_index_.eta_size +
                           si_index_.slack_q_mani_min_size +
                           si_index_.slack_q_mani_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_mani_min_size +
                              si_index_.con_q_mani_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size ;
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

            w_tracking_.setOnes(6);
            w_damping_.setOnes(actuator_dof_);
        }
    
        void QPIK::setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name)
        {
            xdot_desired_ = xdot_desired;
            link_name_ = link_name;
        }
    
        bool QPIK::getOptJointVel(VectorXd &opt_eta, QP::TimeDuration &time_status)
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

        void QPIK::setWeight(const VectorXd w_tracking, const VectorXd w_damping)
        {
            w_tracking_ = w_tracking;
            w_damping_ = w_damping;
        }
    
        void QPIK::setCost()
        {
            /*
                  min     || x_dot_des - J_tilda*eta ||_W1^2 + W * || eta ||_W2^2
                  eta
    
            =>    min     1/2 * eta.T * (2*J_tilda.T*W1*J_tilda + 2*W2) * eta + (-2*J_tilda.T*W1*x_dot_des).T * eta
                  eta
            */
           MatrixXd J_tilda = robot_data_->getJacobianActuated(link_name_);
    
           P_ds_.block(si_index_.eta_start,si_index_.eta_start,si_index_.eta_size,si_index_.eta_size) = 2.0 * J_tilda.transpose() * w_tracking_.asDiagonal() * J_tilda + 2.0 * w_damping_.asDiagonal().toDenseMatrix();
           q_ds_.segment(si_index_.eta_start,si_index_.eta_size) = -2.0 * J_tilda.transpose() * w_tracking_.asDiagonal() * xdot_desired_;
           q_ds_.segment(si_index_.slack_q_mani_min_start,si_index_.slack_q_mani_min_size) = VectorXd::Constant(si_index_.slack_q_mani_min_size, 1000.0);
           q_ds_.segment(si_index_.slack_q_mani_max_start,si_index_.slack_q_mani_max_size) = VectorXd::Constant(si_index_.slack_q_mani_max_size, 1000.0);
           q_ds_(si_index_.slack_sing_start) = 1000.0;
           q_ds_(si_index_.slack_sel_col_start) = 1000.0;
        }
    
        void QPIK::setBoundConstraint()    
        {
            // Manipulator Joint Velocity Limit
            l_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                                mani_dof_) = robot_data_->getJointVelocityLimit().first.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            u_bound_ds_.segment(si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                                mani_dof_) = robot_data_->getJointVelocityLimit().second.segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            l_bound_ds_.segment(si_index_.slack_q_mani_min_start,si_index_.slack_q_mani_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_mani_max_start,si_index_.slack_q_mani_max_size).setZero();
            l_bound_ds_(si_index_.slack_sing_start) = 0.0;
            l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
        }
    
        void QPIK::setIneqConstraint()    
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
            A_ineq_ds_.block(si_index_.con_q_mani_min_start,
                             si_index_.slack_q_mani_min_start,
                             si_index_.con_q_mani_min_size, 
                             si_index_.slack_q_mani_min_size) = -MatrixXd::Identity(si_index_.con_q_mani_min_size, si_index_.slack_q_mani_min_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_min_start, 
                               si_index_.con_q_mani_min_size) = - alpha*(q_mani - q_mani_min);
    
            A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                             si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                             si_index_.con_q_mani_max_size, 
                             mani_dof_) = -MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                             si_index_.slack_q_mani_max_start,
                             si_index_.con_q_mani_max_size, 
                             si_index_.slack_q_mani_max_size) = -MatrixXd::Identity(si_index_.con_q_mani_max_size, si_index_.slack_q_mani_max_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_max_start, 
                               si_index_.con_q_mani_max_size) = - alpha*(q_mani_max - q_mani);
    
            // singularity avoidance (CBF)
            Manipulator::ManipulabilityResult mani_result = robot_data_->getManipulability(true, false, link_name_);
    
            A_ineq_ds_.block(si_index_.con_sing_start, 
                             si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                             si_index_.con_sing_size, 
                             mani_dof_) = mani_result.grad.transpose();
            A_ineq_ds_.block(si_index_.con_sing_start, 
                             si_index_.slack_sing_start,
                             si_index_.con_sing_size, 
                             si_index_.slack_sing_size) = -MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);
    
            // self collision avoidance (CBF)
            VectorXd min_dist_grad;
            Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
            min_dist_res.grad = min_dist_res.grad.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            
            A_ineq_ds_.block(si_index_.con_sel_col_start, 
                             si_index_.eta_start + robot_data_->getActuatorIndex().mani_start,
                             si_index_.con_sel_col_size, 
                             mani_dof_) = min_dist_res.grad.transpose();
            A_ineq_ds_.block(si_index_.con_sel_col_start, 
                             si_index_.slack_sel_col_start,
                             si_index_.con_sel_col_size, 
                             si_index_.slack_sel_col_size) = -MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
            l_ineq_ds_(si_index_.con_sel_col_start) = - alpha*(min_dist_res.distance -0.05);
        }
    
        void QPIK::setEqConstraint()    
        {
    
        }
    } // namespace MobileManipulator
} // namespace drc