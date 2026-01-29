#include "dyros_robot_controller/mobile_manipulator/QP_ID.h"

namespace drc
{
    namespace MobileManipulator
    {
        QPID::QPID(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt)
        : QP::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            actuator_dof_ = robot_data_->getActuatordDof();
            mani_dof_ = robot_data_->getManipulatorDof();
            mobi_dof_ = robot_data_->getMobileDof();
    
            si_index_.eta_dot_size             = actuator_dof_;
            si_index_.torque_size              = actuator_dof_;
            si_index_.slack_q_mani_min_size    = mani_dof_;
            si_index_.slack_q_mani_max_size    = mani_dof_;
            si_index_.slack_qdot_mani_min_size = mani_dof_;
            si_index_.slack_qdot_mani_max_size = mani_dof_;
            si_index_.slack_sing_size          = 1;
            si_index_.slack_sel_col_size       = 1;
            si_index_.slack_base_vel_min_size  = 3;
            si_index_.slack_base_vel_max_size  = 3;
            si_index_.slack_base_acc_min_size  = 3;
            si_index_.slack_base_acc_max_size  = 3;
            si_index_.con_dyn_size             = actuator_dof_;
            si_index_.con_q_mani_min_size      = mani_dof_;
            si_index_.con_q_mani_max_size      = mani_dof_;
            si_index_.con_qdot_mani_min_size   = mani_dof_;
            si_index_.con_qdot_mani_max_size   = mani_dof_;
            si_index_.con_sing_size            = 1;
            si_index_.con_sel_col_size         = 1;
            si_index_.con_base_vel_min_size    = 3;
            si_index_.con_base_vel_max_size    = 3;
            si_index_.con_base_acc_min_size    = 3;
            si_index_.con_base_acc_max_size    = 3;
    
            const int nx = si_index_.eta_dot_size + 
                           si_index_.torque_size +
                           si_index_.slack_q_mani_min_size +
                           si_index_.slack_q_mani_max_size +
                           si_index_.slack_qdot_mani_min_size +
                           si_index_.slack_qdot_mani_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size +
                           si_index_.slack_base_vel_min_size +
                           si_index_.slack_base_vel_max_size +
                           si_index_.slack_base_acc_min_size +
                           si_index_.slack_base_acc_max_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_mani_min_size +
                              si_index_.con_q_mani_max_size +
                              si_index_.con_qdot_mani_min_size +
                              si_index_.con_qdot_mani_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size +
                              si_index_.con_base_vel_min_size +
                              si_index_.con_base_vel_max_size +
                              si_index_.con_base_acc_min_size +
                              si_index_.con_base_acc_max_size;
            const int neq = si_index_.con_dyn_size;
    
            QPBase::setQPsize(nx, nbound, nineq, neq);
    
            si_index_.eta_dot_start             = 0;
            si_index_.torque_start              = si_index_.eta_dot_start             + si_index_.eta_dot_size;
            si_index_.slack_q_mani_min_start    = si_index_.torque_start              + si_index_.torque_size;
            si_index_.slack_q_mani_max_start    = si_index_.slack_q_mani_min_start    + si_index_.slack_q_mani_min_size;
            si_index_.slack_qdot_mani_min_start = si_index_.slack_q_mani_max_start    + si_index_.slack_q_mani_max_size;
            si_index_.slack_qdot_mani_max_start = si_index_.slack_qdot_mani_min_start + si_index_.slack_qdot_mani_min_size;
            si_index_.slack_sing_start          = si_index_.slack_qdot_mani_max_start + si_index_.slack_qdot_mani_max_size;
            si_index_.slack_sel_col_start       = si_index_.slack_sing_start          + si_index_.slack_sing_size;
            si_index_.slack_base_vel_min_start  = si_index_.slack_sel_col_start       + si_index_.slack_sel_col_size;
            si_index_.slack_base_vel_max_start  = si_index_.slack_base_vel_min_start  + si_index_.slack_base_vel_min_size;
            si_index_.slack_base_acc_min_start  = si_index_.slack_base_vel_max_start  + si_index_.slack_base_vel_max_size;
            si_index_.slack_base_acc_max_start  = si_index_.slack_base_acc_min_start  + si_index_.slack_base_acc_min_size;
            si_index_.con_dyn_start             = 0;
            si_index_.con_q_mani_min_start      = 0;
            si_index_.con_q_mani_max_start      = si_index_.con_q_mani_min_start    + si_index_.con_q_mani_min_size;
            si_index_.con_qdot_mani_min_start   = si_index_.con_q_mani_max_start    + si_index_.con_q_mani_max_size;
            si_index_.con_qdot_mani_max_start   = si_index_.con_qdot_mani_min_start + si_index_.con_qdot_mani_min_size;
            si_index_.con_sing_start            = si_index_.con_qdot_mani_max_start + si_index_.con_qdot_mani_max_size;
            si_index_.con_sel_col_start         = si_index_.con_sing_start          + si_index_.con_sing_size;
            si_index_.con_base_vel_min_start    = si_index_.con_sel_col_start       + si_index_.con_sel_col_size;
            si_index_.con_base_vel_max_start    = si_index_.con_base_vel_min_start  + si_index_.con_base_vel_min_size;
            si_index_.con_base_acc_min_start    = si_index_.con_base_vel_max_start  + si_index_.con_base_vel_max_size;
            si_index_.con_base_acc_max_start    = si_index_.con_base_acc_min_start  + si_index_.con_base_acc_min_size;

            w_mani_vel_damping_.setOnes(mani_dof_);
            w_mani_acc_damping_.setOnes(mani_dof_);
            w_base_vel_damping_.setOnes();
            w_base_acc_damping_.setOnes();
        }
        

        void QPID::setDesiredTaskAcc(const std::map<std::string, Vector6d> &link_xddot_desired)
        {
            link_xddot_desired_ = link_xddot_desired;
        }
    
        bool QPID::getOptJoint(Eigen::Ref<Eigen::VectorXd> opt_etadot, Eigen::Ref<Eigen::VectorXd> opt_torque, QP::TimeDuration &time_status)
        {
            if(opt_etadot.size() != actuator_dof_ || opt_torque.size() != actuator_dof_)
            {
                std::cerr << "Size of opt_etadot(" << opt_etadot.size() << ") or opt_torque(" << opt_torque.size() << ") are not same as actuator_dof_(" << actuator_dof_ << ")" << std::endl;
                time_status.setZero();
                return false;
            }
            MatrixXd sol;
            if(!solveQP(sol, time_status))
            {
                std::cerr << "QP ID failed to compute optimal joint torque." << std::endl;
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

        void QPID::setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                             const Eigen::Ref<const VectorXd>& w_mani_vel_damping, 
                             const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                             const Eigen::Vector3d& w_base_vel_damping,
                             const Eigen::Vector3d& w_base_acc_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_mani_vel_damping_ = w_mani_vel_damping;
            w_mani_acc_damping_ = w_mani_acc_damping;
            w_base_vel_damping_ = w_base_vel_damping;
            w_base_acc_damping_ = w_base_acc_damping;
        }

    
        void QPID::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);
            
            // for task space acceleration tracking
            const VectorXd eta = robot_data_->getJointVelocityActuated();
            for(const auto& [link_name, xddot_desired] : link_xddot_desired_)
            {
                const MatrixXd J_i_tilda = robot_data_->getJacobianActuated(link_name);
                const MatrixXd J_i_tilda_dot = robot_data_->getJacobianActuatedTimeVariation(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.eta_dot_start,si_index_.eta_dot_start,si_index_.eta_dot_size,si_index_.eta_dot_size) += 2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * J_i_tilda;
                q_ds_.segment(si_index_.eta_dot_start,si_index_.eta_dot_size) += -2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * (xddot_desired - J_i_tilda_dot * eta);
            }
            
            const auto actuator_idx = robot_data_->getActuatorIndex();
            const int mani_start = actuator_idx.mani_start;
            const int mobi_start = actuator_idx.mobi_start;

            // for manipulator joint velocity/acceleration damping
            P_ds_.block(si_index_.eta_dot_start+mani_start,
                        si_index_.eta_dot_start+mani_start,
                        mani_dof_,
                        mani_dof_) += 2.0 * w_mani_acc_damping_.asDiagonal().toDenseMatrix() +  2.0 * dt_ * dt_ * w_mani_vel_damping_.asDiagonal().toDenseMatrix();
            q_ds_.segment(si_index_.eta_dot_start+mani_start,mani_dof_) += 2.0 * dt_ * w_mani_vel_damping_.asDiagonal().toDenseMatrix() * eta.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);

            const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
            const MatrixXd J_mobile_T = J_mobile.transpose();
            const Matrix3d w_base_acc = w_base_acc_damping_.asDiagonal();
            const Matrix3d w_base_vel = w_base_vel_damping_.asDiagonal();
            P_ds_.block(si_index_.eta_dot_start + mobi_start,
                        si_index_.eta_dot_start + mobi_start,
                        mobi_dof_,
                        mobi_dof_) += 2.0 * J_mobile_T * w_base_acc * J_mobile + 2.0 * dt_ * dt_ * J_mobile_T * w_base_vel * J_mobile;
            q_ds_.segment(si_index_.eta_dot_start + mobi_start, mobi_dof_) += 2.0 * dt_ * J_mobile_T * w_base_vel * robot_data_->getBaseVel();


            // for slack
            q_ds_.segment(si_index_.slack_q_mani_min_start,   si_index_.slack_q_mani_min_size)    = VectorXd::Constant(si_index_.slack_q_mani_min_size,    1000.0);
            q_ds_.segment(si_index_.slack_q_mani_max_start,   si_index_.slack_q_mani_max_size)    = VectorXd::Constant(si_index_.slack_q_mani_max_size,    1000.0);
            q_ds_.segment(si_index_.slack_qdot_mani_min_start,si_index_.slack_qdot_mani_min_size) = VectorXd::Constant(si_index_.slack_qdot_mani_min_size, 1000.0);
            q_ds_.segment(si_index_.slack_qdot_mani_max_start,si_index_.slack_qdot_mani_max_size) = VectorXd::Constant(si_index_.slack_qdot_mani_max_size, 1000.0);
            q_ds_(si_index_.slack_sing_start)    = 1000.0;
            q_ds_(si_index_.slack_sel_col_start) = 1000.0;
            q_ds_.segment(si_index_.slack_base_vel_min_start, si_index_.slack_base_vel_min_size) = VectorXd::Constant(si_index_.slack_base_vel_min_size, 1000.0);
            q_ds_.segment(si_index_.slack_base_vel_max_start, si_index_.slack_base_vel_max_size) = VectorXd::Constant(si_index_.slack_base_vel_max_size, 1000.0);
            q_ds_.segment(si_index_.slack_base_acc_min_start, si_index_.slack_base_acc_min_size) = VectorXd::Constant(si_index_.slack_base_acc_min_size, 1000.0);
            q_ds_.segment(si_index_.slack_base_acc_max_start, si_index_.slack_base_acc_max_size) = VectorXd::Constant(si_index_.slack_base_acc_max_size, 1000.0);

        }
    
        void QPID::setBoundConstraint()    
        {
            l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_,OSQP_INFTY);

            // for slack
            l_bound_ds_.segment(si_index_.slack_q_mani_min_start,si_index_.slack_q_mani_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_mani_max_start,si_index_.slack_q_mani_max_size).setZero();
            l_bound_ds_.segment(si_index_.slack_qdot_mani_min_start,si_index_.slack_qdot_mani_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_qdot_mani_max_start,si_index_.slack_qdot_mani_max_size).setZero();
            l_bound_ds_(si_index_.slack_sing_start) = 0.0;
            l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
            l_bound_ds_.segment(si_index_.slack_base_vel_min_start, si_index_.slack_base_vel_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_base_vel_max_start, si_index_.slack_base_vel_max_size).setZero();
            l_bound_ds_.segment(si_index_.slack_base_acc_min_start, si_index_.slack_base_acc_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_base_acc_max_start, si_index_.slack_base_acc_max_size).setZero();
        }
    
        void QPID::setIneqConstraint()    
        {
            A_ineq_ds_.setZero(nineqc_, nx_);
            l_ineq_ds_.setConstant(nineqc_,-OSQP_INFTY);
            u_ineq_ds_.setConstant(nineqc_,OSQP_INFTY);

            const double alpha = 50.;
    
            // Manipulator Joint Angle Limit
            const auto q_lim = robot_data_->getJointPositionLimit();
            const VectorXd q_mani_min_raw = q_lim.first.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd q_mani_max_raw = q_lim.second.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd q_mani_min =
                (q_mani_min_raw.array() < 0.0)
                    .select(q_mani_min_raw.array() * 0.8, q_mani_min_raw.array() * 1.8)
                    .matrix();
            const VectorXd q_mani_max =
                (q_mani_max_raw.array() > 0.0)
                    .select(q_mani_max_raw.array() * 0.8, q_mani_max_raw.array() * 1.8)
                    .matrix();
    
            const VectorXd q_actuated = robot_data_->getJointPositionActuated();
            const VectorXd qdot_actuated = robot_data_->getJointVelocityActuated();
        
            const VectorXd q_mani = q_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
            const VectorXd qdot_mani = qdot_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
            
            A_ineq_ds_.block(si_index_.con_q_mani_min_start, 
                             si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                             si_index_.con_q_mani_min_size, 
                             mani_dof_) = MatrixXd::Identity(si_index_.con_q_mani_min_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_q_mani_min_start,
                             si_index_.slack_q_mani_min_start,
                             si_index_.con_q_mani_min_size, 
                             si_index_.slack_q_mani_min_size) = MatrixXd::Identity(si_index_.con_q_mani_min_size, si_index_.slack_q_mani_min_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_min_start, 
                               si_index_.con_q_mani_min_size) = -(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani - q_mani_min);
    
            A_ineq_ds_.block(si_index_.con_q_mani_max_start, 
                             si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                             si_index_.con_q_mani_max_size, 
                             mani_dof_) = -MatrixXd::Identity(si_index_.con_q_mani_max_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_q_mani_max_start,
                             si_index_.slack_q_mani_max_start,
                             si_index_.con_q_mani_max_size, 
                             si_index_.slack_q_mani_max_size) = MatrixXd::Identity(si_index_.con_q_mani_max_size, si_index_.slack_q_mani_max_size);
            l_ineq_ds_.segment(si_index_.con_q_mani_max_start, 
                               si_index_.con_q_mani_max_size) = +(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani_max - q_mani);
    
            // Manipulator Joint Velocity Limit
            const auto qdot_lim = robot_data_->getJointVelocityLimit();
            const VectorXd qdot_mani_min_raw = qdot_lim.first.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd qdot_mani_max_raw = qdot_lim.second.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            const VectorXd qdot_mani_min =
                (qdot_mani_min_raw.array() < 0.0)
                    .select(qdot_mani_min_raw.array() * 0.8, qdot_mani_min_raw.array() * 1.8)
                    .matrix();
            const VectorXd qdot_mani_max =
                (qdot_mani_max_raw.array() > 0.0)
                    .select(qdot_mani_max_raw.array() * 0.8, qdot_mani_max_raw.array() * 1.8)
                    .matrix();
    
            A_ineq_ds_.block(si_index_.con_qdot_mani_min_start, 
                             si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                             si_index_.con_qdot_mani_min_size, 
                             mani_dof_) = MatrixXd::Identity(si_index_.con_qdot_mani_min_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_qdot_mani_min_start,
                             si_index_.slack_qdot_mani_min_start,
                             si_index_.con_qdot_mani_min_size, 
                             si_index_.slack_qdot_mani_min_size) = MatrixXd::Identity(si_index_.con_qdot_mani_min_size, si_index_.slack_qdot_mani_min_size);
            l_ineq_ds_.segment(si_index_.con_qdot_mani_min_start, 
                               si_index_.con_qdot_mani_min_size) = -alpha*(qdot_mani - qdot_mani_min);
    
            A_ineq_ds_.block(si_index_.con_qdot_mani_max_start, 
                             si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
                             si_index_.con_qdot_mani_max_size, 
                             mani_dof_) = -MatrixXd::Identity(si_index_.con_qdot_mani_max_size, mani_dof_);
            A_ineq_ds_.block(si_index_.con_qdot_mani_max_start,
                             si_index_.slack_qdot_mani_max_start,
                             si_index_.con_qdot_mani_max_size, 
                             si_index_.slack_qdot_mani_max_size) = MatrixXd::Identity(si_index_.con_qdot_mani_max_size, si_index_.slack_qdot_mani_max_size);
            l_ineq_ds_.segment(si_index_.con_qdot_mani_max_start, si_index_.con_qdot_mani_max_size) = -alpha*(qdot_mani_max - qdot_mani);
    
            // singularity avoidance
            // Manipulator::ManipulabilityResult mani_data = robot_data_->getManipulability(true, true, link_name_);
    
            // A_ineq_ds_.block(si_index_.con_sing_start, 
            //                  si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start, 
            //                  si_index_.con_sing_size, 
            //                  mani_dof_) = mani_data.grad.transpose();
            // A_ineq_ds_.block(si_index_.con_sing_start, 
            //                  si_index_.slack_sing_start,
            //                  si_index_.con_sing_size, 
            //                  si_index_.slack_sing_size) = -MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = -mani_data.grad_dot.dot(qdot_mani) - (alpha + alpha)*mani_data.grad.dot(qdot_mani) - alpha*alpha*(mani_data.manipulability -0.01);
    
            // self collision avoidance
            Manipulator::MinDistResult min_dist_data = robot_data_->getMinDistance(true, true, false);
            min_dist_data.grad = min_dist_data.grad.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
            min_dist_data.grad_dot = min_dist_data.grad_dot.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
    
            A_ineq_ds_.block(si_index_.con_sel_col_start, 
                             si_index_.eta_dot_start + robot_data_->getActuatorIndex().mani_start,  
                             si_index_.con_sel_col_size, 
                             mani_dof_) = min_dist_data.grad.transpose();
            A_ineq_ds_.block(si_index_.con_sel_col_start, 
                             si_index_.slack_sel_col_start,
                             si_index_.con_sel_col_size, 
                             si_index_.slack_sel_col_size) = MatrixXd::Identity(si_index_.con_sel_col_size, si_index_.slack_sel_col_size);
            l_ineq_ds_(si_index_.con_sel_col_start) = -min_dist_data.grad_dot.dot(qdot_mani) - (alpha + alpha)*min_dist_data.grad.dot(qdot_mani) - alpha*alpha*(min_dist_data.distance -0.01);
            
            // Mobile base velocity & acceleration limit
            const auto& param = robot_data_->getKineParam();
            const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
            const int mobi_start = robot_data_->getActuatorIndex().mobi_start;
            const Vector3d base_vel = robot_data_->getMobileBaseVel();

            Vector3d vel_limit;
            vel_limit << param.max_lin_speed, param.max_lin_speed, param.max_ang_speed;
            const Vector3d vel_min = -vel_limit;
            const Vector3d vel_max = vel_limit;

            A_ineq_ds_.block(si_index_.con_base_vel_min_start,
                             si_index_.eta_dot_start + mobi_start,
                             si_index_.con_base_vel_min_size,
                             mobi_dof_) = J_mobile;
            A_ineq_ds_.block(si_index_.con_base_vel_min_start,
                             si_index_.slack_base_vel_min_start,
                             si_index_.con_base_vel_min_size,
                             si_index_.slack_base_vel_min_size) = MatrixXd::Identity(si_index_.con_base_vel_min_size, si_index_.slack_base_vel_min_size);
            l_ineq_ds_.segment(si_index_.con_base_vel_min_start,
                               si_index_.con_base_vel_min_size) = -alpha*(base_vel - vel_min);

            A_ineq_ds_.block(si_index_.con_base_vel_max_start,
                             si_index_.eta_dot_start + mobi_start,
                             si_index_.con_base_vel_max_size,
                             mobi_dof_) = -J_mobile;
            A_ineq_ds_.block(si_index_.con_base_vel_max_start,
                             si_index_.slack_base_vel_max_start,
                             si_index_.con_base_vel_max_size,
                             si_index_.slack_base_vel_max_size) = MatrixXd::Identity(si_index_.con_base_vel_max_size, si_index_.slack_base_vel_max_size);
            l_ineq_ds_.segment(si_index_.con_base_vel_max_start,
                               si_index_.con_base_vel_max_size) = -alpha*(vel_max - base_vel);

            Vector3d acc_limit;
            acc_limit << param.max_lin_acc, param.max_lin_acc, param.max_ang_acc;

            A_ineq_ds_.block(si_index_.con_base_acc_min_start,
                             si_index_.eta_dot_start + mobi_start,
                             si_index_.con_base_acc_min_size,
                             mobi_dof_) = J_mobile;
            A_ineq_ds_.block(si_index_.con_base_acc_min_start,
                             si_index_.slack_base_acc_min_start,
                             si_index_.con_base_acc_min_size,
                             si_index_.slack_base_acc_min_size) = MatrixXd::Identity(si_index_.con_base_acc_min_size, si_index_.slack_base_acc_min_size);
            l_ineq_ds_.segment(si_index_.con_base_acc_min_start,
                               si_index_.con_base_acc_min_size) = -acc_limit;

            A_ineq_ds_.block(si_index_.con_base_acc_max_start,
                             si_index_.eta_dot_start + mobi_start,
                             si_index_.con_base_acc_max_size,
                             mobi_dof_) = -J_mobile;
            A_ineq_ds_.block(si_index_.con_base_acc_max_start,
                             si_index_.slack_base_acc_max_start,
                             si_index_.con_base_acc_max_size,
                             si_index_.slack_base_acc_max_size) = MatrixXd::Identity(si_index_.con_base_acc_max_size, si_index_.slack_base_acc_max_size);
            l_ineq_ds_.segment(si_index_.con_base_acc_max_start,
                               si_index_.con_base_acc_max_size) = -acc_limit;
        }
    
        void QPID::setEqConstraint()    
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);

            // for dynamics
            const MatrixXd M_tilda  = robot_data_->getMassMatrixActuated();
            const MatrixXd g_tilda = robot_data_->getGravityActuated();
    
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.eta_dot_start,si_index_.con_dyn_size,si_index_.eta_dot_size) = M_tilda;
            A_eq_ds_.block(si_index_.con_dyn_start,si_index_.torque_start,si_index_.con_dyn_size,si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size,si_index_.torque_size);
    
            b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -g_tilda;
        }
    } // namespace MobileManipulator
} // namespace drc
