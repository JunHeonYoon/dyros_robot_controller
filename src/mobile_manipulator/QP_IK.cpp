#include "dyros_robot_controller/mobile_manipulator/QP_IK.h"

namespace drc
{
    namespace MobileManipulator
    {
        QPIK::QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt)
        : QP::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            actuator_dof_ = robot_data_->getActuatordDof();
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

            w_mani_damping_.setOnes(mani_dof_);
            w_base_damping_.setOnes();
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
        
        void QPIK::setWeight(const std::map<std::string, Vector6d>& link_w_tracking, 
                               const Eigen::Ref<const VectorXd>& w_mani_damping,
                               const Eigen::Vector3d& w_base_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_mani_damping_ = w_mani_damping;
            w_base_damping_ = w_base_damping;
        }
    
        void QPIK::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);

            // for task space velocity tracking
            for(const auto& [link_name, xdot_desired] : link_xdot_desired_)
            {
                const MatrixXd J_i_tilda = robot_data_->getJacobianActuated(link_name);
                Vector6d w_tracking; w_tracking.setConstant(1.0);

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) w_tracking = iter->second;

                P_ds_.block(si_index_.eta_start,si_index_.eta_start,si_index_.eta_size,si_index_.eta_size) += 2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * J_i_tilda;
                q_ds_.segment(si_index_.eta_start,si_index_.eta_size) += -2.0 * J_i_tilda.transpose() * w_tracking.asDiagonal() * xdot_desired;
            }

            // for manipulator joint velocity damping
            P_ds_.block(si_index_.eta_start+robot_data_->getActuatorIndex().mani_start,
                        si_index_.eta_start+robot_data_->getActuatorIndex().mani_start,
                        mani_dof_,
                        mani_dof_) += 2.0 * w_mani_damping_.asDiagonal();

            // for mobile base velocity damping
            const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
            P_ds_.block(si_index_.eta_start+robot_data_->getActuatorIndex().mobi_start,
                        si_index_.eta_start+robot_data_->getActuatorIndex().mobi_start,
                        mobi_dof_,
                        mobi_dof_) += 2.0 * J_mobile.transpose() * w_base_damping_.asDiagonal() * J_mobile;

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

            const double alpha = 50.;
    
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
            //                  si_index_.slack_sing_size) = -MatrixXd::Identity(si_index_.con_sing_size, si_index_.slack_sing_size);
            // l_ineq_ds_(si_index_.con_sing_start) = - alpha*(mani_result.manipulability -0.01);
    
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
            l_ineq_ds_(si_index_.con_sel_col_start) = - alpha*(min_dist_res.distance -0.01);

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
