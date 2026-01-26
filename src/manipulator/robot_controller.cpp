#include "dyros_robot_controller/manipulator/robot_controller.h"

namespace drc
{
    namespace Manipulator
    {
        RobotController::RobotController(std::shared_ptr<Manipulator::RobotData> robot_data)
        : robot_data_(robot_data), dt_(robot_data->getDt())
        {
            dof_ = robot_data_->getDof();
            Kp_joint_ = VectorXd::Constant(dof_, 400);
            Kv_joint_ = VectorXd::Constant(dof_, 40);
    
            QP_mani_IK_ = std::make_unique<Manipulator::QPIK>(robot_data_, dt_);
            QP_mani_ID_ = std::make_unique<Manipulator::QPID>(robot_data_, dt_);
        }
    
        void RobotController::setJointGain(const VectorXd& Kp, const VectorXd& Kv)
        {
            assert(Kp.size() == dof_ && Kv.size() != dof_);
            Kp_joint_ = Kp;
            Kv_joint_ = Kv;
        }

        void RobotController::setJointKpGain(const VectorXd& Kp)
        {
            assert(Kp.size() == dof_);
            Kp_joint_ = Kp;
        }

        void RobotController::setJointKvGain(const VectorXd& Kv)
        {
            assert(Kv.size() == dof_);
            Kv_joint_ = Kv;
        }
    
        void RobotController::setTaskGain(const std::map<std::string, Vector6d>& link_Kp, const std::map<std::string, Vector6d>& link_Kv)
        {
            link_Kp_task_ = link_Kp;
            link_Kv_task_ = link_Kv;
        }

        void RobotController::setTaskKpGain(const std::map<std::string, Vector6d>& link_Kp)
        {
            link_Kp_task_ = link_Kp;
        }

        void RobotController::setTaskKvGain(const std::map<std::string, Vector6d>& link_Kv)
        {
            link_Kv_task_ = link_Kv;
        }

        void RobotController::setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_damping)
        {
            assert(w_damping.size() == dof_);
            QP_mani_IK_->setWeight(link_w_tracking, w_damping);
        }

        void RobotController::setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, const VectorXd& w_vel_damping, const VectorXd& w_acc_damping)
        {
            QP_mani_ID_->setWeight(link_w_tracking, w_vel_damping, w_acc_damping);
        }

        VectorXd RobotController::moveJointPositionCubic(const VectorXd& q_target,
                                                         const VectorXd& qdot_target,
                                                         const VectorXd& q_init,
                                                         const VectorXd& qdot_init,
                                                         const double& current_time,
                                                         const double& init_time,
                                                         const double& duration)
        {
            const VectorXd q_desired = DyrosMath::cubicVector(current_time,
                                                              init_time,
                                                              init_time + duration,
                                                              q_init,
                                                              q_target,
                                                              qdot_init,
                                                              qdot_target);
    
            return q_desired;
        }

        VectorXd RobotController::moveJointVelocityCubic(const VectorXd& q_target,
                                                         const VectorXd& qdot_target,
                                                         const VectorXd& q_init,
                                                         const VectorXd& qdot_init,
                                                         const double& current_time,
                                                         const double& init_time,
                                                         const double& duration)
        {
            const VectorXd qdot_desired =  DyrosMath::cubicDotVector(current_time,
                                                                     init_time,
                                                                     init_time + duration,
                                                                     q_init,
                                                                     q_target,
                                                                     qdot_init,
                                                                     qdot_target);
    
            return qdot_desired;
        }

        VectorXd RobotController::moveJointTorqueStep(const VectorXd& qddot_target, const bool use_mass)
        {
            if(use_mass) return robot_data_->getMassMatrix() * qddot_target + robot_data_->getGravity();
            else         return qddot_target + robot_data_->getGravity();
        }

        VectorXd RobotController::moveJointTorqueStep(const VectorXd& q_target,
                                                      const VectorXd& qdot_target,
                                                      const bool use_mass)
        {
            const VectorXd qddot_desired = Kp_joint_.asDiagonal() * (q_target - robot_data_->getJointPosition()) + Kv_joint_.asDiagonal() * (qdot_target - robot_data_->getJointVelocity());
            return moveJointTorqueStep(qddot_desired, use_mass);
        }
    
        VectorXd RobotController::moveJointTorqueCubic(const VectorXd& q_target,
                                                       const VectorXd& qdot_target,
                                                       const VectorXd& q_init,
                                                       const VectorXd& qdot_init,
                                                       const double& current_time,
                                                       const double& init_time,
                                                       const double& duration,
                                                       const bool use_mass)
        {
            const VectorXd q_desired = DyrosMath::cubicVector(current_time,
                                                              init_time,
                                                              init_time + duration,
                                                              q_init,
                                                              q_target,
                                                              qdot_init,
                                                              qdot_target);
    
            const VectorXd qdot_desired = DyrosMath::cubicDotVector(current_time,
                                                                    init_time,
                                                                    init_time + duration,
                                                                    q_init,
                                                                    q_target,
                                                                    qdot_init,
                                                                    qdot_target);
    
            return moveJointTorqueStep(q_desired, qdot_desired, use_mass);
        }
    
        VectorXd RobotController::CLIK(const std::map<std::string, Vector6d>& link_xdot_target, const VectorXd& null_qdot)
        {
            MatrixXd J_total;
            J_total.setZero(6*link_xdot_target.size(), dof_);

            VectorXd x_dot_target_total;
            x_dot_target_total.setZero(6*link_xdot_target.size());

            int i=0;
            for (auto &[link_name, xdot_target] : link_xdot_target)
            {
                J_total.block(6*i, 0, 6, dof_) = robot_data_->getJacobian(link_name);
                x_dot_target_total.segment(6*i, 6) = xdot_target;
                i++;
            }

            const MatrixXd J_total_pinv = DyrosMath::PinvCOD(J_total);
            const MatrixXd null_proj = MatrixXd::Identity(dof_, dof_) - J_total_pinv * J_total;
    
            return J_total_pinv * x_dot_target_total + null_proj * null_qdot;
        }

        VectorXd RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data, const VectorXd& null_qdot)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
            return CLIK(link_xdot_target, null_qdot);
        }
        
        VectorXd RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data)
        {
            return CLIK(link_task_data, VectorXd::Zero(dof_));
        }
    
        VectorXd RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const VectorXd& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes(); 
                auto iter = link_Kp_task_.find(link_name);
                if(iter != link_Kp_task_.end()) Kp_task = iter->second;

                link_task_data_result[link_name].xdot_desired = Kp_task.asDiagonal() * x_error + task_data.xdot_desired;
            }

            return CLIK(link_task_data_result, null_qdot);
        }
    
        VectorXd RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data)
        {
            return CLIKStep(link_task_data, VectorXd::Zero(dof_));
        }

        VectorXd RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration,
                                            const VectorXd& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return CLIKStep(link_task_data_result, null_qdot);
        }
        
        VectorXd RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration)
        {
            return CLIKCubic(link_task_data, current_time, init_time, duration, VectorXd::Zero(dof_));
        }

        VectorXd RobotController::OSF(const std::map<std::string, Vector6d>& link_xddot_target, const VectorXd& null_torque)
        {
            MatrixXd J_total;
            J_total.setZero(6*link_xddot_target.size(), dof_);

            VectorXd x_ddot_target_total;
            x_ddot_target_total.setZero(6*link_xddot_target.size());

            int i=0;
            for (auto &[link_name, xddot_target] : link_xddot_target)
            {
                J_total.block(6*i, 0, 6, dof_) = robot_data_->getJacobian(link_name);
                x_ddot_target_total.segment(6*i, 6) = xddot_target;
                i++;
            }

            const MatrixXd J_total_T = J_total.transpose();
            const MatrixXd M_inv = robot_data_->getMassMatrixInv();
    
            const MatrixXd M_task_total = DyrosMath::PinvCOD(J_total * M_inv * J_total_T); 
            const MatrixXd J_total_T_pinv = M_task_total * J_total * M_inv;
            const MatrixXd null_proj = MatrixXd::Identity(dof_, dof_) - (J_total_T * J_total_T_pinv);
            
            const VectorXd force_desired = M_task_total * x_ddot_target_total;
            
            return J_total_T * force_desired + null_proj * null_torque + robot_data_->getGravity();
        }

        VectorXd RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data, 
                                      const VectorXd& null_torque)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }
            
            return OSF(link_xddot_target, null_torque);
        }
        
        VectorXd RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data)
        {
            return OSF(link_task_data, VectorXd::Zero(dof_));
        }

        VectorXd RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                          const VectorXd& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes(); 
                Vector6d Kv_task; Kv_task.setOnes(); 
                auto iter_kp = link_Kp_task_.find(link_name);
                if(iter_kp != link_Kp_task_.end()) Kp_task = iter_kp->second;
                auto iter_kv = link_Kv_task_.find(link_name);
                if(iter_kv != link_Kv_task_.end()) Kv_task = iter_kv->second;

                link_task_data_result[link_name].xddot_desired = Kp_task.asDiagonal() * x_error + Kv_task.asDiagonal() * xdot_error + task_data.xddot_desired;
            }
            return OSF(link_task_data_result, null_torque);
        }
        
        VectorXd RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data)
        {
            return OSFStep(link_task_data, VectorXd::Zero(dof_));
        }

        VectorXd RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const VectorXd& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }
            return OSFStep(link_task_data_result, null_torque);
        }
      
        VectorXd RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration)
        {
            return OSFCubic(link_task_data, current_time, init_time, duration, VectorXd::Zero(dof_));
        }

        VectorXd RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target, const bool time_verbose)
        {
            QP_mani_IK_->setDesiredTaskVel(link_xdot_target);
            VectorXd opt_qdot = VectorXd::Zero(dof_);
            QP::TimeDuration time_duration;
            if(!QP_mani_IK_->getOptJointVel(opt_qdot, time_duration))
            {
                std::cerr << "QP IK failed to compute optimal joint velocity." << std::endl;
                opt_qdot.setZero(dof_);
            }

            if(time_verbose)
            {
                std::cout << "============ QPIK Time Information ==============" << std::endl;
                std::cout << "Duration for total [ms]: " << (time_duration.set_qp + time_duration.set_solver + time_duration.solve_qp)*1000 << std::endl;
                std::cout << "\tDuration for set up QP problem [ms]: " << time_duration.set_qp*1000 << std::endl;
                std::cout << "\t\tDuration for set up cost [ms]      : " << time_duration.set_cost*1000 << std::endl;
                std::cout << "\t\tDuration for set up constraint [ms]: " << time_duration.set_constraint*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up bound [ms]: " << time_duration.set_bound*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up ineq [ms] : " << time_duration.set_ineq*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up eq [ms]   : " << time_duration.set_eq*1000 << std::endl;
                std::cout << "\tDuration for set up QP solver [ms] : " << time_duration.set_solver*1000 << std::endl;
                std::cout << "\tDuration for solve QP [ms]         : " << time_duration.solve_qp*1000 << std::endl;
                std::cout << "=================================================" << std::endl;
            }
        
            return opt_qdot;
        }

        VectorXd RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose)
        {  
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
        
            return QPIK(link_xdot_target, time_verbose);
        }

        VectorXd RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes(); 
                auto iter = link_Kp_task_.find(link_name);
                if(iter != link_Kp_task_.end()) Kp_task = iter->second;

                link_task_data_result[link_name].xdot_desired = Kp_task.asDiagonal() * x_error + task_data.xdot_desired;
            }
            return QPIK(link_task_data_result, time_verbose);
        }
    
        VectorXd RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration, 
                                            const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }
    
            return QPIKStep(link_task_data_result, time_verbose);
        }

        VectorXd RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target, const bool time_verbose)
        {
            QP_mani_ID_->setDesiredTaskAcc(link_xddot_target);
            VectorXd opt_qddot = VectorXd::Zero(dof_);
            VectorXd opt_torque = VectorXd::Zero(dof_);
            QP::TimeDuration time_duration;
            if(!QP_mani_ID_->getOptJoint(opt_qddot, opt_torque, time_duration))
            {
                std::cerr << "QP ID failed to compute optimal joint torque." << std::endl;
                opt_torque = robot_data_->getGravity();
            }

            if(time_verbose)
            {
                std::cout << "============ QPID Time Information ==============" << std::endl;
                std::cout << "Duration for total [ms]: " << (time_duration.set_qp + time_duration.set_solver + time_duration.solve_qp)*1000 << std::endl;
                std::cout << "\tDuration for set up QP problem [ms]: " << time_duration.set_qp*1000 << std::endl;
                std::cout << "\t\tDuration for set up cost [ms]      : " << time_duration.set_cost*1000 << std::endl;
                std::cout << "\t\tDuration for set up constraint [ms]: " << time_duration.set_constraint*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up bound [ms]: " << time_duration.set_bound*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up ineq [ms] : " << time_duration.set_ineq*1000 << std::endl;
                std::cout << "\t\t\tDuration for set up eq [ms]   : " << time_duration.set_eq*1000 << std::endl;
                std::cout << "\tDuration for set up QP solver [ms] : " << time_duration.set_solver*1000 << std::endl;
                std::cout << "\tDuration for solve QP [ms]         : " << time_duration.solve_qp*1000 << std::endl;
                std::cout << "=================================================" << std::endl;
            }
        
            return opt_torque;
        }

        VectorXd RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }

            return QPID(link_xddot_target, time_verbose);
        }

        VectorXd RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data, const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes(); 
                Vector6d Kv_task; Kv_task.setOnes(); 
                auto iter_kp = link_Kp_task_.find(link_name);
                if(iter_kp != link_Kp_task_.end()) Kp_task = iter_kp->second;
                auto iter_kv = link_Kv_task_.find(link_name);
                if(iter_kv != link_Kv_task_.end()) Kv_task = iter_kv->second;

                link_task_data_result[link_name].xddot_desired = Kp_task.asDiagonal() * x_error + Kv_task.asDiagonal() * xdot_error + task_data.xddot_desired;
            }
            
            return QPID(link_task_data_result, time_verbose);
        }
    
    
        VectorXd RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration, 
                                            const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }
    
            return QPIDStep(link_task_data_result, time_verbose);
        }
    } // namespace Manipulator
} // namespace drc
