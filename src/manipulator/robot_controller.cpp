#include "dyros_robot_controller/manipulator/robot_controller.h"
#include <iostream>
#include <sstream>

namespace
{
    std::string formatQPTimeInfo(const std::string& qp_name, const drc::QP::TimeDuration& time_duration)
    {
        std::ostringstream oss;
        oss << "============ " << qp_name << " Time Information ==============\n";
        oss << "Duration for total [ms]: " << (time_duration.set_qp + time_duration.set_solver + time_duration.solve_qp) * 1000 << "\n";
        oss << "\tDuration for set up QP problem [ms]: " << time_duration.set_qp * 1000 << "\n";
        oss << "\t\tDuration for set up cost [ms]      : " << time_duration.set_cost * 1000 << "\n";
        oss << "\t\tDuration for set up constraint [ms]: " << time_duration.set_constraint * 1000 << "\n";
        oss << "\t\t\tDuration for set up bound [ms]: " << time_duration.set_bound * 1000 << "\n";
        oss << "\t\t\tDuration for set up ineq [ms] : " << time_duration.set_ineq * 1000 << "\n";
        oss << "\t\t\tDuration for set up eq [ms]   : " << time_duration.set_eq * 1000 << "\n";
        oss << "\tDuration for set up QP solver [ms] : " << time_duration.set_solver * 1000 << "\n";
        oss << "\tDuration for solve QP [ms]         : " << time_duration.solve_qp * 1000 << "\n";
        oss << "=================================================";
        return oss.str();
    }

    void printQPTimeInfoIfEnabled(const bool time_verbose, const std::string& verbose)
    {
        if(time_verbose && !verbose.empty())
        {
            std::cout << verbose << std::endl;
        }
    }
}

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

            link_IK_Kp_task_.clear();
            link_ID_Kp_task_.clear();
            link_ID_Kv_task_.clear();
    
            QP_mani_IK_ = std::make_unique<Manipulator::QPIK>(robot_data_, dt_);
            QP_mani_ID_ = std::make_unique<Manipulator::QPID>(robot_data_, dt_);
        }
    
        void RobotController::setJointGain(const Eigen::Ref<const VectorXd>& Kp, const Eigen::Ref<const VectorXd>& Kv)
        {
            assert(Kp.size() == dof_ && Kv.size() == dof_);
            Kp_joint_ = Kp;
            Kv_joint_ = Kv;
        }

        void RobotController::setJointKpGain(const Eigen::Ref<const VectorXd>& Kp)
        {
            assert(Kp.size() == dof_);
            Kp_joint_ = Kp;
        }

        void RobotController::setJointKvGain(const Eigen::Ref<const VectorXd>& Kv)
        {
            assert(Kv.size() == dof_);
            Kv_joint_ = Kv;
        }

        void RobotController::setIKGain(const std::map<std::string, Vector6d>& link_Kp)
        {
            for(const auto& [link_name, Kp] :link_Kp)
            {
                if(!robot_data_->hasLinkFrame(link_name))
                {
                    std::cerr << "\033[1;31m" << "Warn: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                }
                else
                {
                    link_IK_Kp_task_[link_name] = Kp;
                }
            }
        }

        void RobotController::setIKGain(const Vector6d& Kp)
        {
            std::map<std::string, Vector6d> link_IK_Kp_task;
            for(const auto& link_name : robot_data_->getLinkFrameVector())
            {
                link_IK_Kp_task[link_name] = Kp;
            }
            setIKGain(link_IK_Kp_task);
        }

        void RobotController::setIDGain(const std::map<std::string, Vector6d>& link_Kp, const std::map<std::string, Vector6d>& link_Kv)
        {
            setIDKpGain(link_Kp);
            setIDKvGain(link_Kv);
        }

        void RobotController::setIDGain(const Vector6d& Kp, const Vector6d& Kv)
        {
            setIDKpGain(Kp);
            setIDKvGain(Kv);
        }

        void RobotController::setIDKpGain(const std::map<std::string, Vector6d>& link_Kp)
        {
            for(const auto& [link_name, Kp] :link_Kp)
            {
                if(!robot_data_->hasLinkFrame(link_name))
                {
                    std::cerr << "\033[1;31m" << "Warn: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                }
                else
                {
                    link_ID_Kp_task_[link_name] = Kp;
                }
            }
        }

        void RobotController::setIDKpGain(const Vector6d& Kp)
        {
            std::map<std::string, Vector6d> link_ID_Kp_task;
            for(const auto& link_name : robot_data_->getLinkFrameVector())
            {
                link_ID_Kp_task[link_name] = Kp;
            }
            setIDKpGain(link_ID_Kp_task);
        }

        void RobotController::setIDKvGain(const std::map<std::string, Vector6d>& link_Kv)
        {
            for(const auto& [link_name, Kv] :link_Kv)
            {
                if(!robot_data_->hasLinkFrame(link_name))
                {
                    std::cerr << "\033[1;31m" << "Warn: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                }
                else
                {
                    link_ID_Kv_task_[link_name] = Kv;
                }
            }
        }

        void RobotController::setIDKvGain(const Vector6d& Kv)
        {
            std::map<std::string, Vector6d> link_ID_Kv_task;
            for(const auto& link_name : robot_data_->getLinkFrameVector())
            {
                link_ID_Kv_task[link_name] = Kv;
            }
            setIDKvGain(link_ID_Kv_task);
        }

        void RobotController::setQPIKTrackingGain(const std::map<std::string, Vector6d>& link_w_tracking)
        {
            QP_mani_IK_->setTrackingWeight(link_w_tracking);
        }

        void RobotController::setQPIKTrackingGain(const Vector6d& w_tracking)
        {
            QP_mani_IK_->setTrackingWeight(w_tracking);
        }

        void RobotController::setQPIKJointVelGain(const Eigen::Ref<const VectorXd>& w_vel_damping)
        {
            assert(w_vel_damping.size() == dof_);
            QP_mani_IK_->setJointVelWeight(w_vel_damping);
        }

        void RobotController::setQPIKJointAccGain(const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            assert(w_acc_damping.size() == dof_);
            QP_mani_IK_->setJointAccWeight(w_acc_damping);
        }
        
        void RobotController::setQPIKGain(const Vector6d& w_tracking,
                                          const Eigen::Ref<const VectorXd>& w_vel_damping,
                                          const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            assert(w_vel_damping.size() == dof_);
            assert(w_acc_damping.size() == dof_);
            QP_mani_IK_->setWeight(w_tracking, w_vel_damping, w_acc_damping);
        }

        void RobotController::setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking,
                                          const Eigen::Ref<const VectorXd>& w_vel_damping,
                                          const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            assert(w_vel_damping.size() == dof_);
            assert(w_acc_damping.size() == dof_);
            QP_mani_IK_->setWeight(link_w_tracking, w_vel_damping, w_acc_damping);
        }

        void RobotController::setQPIDTrackingGain(const Vector6d& w_tracking)
        {
            QP_mani_ID_->setTrackingWeight(w_tracking);
        }

        void RobotController::setQPIDTrackingGain(const std::map<std::string, Vector6d>& link_w_tracking)
        {
            QP_mani_ID_->setTrackingWeight(link_w_tracking);
        }

        void RobotController::setQPIDJointVelGain(const Eigen::Ref<const VectorXd>& w_vel_damping)
        {
            QP_mani_ID_->setJointVelWeight(w_vel_damping);
        }

        void RobotController::setQPIDJointAccGain(const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            QP_mani_ID_->setJointAccWeight(w_acc_damping);
        }

        void RobotController::setQPIDGain(const Vector6d& w_tracking, const Eigen::Ref<const VectorXd>& w_vel_damping, const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            QP_mani_ID_->setWeight(w_tracking, w_vel_damping, w_acc_damping);
        }
        
        void RobotController::setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, const Eigen::Ref<const VectorXd>& w_vel_damping, const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            QP_mani_ID_->setWeight(link_w_tracking, w_vel_damping, w_acc_damping);
        }

        VectorXd RobotController::moveJointPositionCubic(const Eigen::Ref<const VectorXd>& q_target,
                                                         const Eigen::Ref<const VectorXd>& qdot_target,
                                                         const Eigen::Ref<const VectorXd>& q_init,
                                                         const Eigen::Ref<const VectorXd>& qdot_init,
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

        VectorXd RobotController::moveJointVelocityCubic(const Eigen::Ref<const VectorXd>& q_target,
                                                         const Eigen::Ref<const VectorXd>& qdot_target,
                                                         const Eigen::Ref<const VectorXd>& q_init,
                                                         const Eigen::Ref<const VectorXd>& qdot_init,
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

        VectorXd RobotController::moveJointTorqueStep(const Eigen::Ref<const VectorXd>& qddot_target, const bool use_mass)
        {
            if(use_mass) return robot_data_->getMassMatrix() * qddot_target + robot_data_->getGravity();
            else         return qddot_target + robot_data_->getGravity();
        }

        VectorXd RobotController::moveJointTorqueStep(const Eigen::Ref<const VectorXd>& q_target,
                                                      const Eigen::Ref<const VectorXd>& qdot_target,
                                                      const bool use_mass)
        {
            const VectorXd qddot_desired = Kp_joint_.asDiagonal() * (q_target - robot_data_->getJointPosition()) + Kv_joint_.asDiagonal() * (qdot_target - robot_data_->getJointVelocity());
            return moveJointTorqueStep(qddot_desired, use_mass);
        }
    
        VectorXd RobotController::moveJointTorqueCubic(const Eigen::Ref<const VectorXd>& q_target,
                                                       const Eigen::Ref<const VectorXd>& qdot_target,
                                                       const Eigen::Ref<const VectorXd>& q_init,
                                                       const Eigen::Ref<const VectorXd>& qdot_init,
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
    
        bool RobotController::CLIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   const Eigen::Ref<const VectorXd>& null_qdot)
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

            opt_qdot.noalias() = J_total_pinv * x_dot_target_total + null_proj * null_qdot;
            return true;
        }

        bool RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
            return CLIK(link_xdot_target, opt_qdot, null_qdot);
        }

        bool RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot)
        {
            VectorXd null_qdot = VectorXd::Zero(dof_);
            return CLIK(link_task_data, opt_qdot, null_qdot);
        }

        bool RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                       const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes();
                auto iter = link_IK_Kp_task_.find(link_name);
                if(iter != link_IK_Kp_task_.end()) Kp_task = iter->second;

                link_task_data_result[link_name].xdot_desired = Kp_task.asDiagonal() * x_error + task_data.xdot_desired;
            }

            return CLIK(link_task_data_result, opt_qdot, null_qdot);
        }

        bool RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot)
        {
            VectorXd null_qdot = VectorXd::Zero(dof_);
            return CLIKStep(link_task_data, opt_qdot, null_qdot);
        }

        bool RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                        const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return CLIKStep(link_task_data_result, opt_qdot, null_qdot);
        }

        bool RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot)
        {
            VectorXd null_qdot = VectorXd::Zero(dof_);
            return CLIKCubic(link_task_data, current_time, duration, opt_qdot, null_qdot);
        }

        bool RobotController::OSF(const std::map<std::string, Vector6d>& link_xddot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque,
                                  const Eigen::Ref<const VectorXd>& null_torque)
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

            opt_torque.noalias() = J_total_T * force_desired + null_proj * null_torque + robot_data_->getGravity();
            return true;
        }

        bool RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque,
                                  const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }

            return OSF(link_xddot_target, opt_torque, null_torque);
        }

        bool RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque)
        {
            VectorXd null_torque = VectorXd::Zero(dof_);
            return OSF(link_task_data, opt_torque, null_torque);
        }

        bool RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque,
                                      const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes();
                Vector6d Kv_task; Kv_task.setOnes();
                auto iter_kp = link_ID_Kp_task_.find(link_name);
                if(iter_kp != link_ID_Kp_task_.end()) Kp_task = iter_kp->second;
                auto iter_kv = link_ID_Kv_task_.find(link_name);
                if(iter_kv != link_ID_Kv_task_.end()) Kv_task = iter_kv->second;

                link_task_data_result[link_name].xddot_desired = Kp_task.asDiagonal() * x_error + Kv_task.asDiagonal() * xdot_error + task_data.xddot_desired;
            }
            return OSF(link_task_data_result, opt_torque, null_torque);
        }

        bool RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque)
        {
            VectorXd null_torque = VectorXd::Zero(dof_);
            return OSFStep(link_task_data, opt_torque, null_torque);
        }

        bool RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque,
                                       const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }
            return OSFStep(link_task_data_result, opt_torque, null_torque);
        }

        bool RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque)
        {
            VectorXd null_torque = VectorXd::Zero(dof_);
            return OSFCubic(link_task_data, current_time, duration, opt_torque, null_torque);
        }

        bool RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   std::string& time_verbose)
        {
            time_verbose.clear();
            if(opt_qdot.size() != dof_)
            {
                std::cerr << "Size of opt_qdot(" << opt_qdot.size() << ") is not same as dof_(" << dof_ << ")" << std::endl;
                return false;
            }
            opt_qdot.setZero();

            QP_mani_IK_->setDesiredTaskVel(link_xdot_target);
            QP::TimeDuration time_duration;
            const bool qp_success = QP_mani_IK_->getOptJointVel(opt_qdot, time_duration);
            if(!qp_success)
            {
                opt_qdot.setZero();
            }

            time_verbose = formatQPTimeInfo("QPIK", time_duration);
            return qp_success;
        }

        bool RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIK(link_xdot_target, opt_qdot, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   std::string& time_verbose)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }

            return QPIK(link_xdot_target, opt_qdot, time_verbose);
        }

        bool RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIK(link_task_data, opt_qdot, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                       std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes();
                auto iter = link_IK_Kp_task_.find(link_name);
                if(iter != link_IK_Kp_task_.end()) Kp_task = iter->second;

                link_task_data_result[link_name].xdot_desired = Kp_task.asDiagonal() * x_error + task_data.xdot_desired;
            }
            return QPIK(link_task_data_result, opt_qdot, time_verbose);
        }

        bool RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                       const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIKStep(link_task_data, opt_qdot, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                        std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIKStep(link_task_data_result, opt_qdot, time_verbose);
        }

        bool RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot,
                                        const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIKCubic(link_task_data, current_time, duration, opt_qdot, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque,
                                   std::string& time_verbose)
        {
            time_verbose.clear();
            QP_mani_ID_->setDesiredTaskAcc(link_xddot_target);
            VectorXd opt_qddot = VectorXd::Zero(dof_);
            if(opt_torque.size() != dof_)
            {
                std::cerr << "Size of opt_torque(" << opt_torque.size() << ") is not same as dof_(" << dof_ << ")" << std::endl;
                return false;
            }
            opt_torque.setZero();
            QP::TimeDuration time_duration;
            const bool qp_success = QP_mani_ID_->getOptJoint(opt_qddot, opt_torque, time_duration);
            if(!qp_success)
            {
                opt_torque = robot_data_->getGravity();
            }

            time_verbose = formatQPTimeInfo("QPID", time_duration);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPID(link_xddot_target, opt_torque, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque,
                                   std::string& time_verbose)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }

            return QPID(link_xddot_target, opt_torque, time_verbose);
        }

        bool RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPID(link_task_data, opt_torque, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque,
                                       std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes();
                Vector6d Kv_task; Kv_task.setOnes();
                auto iter_kp = link_ID_Kp_task_.find(link_name);
                if(iter_kp != link_ID_Kp_task_.end()) Kp_task = iter_kp->second;
                auto iter_kv = link_ID_Kv_task_.find(link_name);
                if(iter_kv != link_ID_Kv_task_.end()) Kv_task = iter_kv->second;

                link_task_data_result[link_name].xddot_desired = Kp_task.asDiagonal() * x_error + Kv_task.asDiagonal() * xdot_error + task_data.xddot_desired;
            }

            return QPID(link_task_data_result, opt_torque, time_verbose);
        }

        bool RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque,
                                       const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIDStep(link_task_data, opt_torque, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_torque,
                                        std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIDStep(link_task_data_result, opt_torque, time_verbose);
        }

        bool RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_torque,
                                        const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIDCubic(link_task_data, current_time, duration, opt_torque, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }
    } // namespace Manipulator
} // namespace drc
