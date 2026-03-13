#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"
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
    namespace MobileManipulator
    {
        RobotController::RobotController(std::shared_ptr<MobileManipulator::RobotData> robot_data)
        : Mobile::RobotController(std::static_pointer_cast<Mobile::RobotData>(robot_data))
        , dt_(robot_data->getDt())
        , robot_data_(std::move(robot_data))
        {
            dof_ = robot_data_->getDof();
            mani_dof_ = robot_data_->getManipulatorDof();
            mobi_dof_ = robot_data_->getMobileDof();
            actuator_dof_ = mani_dof_ + mobi_dof_;
            Kp_mani_joint_ = VectorXd::Constant(mani_dof_, 400);
            Kv_mani_joint_ = VectorXd::Constant(mani_dof_, 40);

            link_IK_Kp_task_.clear();
            link_ID_Kp_task_.clear();
            link_ID_Kv_task_.clear();
            
            QP_moma_IK_ = std::make_unique<MobileManipulator::QPIK>(robot_data_, dt_);
            QP_moma_ID_ = std::make_unique<MobileManipulator::QPID>(robot_data_, dt_);
        }

        void RobotController::setManipulatorJointGain(const Eigen::Ref<const VectorXd>& Kp, const Eigen::Ref<const VectorXd>& Kv)
        {
            assert(Kp.size() == mani_dof_ && Kv.size() != mani_dof_);
            Kp_mani_joint_ = Kp;
            Kv_mani_joint_ = Kv;
        }

        void RobotController::setManipulatorJointKpGain(const Eigen::Ref<const VectorXd>& Kp)
        {
            assert(Kp.size() == mani_dof_);
            Kp_mani_joint_ = Kp;
        }

        void RobotController::setManipulatorJointKvGain(const Eigen::Ref<const VectorXd>& Kv)
        {
            assert(Kv.size() == mani_dof_);
            Kv_mani_joint_ = Kv;
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

        void RobotController::setIDGain(const std::map<std::string, Vector6d>& link_Kp,
                                        const std::map<std::string, Vector6d>& link_Kv)
        {
            setIDKpGain(link_Kp);
            setIDKvGain(link_Kv);
        }

        void RobotController::setIDGain(const Vector6d& Kp,
                                        const Vector6d& Kv)
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
            QP_moma_IK_->setTrackingWeight(link_w_tracking);
        }

        void RobotController::setQPIKTrackingGain(const Vector6d& w_tracking)
        {
            QP_moma_IK_->setTrackingWeight(w_tracking);
        }

        void RobotController::setQPIKManiJointVelGain(const Eigen::Ref<const VectorXd>& w_mani_vel_damping)
        {
            QP_moma_IK_->setManiJointVelWeight(w_mani_vel_damping);
        }

        void RobotController::setQPIKManiJointAccGain(const Eigen::Ref<const VectorXd>& w_mani_acc_damping)
        {
            QP_moma_IK_->setManiJointAccWeight(w_mani_acc_damping);
        }

        void RobotController::setQPIKBaseVelGain(const Eigen::Vector3d& w_base_vel_damping)
        {
            QP_moma_IK_->setBaseVelWeight(w_base_vel_damping);
        }

        void RobotController::setQPIKBaseAccGain(const Eigen::Vector3d& w_base_acc_damping)
        {
            QP_moma_IK_->setBaseAccWeight(w_base_acc_damping);
        }

        void RobotController::setQPIKGain(const Vector6d& w_tracking, 
                                         const Eigen::Ref<const VectorXd>& w_mani_vel_damping,
                                         const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                         const Eigen::Vector3d& w_base_vel_damping,
                                         const Eigen::Vector3d& w_base_acc_damping)
        {
            assert(w_mani_vel_damping.size() == mani_dof_);
            assert(w_mani_acc_damping.size() == mani_dof_);
            QP_moma_IK_->setWeight(w_tracking, w_mani_vel_damping, w_mani_acc_damping, w_base_vel_damping, w_base_acc_damping);
        }

        void RobotController::setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking, 
                                         const Eigen::Ref<const VectorXd>& w_mani_vel_damping,
                                         const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                         const Eigen::Vector3d& w_base_vel_damping,
                                         const Eigen::Vector3d& w_base_acc_damping)
        {
            assert(w_mani_vel_damping.size() == mani_dof_);
            assert(w_mani_acc_damping.size() == mani_dof_);
            QP_moma_IK_->setWeight(link_w_tracking, w_mani_vel_damping, w_mani_acc_damping, w_base_vel_damping, w_base_acc_damping);
        }

        void RobotController::setQPIDTrackingGain(const Vector6d& w_tracking)
        {
            QP_moma_ID_->setTrackingWeight(w_tracking);
        }

        void RobotController::setQPIDTrackingGain(const std::map<std::string, Vector6d>& link_w_tracking)
        {
            QP_moma_ID_->setTrackingWeight(link_w_tracking);
        }

        void RobotController::setQPIDManiJointVelGain(const Eigen::Ref<const VectorXd>& w_mani_vel_damping)
        {
            QP_moma_ID_->setManiJointVelWeight(w_mani_vel_damping);
        }

        void RobotController::setQPIDManiJointAccGain(const Eigen::Ref<const VectorXd>& w_mani_acc_damping)
        {
            QP_moma_ID_->setManiJointAccWeight(w_mani_acc_damping);
        }

        void RobotController::setQPIDBaseVelGain(const Eigen::Vector3d& w_base_vel_damping)
        {
            QP_moma_ID_->setBaseVelWeight(w_base_vel_damping);
        }

        void RobotController::setQPIDBaseAccGain(const Eigen::Vector3d& w_base_acc_damping)
        {
            QP_moma_ID_->setBaseAccWeight(w_base_acc_damping);
        }

        void RobotController::setQPIDGain(const Vector6d& w_tracking, 
                                          const Eigen::Ref<const VectorXd>& w_mani_vel_damping, 
                                          const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                          const Eigen::Vector3d& w_base_vel_damping,
                                          const Eigen::Vector3d& w_base_acc_damping)
        {
            assert(w_mani_vel_damping.size() == mani_dof_);
            assert(w_mani_acc_damping.size() == mani_dof_);
            QP_moma_ID_->setWeight(w_tracking, w_mani_vel_damping, w_mani_acc_damping, w_base_vel_damping, w_base_acc_damping);
        }

        void RobotController::setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, 
                                          const Eigen::Ref<const VectorXd>& w_mani_vel_damping, 
                                          const Eigen::Ref<const VectorXd>& w_mani_acc_damping,
                                          const Eigen::Vector3d& w_base_vel_damping,
                                          const Eigen::Vector3d& w_base_acc_damping)
        {
            assert(w_mani_vel_damping.size() == mani_dof_);
            assert(w_mani_acc_damping.size() == mani_dof_);
            QP_moma_ID_->setWeight(link_w_tracking, w_mani_vel_damping, w_mani_acc_damping, w_base_vel_damping, w_base_acc_damping);
        }

        VectorXd RobotController::computeMobileWheelVel(const Vector3d& base_vel)
        {
            return Mobile::RobotController::computeWheelVel(base_vel);
        }

        MatrixXd RobotController::computeMobileIKJacobian()
        {
            return Mobile::RobotController::computeIKJacobian();
        }

        VectorXd RobotController::MobileVelocityCommand(const Vector3d& desired_base_vel)
        {
            return Mobile::RobotController::VelocityCommand(desired_base_vel);
        }

        VectorXd RobotController::moveManipulatorJointPositionCubic(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                    const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                    const Eigen::Ref<const VectorXd>& q_mani_init,
                                                                    const Eigen::Ref<const VectorXd>& qdot_mani_init,
                                                                    const double& current_time,
                                                                    const double& init_time,
                                                                    const double& duration)
        {
            assert(q_mani_target.size() == mani_dof_ && 
                   qdot_mani_target.size() == mani_dof_ &&
                   q_mani_init.size() == mani_dof_ &&
                   qdot_mani_init.size() == mani_dof_);

            const VectorXd q_mani_desired =  DyrosMath::cubicVector(current_time,
                                                                    init_time,
                                                                    init_time + duration,
                                                                    q_mani_init,
                                                                    q_mani_target,
                                                                    qdot_mani_init,
                                                                    qdot_mani_target);
            return q_mani_desired;
        }

        VectorXd RobotController::moveManipulatorJointVelocityCubic(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                    const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                    const Eigen::Ref<const VectorXd>& q_mani_init,
                                                                    const Eigen::Ref<const VectorXd>& qdot_mani_init,
                                                                    const double& current_time,
                                                                    const double& init_time,
                                                                    const double& duration)
        {
            assert(q_mani_target.size() == mani_dof_ && 
                   qdot_mani_target.size() == mani_dof_ &&
                   q_mani_init.size() == mani_dof_ &&
                   qdot_mani_init.size() == mani_dof_);

            const VectorXd qdot_mani_desired =  DyrosMath::cubicDotVector(current_time,
                                                                          init_time,
                                                                          init_time + duration,
                                                                          q_mani_init,
                                                                          q_mani_target,
                                                                          qdot_mani_init,
                                                                          qdot_mani_target);
            return qdot_mani_desired;
        }

        VectorXd RobotController::moveManipulatorJointTorqueStep(const Eigen::Ref<const VectorXd>& qddot_mani_target, const bool use_mass)
        {
            assert(qddot_mani_target.size() == mani_dof_);
            const MatrixXd M_mani = robot_data_->getMassMatrix().block(robot_data_->getJointIndex().mani_start,robot_data_->getJointIndex().mani_start,mani_dof_,mani_dof_);
            const MatrixXd g_mani = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            if(use_mass) return M_mani * qddot_mani_target + g_mani;
            else         return qddot_mani_target + g_mani;
        }

        VectorXd RobotController::moveManipulatorJointTorqueStep(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                 const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                 const bool use_mass)
        {
            const VectorXd q_mani = robot_data_->getJointPosition().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            const VectorXd qdot_mani = robot_data_->getJointVelocity().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            const VectorXd qddot_mani_desired = Kp_mani_joint_.asDiagonal() * (q_mani_target - q_mani) + Kv_mani_joint_.asDiagonal() * (qdot_mani_target - qdot_mani);
            return moveManipulatorJointTorqueStep(qddot_mani_desired, use_mass);
        }

        VectorXd RobotController::moveManipulatorJointTorqueCubic(const Eigen::Ref<const VectorXd>& q_mani_target,
                                                                  const Eigen::Ref<const VectorXd>& qdot_mani_target,
                                                                  const Eigen::Ref<const VectorXd>& q_mani_init,
                                                                  const Eigen::Ref<const VectorXd>& qdot_mani_init,
                                                                  const double& current_time,
                                                                  const double& init_time,
                                                                  const double& duration,
                                                                  const bool use_mass)
        {
            const VectorXd q_mani_desired =  DyrosMath::cubicVector(current_time,
                                                                    init_time,
                                                                    init_time + duration,
                                                                    q_mani_init,
                                                                    q_mani_target,
                                                                    qdot_mani_init,
                                                                    qdot_mani_target);
    
            const VectorXd qdot_mani_desired =  DyrosMath::cubicDotVector(current_time,
                                                                          init_time,
                                                                          init_time + duration,
                                                                          q_mani_init,
                                                                          q_mani_target,
                                                                          qdot_mani_init,
                                                                          qdot_mani_target);
    
            return moveManipulatorJointTorqueStep(q_mani_desired, qdot_mani_desired, use_mass);
        }

        bool RobotController::CLIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const Eigen::Ref<const VectorXd>& null_qdot)
        {
            if(opt_qdot_mobile.size() != mobi_dof_ || opt_qdot_manipulator.size() != mani_dof_)
            {
                std::cerr << "Size of opt_qdot_mobile(" << opt_qdot_mobile.size() << ") or opt_qdot_manipulator(" << opt_qdot_manipulator.size()
                          << ") are not same as mobi_dof_(" << mobi_dof_ << ") and mani_dof_(" << mani_dof_ << ")" << std::endl;
                return false;
            }
            if(null_qdot.size() != actuator_dof_)
            {
                std::cerr << "Size of null_qdot(" << null_qdot.size() << ") is not same as actuator_dof_(" << actuator_dof_ << ")" << std::endl;
                return false;
            }
            if(link_xdot_target.empty())
            {
                opt_qdot_mobile.setZero();
                opt_qdot_manipulator.setZero();
                return false;
            }

            MatrixXd J_total;
            J_total.setZero(6 * link_xdot_target.size(), actuator_dof_);

            VectorXd x_dot_target_total;
            x_dot_target_total.setZero(6 * link_xdot_target.size());

            int i = 0;
            for (const auto& [link_name, xdot_target] : link_xdot_target)
            {
                J_total.block(6 * i, 0, 6, actuator_dof_) = robot_data_->getJacobianActuated(link_name);
                x_dot_target_total.segment(6 * i, 6) = xdot_target;
                ++i;
            }

            const MatrixXd J_total_pinv = DyrosMath::PinvCOD(J_total);
            const MatrixXd null_proj = MatrixXd::Identity(actuator_dof_, actuator_dof_) - J_total_pinv * J_total;

            VectorXd qdot_task(actuator_dof_);
            qdot_task.noalias() = J_total_pinv * x_dot_target_total;

            VectorXd qdot_null(actuator_dof_);
            qdot_null.noalias() = null_proj * null_qdot;

            const VectorXd qdot_total = qdot_task + qdot_null;

            opt_qdot_mobile = qdot_total.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_qdot_manipulator = qdot_total.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
            return true;
        }

        bool RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (const auto& [link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
            return CLIK(link_xdot_target, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::CLIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator)
        {
            const VectorXd null_qdot = VectorXd::Zero(actuator_dof_);
            return CLIK(link_task_data, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (const auto& [link_name, task_data] : link_task_data)
            {
                Vector6d x_error, xdot_error;
                DyrosMath::getTaskSpaceError(task_data.x_desired, task_data.xdot_desired, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);

                Vector6d Kp_task; Kp_task.setOnes();
                auto iter = link_IK_Kp_task_.find(link_name);
                if(iter != link_IK_Kp_task_.end()) Kp_task = iter->second;

                link_task_data_result[link_name].xdot_desired = Kp_task.asDiagonal() * x_error + task_data.xdot_desired;
            }

            return CLIK(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::CLIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator)
        {
            const VectorXd null_qdot = VectorXd::Zero(actuator_dof_);
            return CLIKStep(link_task_data, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                        const Eigen::Ref<const VectorXd>& null_qdot)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (const auto& [link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return CLIKStep(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::CLIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator)
        {
            const VectorXd null_qdot = VectorXd::Zero(actuator_dof_);
            return CLIKCubic(link_task_data, current_time, duration, opt_qdot_mobile, opt_qdot_manipulator, null_qdot);
        }

        bool RobotController::OSF(const std::map<std::string, Vector6d>& link_xddot_target,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  const Eigen::Ref<const VectorXd>& null_torque)
        {
            if(opt_qddot_mobile.size() != mobi_dof_ || opt_torque_manipulator.size() != mani_dof_)
            {
                std::cerr << "Size of opt_qddot_mobile(" << opt_qddot_mobile.size() << ") or opt_torque_manipulator(" << opt_torque_manipulator.size()
                          << ") are not same as mobi_dof_(" << mobi_dof_ << ") and mani_dof_(" << mani_dof_ << ")" << std::endl;
                return false;
            }
            if(null_torque.size() != actuator_dof_)
            {
                std::cerr << "Size of null_torque(" << null_torque.size() << ") is not same as actuator_dof_(" << actuator_dof_ << ")" << std::endl;
                return false;
            }
            if(link_xddot_target.empty())
            {
                opt_qddot_mobile.setZero();
                opt_torque_manipulator.setZero();
                return false;
            }

            MatrixXd J_total;
            J_total.setZero(6 * link_xddot_target.size(), actuator_dof_);

            VectorXd x_ddot_target_total;
            x_ddot_target_total.setZero(6 * link_xddot_target.size());

            int i = 0;
            for (const auto& [link_name, xddot_target] : link_xddot_target)
            {
                J_total.block(6 * i, 0, 6, actuator_dof_) = robot_data_->getJacobianActuated(link_name);
                x_ddot_target_total.segment(6 * i, 6) = xddot_target;
                ++i;
            }

            const MatrixXd J_total_T = J_total.transpose();
            const MatrixXd M_inv = robot_data_->getMassMatrixActuatedInv();

            const MatrixXd M_task_total = DyrosMath::PinvCOD(J_total * M_inv * J_total_T);
            const MatrixXd J_total_T_pinv = M_task_total * J_total * M_inv;
            const MatrixXd null_proj = MatrixXd::Identity(actuator_dof_, actuator_dof_) - (J_total_T * J_total_T_pinv);

            const VectorXd force_desired = M_task_total * x_ddot_target_total;
            const VectorXd torque_actuated = J_total_T * force_desired + null_proj * null_torque + robot_data_->getGravityActuated();

            const VectorXd qddot_actuated = M_inv * (torque_actuated - robot_data_->getNonlinearEffectsActuated());

            opt_qddot_mobile = qddot_actuated.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_torque_manipulator = torque_actuated.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
            return true;
        }

        bool RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                  const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (const auto& [link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }
            return OSF(link_xddot_target, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }

        bool RobotController::OSF(const std::map<std::string, TaskSpaceData>& link_task_data,
                                  Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                  Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator)
        {
            const VectorXd null_torque = VectorXd::Zero(actuator_dof_);
            return OSF(link_task_data, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }

        bool RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                      const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (const auto& [link_name, task_data] : link_task_data)
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
            return OSF(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }

        bool RobotController::OSFStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                      Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                      Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator)
        {
            const VectorXd null_torque = VectorXd::Zero(actuator_dof_);
            return OSFStep(link_task_data, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }

        bool RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                       const Eigen::Ref<const VectorXd>& null_torque)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (const auto& [link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return OSFStep(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }

        bool RobotController::OSFCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       const double& current_time,
                                       const double& duration,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator)
        {
            const VectorXd null_torque = VectorXd::Zero(actuator_dof_);
            return OSFCubic(link_task_data, current_time, duration, opt_qddot_mobile, opt_torque_manipulator, null_torque);
        }


        bool RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   std::string& time_verbose)
        {
            time_verbose.clear();
            if(opt_qdot_mobile.size() != mobi_dof_ || opt_qdot_manipulator.size() != mani_dof_)
            {
                std::cerr << "Size of opt_qdot_mobile(" << opt_qdot_mobile.size() << ") or opt_qdot_manipulator(" << opt_qdot_manipulator.size()
                          << ") are not same as mobi_dof_(" << mobi_dof_ << ") and mani_dof_(" << mani_dof_ << ")" << std::endl;
                return false;
            }
            opt_qdot_mobile.setZero();
            opt_qdot_manipulator.setZero();

            QP_moma_IK_->setDesiredTaskVel(link_xdot_target);
            VectorXd opt_qdot = VectorXd::Zero(actuator_dof_);
            QP::TimeDuration time_duration;
            const bool qp_success = QP_moma_IK_->getOptJointVel(opt_qdot, time_duration);
            if(!qp_success)
            {
                opt_qdot.setZero(dof_);
            }

            opt_qdot_mobile = opt_qdot.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_qdot_manipulator = opt_qdot.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
            time_verbose = formatQPTimeInfo("QPIK", time_duration);
            return qp_success;
        }

        bool RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIK(link_xdot_target, opt_qdot_mobile, opt_qdot_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   std::string& time_verbose)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
            return QPIK(link_xdot_target, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIK(link_task_data, opt_qdot_mobile, opt_qdot_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
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

            return QPIK(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIKStep(link_task_data, opt_qdot_mobile, opt_qdot_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                        std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIKStep(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                        const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIKCubic(link_task_data, current_time, duration, opt_qdot_mobile, opt_qdot_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   std::string& time_verbose)
        {
            time_verbose.clear();
            if(opt_qddot_mobile.size() != mobi_dof_ || opt_torque_manipulator.size() != mani_dof_)
            {
                std::cerr << "Size of opt_qddot_mobile(" << opt_qddot_mobile.size() << ") or opt_torque_manipulator(" << opt_torque_manipulator.size()
                          << ") are not same as mobi_dof_(" << mobi_dof_ << ") and mani_dof_(" << mani_dof_ << ")" << std::endl;
                return false;
            }
            opt_qddot_mobile.setZero();
            opt_torque_manipulator.setZero();

            QP_moma_ID_->setDesiredTaskAcc(link_xddot_target);
            VectorXd opt_qddot = VectorXd::Zero(actuator_dof_);
            VectorXd opt_torque = VectorXd::Zero(actuator_dof_);
            QP::TimeDuration time_duration;
            const bool qp_success = QP_moma_ID_->getOptJoint(opt_qddot, opt_torque, time_duration);
            if(!qp_success)
            {
                opt_torque = robot_data_->getGravityActuated();
                opt_qddot.setZero();
            }

            opt_qddot_mobile = opt_qddot.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_torque_manipulator = opt_torque.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
            time_verbose = formatQPTimeInfo("QPID", time_duration);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPID(link_xddot_target, opt_qddot_mobile, opt_torque_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   std::string& time_verbose)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }
            return QPID(link_xddot_target, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }

        bool RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPID(link_task_data, opt_qddot_mobile, opt_torque_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
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
            return QPID(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }

        bool RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                       const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIDStep(link_task_data, opt_qddot_mobile, opt_torque_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }

        bool RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                        std::string& time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, task_data.control_start_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIDStep(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }

        bool RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                        const bool time_verbose)
        {
            std::string time_verbose_str;
            const bool qp_success = QPIDCubic(link_task_data, current_time, duration, opt_qddot_mobile, opt_torque_manipulator, time_verbose_str);
            printQPTimeInfoIfEnabled(time_verbose, time_verbose_str);
            return qp_success;
        }
    } // namespace MobileManipulator
} // namespace drc
