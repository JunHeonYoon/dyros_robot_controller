#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

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

        void RobotController::setQPIKGain(const std::map<std::string, Vector6d>& link_w_tracking, const Eigen::Ref<const VectorXd>& w_damping)
        {
            assert(w_damping.size() == actuator_dof_);
            QP_moma_IK_->setWeight(link_w_tracking, w_damping);
        }

        void RobotController::setQPIDGain(const std::map<std::string, Vector6d>& link_w_tracking, const Eigen::Ref<const VectorXd>& w_vel_damping, const Eigen::Ref<const VectorXd>& w_acc_damping)
        {
            QP_moma_ID_->setWeight(link_w_tracking, w_vel_damping, w_acc_damping);
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


        bool RobotController::QPIK(const std::map<std::string, Vector6d>& link_xdot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const bool time_verbose)
        {
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
            return qp_success;
        }

        bool RobotController::QPIK(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                   const bool time_verbose)
        {
            std::map<std::string, Vector6d> link_xdot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xdot_target[link_name] = task_data.xdot_desired;
            }
            return QPIK(link_xdot_target, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPIKStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                       const bool time_verbose)
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

            return QPIK(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPIKCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& init_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_qdot_manipulator,
                                        const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIKStep(link_task_data_result, opt_qdot_mobile, opt_qdot_manipulator, time_verbose);
        }

        bool RobotController::QPID(const std::map<std::string, Vector6d>& link_xddot_target,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   const bool time_verbose)
        {
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
                opt_torque = robot_data_->getGravity();
                opt_qddot.setZero();
            }

            opt_qddot_mobile = opt_qddot.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_torque_manipulator = opt_torque.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);

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
            return qp_success;
        }

        bool RobotController::QPID(const std::map<std::string, TaskSpaceData>& link_task_data,
                                   Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                   Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                   const bool time_verbose)
        {
            std::map<std::string, Vector6d> link_xddot_target;
            for (auto &[link_name, task_data] : link_task_data)
            {
                link_xddot_target[link_name] = task_data.xddot_desired;
            }
            return QPID(link_xddot_target, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }

        bool RobotController::QPIDStep(const std::map<std::string, TaskSpaceData>& link_task_data,
                                       Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                       Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                       const bool time_verbose)
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
            return QPID(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }

        bool RobotController::QPIDCubic(const std::map<std::string, TaskSpaceData>& link_task_data,
                                        const double& current_time,
                                        const double& init_time,
                                        const double& duration,
                                        Eigen::Ref<Eigen::VectorXd> opt_qddot_mobile,
                                        Eigen::Ref<Eigen::VectorXd> opt_torque_manipulator,
                                        const bool time_verbose)
        {
            std::map<std::string, TaskSpaceData> link_task_data_result;
            for (auto &[link_name, task_data] : link_task_data)
            {
                TaskSpaceData task_data_result = task_data;
                DyrosMath::getTaskSpaceCubic(task_data.x_desired, task_data.xdot_desired, task_data.x_init, task_data.xdot_init, current_time, init_time, duration, task_data_result.x_desired, task_data_result.xdot_desired);
                link_task_data_result[link_name] = task_data_result;
            }

            return QPIDStep(link_task_data_result, opt_qddot_mobile, opt_torque_manipulator, time_verbose);
        }
    } // namespace MobileManipulator
} // namespace drc
