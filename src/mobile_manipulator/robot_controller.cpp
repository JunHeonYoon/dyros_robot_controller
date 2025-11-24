#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

namespace drc
{
    namespace MobileManipulator
    {
        RobotController::RobotController(const double& dt,
                                         std::shared_ptr<MobileManipulator::RobotData> robot_data)
        : Mobile::RobotController(dt, std::static_pointer_cast<Mobile::RobotData>(robot_data))
        , robot_data_(std::move(robot_data))
        {
            dof_ = robot_data_->getDof();
            mani_dof_ = robot_data_->getManipulatorDof();
            mobi_dof_ = robot_data_->getMobileDof();
            actuator_dof_ = mani_dof_ + mobi_dof_;
            Kp_task_ = VectorXd::Constant(6, 400);
            Kv_task_ = VectorXd::Constant(6, 40);
            Kp_mani_joint_ = VectorXd::Constant(mani_dof_, 400);
            Kv_mani_joint_ = VectorXd::Constant(mani_dof_, 40);
            
            QP_moma_IK_ = std::make_unique<MobileManipulator::QPIK>(robot_data_, dt_);
            QP_moma_ID_ = std::make_unique<MobileManipulator::QPID>(robot_data_, dt_);
        }

        void RobotController::setManipulatorJointGain(const VectorXd& Kp, const VectorXd& Kv)
        {
            if (Kp.size() != mani_dof_ || Kv.size() != mani_dof_)
            {
                throw std::runtime_error("Kp and Kv must be of size mani_dof_.");
            }
            Kp_mani_joint_ = Kp;
            Kv_mani_joint_ = Kv;
        }

        void RobotController::setManipulatorJointKpGain(const VectorXd& Kp)
        {
            if (Kp.size() != mani_dof_)
            {
                throw std::runtime_error("Kp must be of size mani_dof_.");
            }
            Kp_mani_joint_ = Kp;
        }

        void RobotController::setManipulatorJointKvGain(const VectorXd& Kv)
        {
            if (Kv.size() != mani_dof_)
            {
                throw std::runtime_error("Kv must be of size mani_dof_.");
            }
            Kv_mani_joint_ = Kv;
        }
    
        void RobotController::setTaskGain(const VectorXd& Kp, const VectorXd& Kv)
        {
            if (Kp.size() != 6 || Kv.size() != 6)
            {
                throw std::runtime_error("Kp and Kv must be of size 6.");
            }
            Kp_task_ = Kp;
            Kv_task_ = Kv;
        }

        void RobotController::setTaskKpGain(const VectorXd& Kp)
        {
            if (Kp.size() != 6)
            {
                throw std::runtime_error("Kp must be of size 6.");
            }
            Kp_task_ = Kp;
        }

        void RobotController::setTaskKvGain(const VectorXd& Kv)
        {
            if (Kv.size() != 6)
            {
                throw std::runtime_error("Kv must be of size 6.");
            }
            Kv_task_ = Kv;
        }

        void RobotController::setQPIKGain(const VectorXd& w_tracking, const VectorXd& w_damping)
        {
            if (w_tracking.size() != 6 || w_damping.size() != actuator_dof_)
            {
                throw std::runtime_error("w_tracking must be of size 6 and w_damping must be of size actuator_dof_");
            }
            QP_moma_IK_->setWeight(w_tracking, w_damping);
        }

        void RobotController::setQPIDGain(const VectorXd& w_tracking, const VectorXd& w_vel_damping, const VectorXd& w_acc_damping)
        {
            if (w_tracking.size() != 6 || w_vel_damping.size() != actuator_dof_ || w_acc_damping.size() != actuator_dof_)
            {
                throw std::runtime_error("w_tracking must be of size 6 and w_damping must be of size actuator_dof_");
            }
            QP_moma_ID_->setWeight(w_tracking, w_vel_damping, w_acc_damping);
        }

        VectorXd RobotController::computeMobileWheelVel(const VectorXd& base_vel)
        {
            return Mobile::RobotController::computeWheelVel(base_vel);
        }

        MatrixXd RobotController::computeMobileIKJacobian()
        {
            return Mobile::RobotController::computeIKJacobian();
        }

        VectorXd RobotController::MobileVelocityCommand(const VectorXd& desired_base_vel)
        {
            return Mobile::RobotController::VelocityCommand(desired_base_vel);
        }

        VectorXd RobotController::moveManipulatorJointPositionCubic(const VectorXd& q_mani_target,
                                                                    const VectorXd& qdot_mani_target,
                                                                    const VectorXd& q_mani_init,
                                                                    const VectorXd& qdot_mani_init,
                                                                    const double& current_time,
                                                                    const double& init_time,
                                                                    const double& duration)
        {
            assert(q_mani_target.size() == mani_dof_ && 
                   qdot_mani_target.size() == mani_dof_ &&
                   q_mani_init.size() == mani_dof_ &&
                   qdot_mani_init.size() == mani_dof_);

            VectorXd q_mani_desired =  DyrosMath::cubicVector(current_time,
                                                              init_time,
                                                              init_time + duration,
                                                              q_mani_init,
                                                              q_mani_target,
                                                              qdot_mani_init,
                                                              qdot_mani_target);
            return q_mani_desired;
        }

        VectorXd RobotController::moveManipulatorJointTorqueStep(const VectorXd& qddot_mani_target, const bool use_mass)
        {
            assert(qddot_mani_target.size() == mani_dof_);
            MatrixXd M_mani = robot_data_->getMassMatrix().block(robot_data_->getJointIndex().mani_start,robot_data_->getJointIndex().mani_start,mani_dof_,mani_dof_);
            MatrixXd g_mani = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            if(use_mass) return M_mani * qddot_mani_target + g_mani;
            else         return qddot_mani_target + g_mani;
        }

        VectorXd RobotController::moveManipulatorJointTorqueStep(const VectorXd& q_mani_target,
                                                                 const VectorXd& qdot_mani_target,
                                                                 const bool use_mass)
        {
            VectorXd q_mani = robot_data_->getJointPosition().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            VectorXd qdot_mani = robot_data_->getJointVelocity().segment(robot_data_->getJointIndex().mani_start,mani_dof_);
            VectorXd qddot_mani_desired = Kp_mani_joint_.asDiagonal() * (q_mani_target - q_mani) + Kv_mani_joint_.asDiagonal() * (qdot_mani_target - qdot_mani);
            return moveManipulatorJointTorqueStep(qddot_mani_desired, use_mass);
        }

        VectorXd RobotController::moveManipulatorJointTorqueCubic(const VectorXd& q_mani_target,
                                                                  const VectorXd& qdot_mani_target,
                                                                  const VectorXd& q_mani_init,
                                                                  const VectorXd& qdot_mani_init,
                                                                  const double& current_time,
                                                                  const double& init_time,
                                                                  const double& duration,
                                                                  const bool use_mass)
        {
            VectorXd q_mani_desired =  DyrosMath::cubicVector(current_time,
                                                              init_time,
                                                              init_time + duration,
                                                              q_mani_init,
                                                              q_mani_target,
                                                              qdot_mani_init,
                                                              qdot_mani_target);
    
            VectorXd qdot_mani_desired =  DyrosMath::cubicDotVector(current_time,
                                                                    init_time,
                                                                    init_time + duration,
                                                                    q_mani_init,
                                                                    q_mani_target,
                                                                    qdot_mani_init,
                                                                    qdot_mani_target);
    
            return moveManipulatorJointTorqueStep(q_mani_desired, qdot_mani_desired, use_mass);
        }

        void RobotController::QPIK(const VectorXd& xdot_target,
                                   const std::string& link_name,
                                   VectorXd& opt_qdot_mobile,
                                   VectorXd& opt_qdot_manipulator)
        {  
            QP_moma_IK_->setDesiredTaskVel(xdot_target, link_name);
            VectorXd opt_qdot = VectorXd::Zero(actuator_dof_);
            QP::TimeDuration tmp;
            if(!QP_moma_IK_->getOptJointVel(opt_qdot, tmp))
            {
                std::cerr << "QP IK failed to compute optimal joint velocity." << std::endl;
                opt_qdot.setZero(dof_);
            }
        
            opt_qdot_mobile.setZero(mobi_dof_);
            opt_qdot_manipulator.setZero(mani_dof_);
            opt_qdot_mobile = opt_qdot.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_qdot_manipulator = opt_qdot.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
        }

        void RobotController::QPIKStep(const Affine3d& x_target, 
                                       const VectorXd& xdot_target,
                                       const std::string& link_name,
                                       VectorXd& opt_qdot_mobile,
                                       VectorXd& opt_qdot_manipulator)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
            
            QPIK(Kp_task_.asDiagonal() * x_error + xdot_target, link_name, opt_qdot_mobile, opt_qdot_manipulator);
        }
    
    
        void RobotController::QPIKCubic(const Affine3d& x_target,
                                        const VectorXd& xdot_target,
                                        const Affine3d& x_init,
                                        const VectorXd& xdot_init,
                                        const double& current_time,
                                        const double& init_time,
                                        const double& duration,
                                        const std::string& link_name,
                                        VectorXd& opt_qdot_mobile,
                                        VectorXd& opt_qdot_manipulator)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
    
            QPIKStep(x_desired, xdot_desired, link_name, opt_qdot_mobile, opt_qdot_manipulator);
        }

        void RobotController::QPID(const VectorXd& xddot_target,
                                   const std::string& link_name,
                                   VectorXd& opt_qddot_mobile,
                                   VectorXd& opt_torque_manipulator)
        {
            QP_moma_ID_->setDesiredTaskAcc(xddot_target, link_name);
            VectorXd opt_qddot = VectorXd::Zero(dof_);
            VectorXd opt_torque = VectorXd::Zero(dof_);
            QP::TimeDuration tmp;
            if(!QP_moma_ID_->getOptJoint(opt_qddot, opt_torque, tmp))
            {
                std::cerr << "QP ID failed to compute optimal joint torque." << std::endl;
                opt_torque = robot_data_->getGravity();
                opt_qddot.setZero();
            }

            opt_qddot_mobile.setZero(mobi_dof_);
            opt_torque_manipulator.setZero(mani_dof_);
            opt_qddot_mobile = opt_qddot.segment(robot_data_->getActuatorIndex().mobi_start, mobi_dof_);
            opt_torque_manipulator = opt_torque.segment(robot_data_->getActuatorIndex().mani_start, mani_dof_);
        }

        void RobotController::QPIDStep(const Affine3d& x_target, 
                                       const VectorXd& xdot_target,
                                       const std::string& link_name,
                                       VectorXd& opt_qddot_mobile,
                                       VectorXd& opt_torque_manipulator)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
            
            QPID(Kp_task_.asDiagonal() * x_error + Kv_task_.asDiagonal() * xdot_error, link_name, opt_qddot_mobile, opt_torque_manipulator);
        }
    
    
        void RobotController::QPIDCubic(const Affine3d& x_target,
                                        const VectorXd& xdot_target,
                                        const Affine3d& x_init,
                                        const VectorXd& xdot_init,
                                        const double& current_time,
                                        const double& init_time,
                                        const double& duration,
                                        const std::string& link_name,
                                        VectorXd& opt_qddot_mobile,
                                        VectorXd& opt_torque_manipulator)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
    
            return QPIDStep(x_desired, xdot_desired, link_name, opt_qddot_mobile, opt_torque_manipulator);
        }
    } // namespace MobileManipulator
} // namespace drc