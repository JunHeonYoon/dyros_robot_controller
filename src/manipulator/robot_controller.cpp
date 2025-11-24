#include "dyros_robot_controller/manipulator/robot_controller.h"

namespace drc
{
    namespace Manipulator
    {
        RobotController::RobotController(const double& dt,
                                         std::shared_ptr<Manipulator::RobotData> robot_data)
        : robot_data_(robot_data)
        {
            dof_ = robot_data_->getDof();
            Kp_task_ = VectorXd::Constant(6, 100);
            Kv_task_ = VectorXd::Constant(6, 20);
            Kp_joint_ = VectorXd::Constant(dof_, 400);
            Kv_joint_ = VectorXd::Constant(dof_, 40);
    
            QP_mani_IK_ = std::make_unique<Manipulator::QPIK>(robot_data_);
            QP_mani_ID_ = std::make_unique<Manipulator::QPID>(robot_data_);
        }
    
        void RobotController::setJointGain(const VectorXd& Kp, const VectorXd& Kv)
        {
            if (Kp.size() != dof_ || Kv.size() != dof_)
            {
                throw std::runtime_error("Kp and Kv must be of size dof_.");
            }
            Kp_joint_ = Kp;
            Kv_joint_ = Kv;
        }

        void RobotController::setJointKpGain(const VectorXd& Kp)
        {
            if (Kp.size() != dof_ )
            {
                throw std::runtime_error("Kp must be of size dof_.");
            }
            Kp_joint_ = Kp;
        }

        void RobotController::setJointKvGain(const VectorXd& Kv)
        {
            if (Kv.size() != dof_ )
            {
                throw std::runtime_error("Kv must be of size dof_.");
            }
            Kv_joint_ = Kv;
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
            if (w_tracking.size() != 6 || w_damping.size() != dof_)
            {
                throw std::runtime_error("w_tracking must be of size 6 and w_damping must be of size dof_");
            }
            QP_mani_IK_->setWeight(w_tracking, w_damping);
        }

        void RobotController::setQPIDGain(const VectorXd& w_tracking, const VectorXd& w_damping)
        {
            if (w_tracking.size() != 6 || w_damping.size() != dof_)
            {
                throw std::runtime_error("w_tracking must be of size 6 and w_damping must be of size dof_");
            }
            QP_mani_ID_->setWeight(w_tracking, w_damping);
        }
    
        VectorXd RobotController::moveJointPositionCubic(const VectorXd& q_target,
                                                         const VectorXd& qdot_target,
                                                         const VectorXd& q_init,
                                                         const VectorXd& qdot_init,
                                                         const double& current_time,
                                                         const double& init_time,
                                                         const double& duration)
        {
            VectorXd q_desired =  DyrosMath::cubicVector(current_time,
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
            VectorXd qdot_desired =  DyrosMath::cubicDotVector(current_time,
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
            VectorXd qddot_desired = Kp_joint_.asDiagonal() * (q_target - robot_data_->getJointPosition()) + Kv_joint_.asDiagonal() * (qdot_target - robot_data_->getJointVelocity());
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
            VectorXd q_desired =  DyrosMath::cubicVector(current_time,
                                                         init_time,
                                                         init_time + duration,
                                                         q_init,
                                                         q_target,
                                                         qdot_init,
                                                         qdot_target);
    
            VectorXd qdot_desired =  DyrosMath::cubicDotVector(current_time,
                                                               init_time,
                                                               init_time + duration,
                                                               q_init,
                                                               q_target,
                                                               qdot_init,
                                                               qdot_target);
    
            return moveJointTorqueStep(q_desired, qdot_desired, use_mass);
        }
    
        
    
        VectorXd RobotController::CLIKStep(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const VectorXd& null_qdot,
                                           const std::string& link_name)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
    
            MatrixXd J = robot_data_->getJacobian(link_name);
            MatrixXd J_pinv = DyrosMath::PinvCOD(J);
    
            MatrixXd null_proj = MatrixXd::Identity(dof_, dof_) - J_pinv * J;
    
            VectorXd qdot_desired = J_pinv * (Kp_task_.asDiagonal() * x_error + xdot_target) + null_proj * null_qdot;
            return qdot_desired;
        }

        VectorXd RobotController::CLIKStep(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const std::string& link_name)
        {
            return CLIKStep(x_target, xdot_target, VectorXd::Zero(dof_), link_name);
        }
    
        VectorXd RobotController::CLIKCubic(const Affine3d& x_target,
                                            const VectorXd& xdot_target,
                                            const Affine3d& x_init,
                                            const VectorXd& xdot_init,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration,
                                            const VectorXd& null_qdot,
                                            const std::string& link_name)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
            return CLIKStep(x_desired, xdot_desired, null_qdot, link_name);
        }

        VectorXd RobotController::CLIKCubic(const Affine3d& x_target,
                                            const VectorXd& xdot_target,
                                            const Affine3d& x_init,
                                            const VectorXd& xdot_init,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration,
                                            const std::string& link_name)
        {
            return CLIKCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, VectorXd::Zero(dof_), link_name);
        }
    
        VectorXd RobotController::OSF(const VectorXd& xddot_target, 
                                      const VectorXd& null_torque,
                                      const std::string& link_name)
        {
            MatrixXd J = robot_data_->getJacobian(link_name);
            MatrixXd J_T = J.transpose();
            MatrixXd M_inv = robot_data_->getMassMatrixInv();
    
            MatrixXd M_task = DyrosMath::PinvCOD(J * M_inv * J_T); 
            MatrixXd J_T_pinv = M_task * J * M_inv;
            MatrixXd null_proj = MatrixXd::Identity(dof_, dof_) - (J_T * J_T_pinv);
            
            VectorXd force_desired = M_task * xddot_target;
            
            return J_T * force_desired + null_proj * null_torque + robot_data_->getGravity();
        }
    
        VectorXd RobotController::OSF(const VectorXd& xddot_target,
                                      const std::string& link_name)
        {
            return OSF(xddot_target, VectorXd::Zero(dof_), link_name);
        }
    
        VectorXd RobotController::OSFStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const VectorXd& null_torque,
                                          const std::string& link_name)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
            VectorXd xddot_target = Kp_task_.asDiagonal() * x_error + Kv_task_.asDiagonal() * xdot_error;
            return OSF(xddot_target, null_torque, link_name);
        }
    
        VectorXd RobotController::OSFStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name)
        {
            return OSFStep(x_target, xdot_target, VectorXd::Zero(dof_), link_name);
        }
    
        VectorXd RobotController::OSFCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const VectorXd& null_torque,
                                           const std::string& link_name)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
            return OSFStep(x_desired, xdot_desired, null_torque, link_name);
        }
    
        VectorXd RobotController::OSFCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name)
        {
            return OSFCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, VectorXd::Zero(dof_), link_name);
        }
    
        VectorXd RobotController::QPIK(const VectorXd& xdot_target,
                                       const std::string& link_name)
        {  
            QP_mani_IK_->setDesiredTaskVel(xdot_target, link_name);
            VectorXd opt_qdot = VectorXd::Zero(dof_);
            QP::TimeDuration tmp;
            if(!QP_mani_IK_->getOptJointVel(opt_qdot, tmp))
            {
                std::cerr << "QP IK failed to compute optimal joint velocity." << std::endl;
                opt_qdot.setZero(dof_);
            }
        
            return opt_qdot;
        }

        VectorXd RobotController::QPIKStep(const Affine3d& x_target, 
                                           const VectorXd& xdot_target,
                                           const std::string& link_name)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
            
            return QPIK(Kp_task_.asDiagonal() * x_error + Kv_task_.asDiagonal() * xdot_error, link_name);
        }
    
    
        VectorXd RobotController::QPIKCubic(const Affine3d& x_target,
                                            const VectorXd& xdot_target,
                                            const Affine3d& x_init,
                                            const VectorXd& xdot_init,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration,
                                            const std::string& link_name)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
    
            return QPIKStep(x_desired, xdot_desired, link_name);
        }

        VectorXd RobotController::QPID(const VectorXd& xddot_target,
                                       const std::string& link_name)
        {
            QP_mani_ID_->setDesiredTaskAcc(xddot_target, link_name);
            VectorXd opt_qddot = VectorXd::Zero(dof_);
            VectorXd opt_torque = VectorXd::Zero(dof_);
            QP::TimeDuration tmp;
            if(!QP_mani_ID_->getOptJoint(opt_qddot, opt_torque, tmp))
            {
                std::cerr << "QP ID failed to compute optimal joint torque." << std::endl;
                opt_torque = robot_data_->getGravity();
            }

            return opt_torque;
        }

        VectorXd RobotController::QPIDStep(const Affine3d& x_target, 
                                           const VectorXd& xdot_target,
                                           const std::string& link_name)
        {
            VectorXd x_error, xdot_error;
            DyrosMath::getTaskSpaceError(x_target, xdot_target, robot_data_->getPose(link_name), robot_data_->getVelocity(link_name), x_error, xdot_error);
            
            return QPID(Kp_task_.asDiagonal() * x_error + Kv_task_.asDiagonal() * xdot_error, link_name);
        }
    
    
        VectorXd RobotController::QPIDCubic(const Affine3d& x_target,
                                            const VectorXd& xdot_target,
                                            const Affine3d& x_init,
                                            const VectorXd& xdot_init,
                                            const double& current_time,
                                            const double& init_time,
                                            const double& duration,
                                            const std::string& link_name)
        {
            Affine3d x_desired;
            VectorXd xdot_desired;
            DyrosMath::getTaskSpaceCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, x_desired, xdot_desired);
    
            return QPIDStep(x_desired, xdot_desired, link_name);
        }
    } // namespace Manipulator
} // namespace drc