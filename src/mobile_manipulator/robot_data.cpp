#include "dyros_robot_controller/mobile_manipulator/robot_data.h"

namespace drc
{
    namespace MobileManipulator
    {
        RobotData::RobotData(const double dt,
                             const Mobile::KinematicParam& mobile_param,
                             const JointIndex& joint_idx,
                             const ActuatorIndex& actuator_idx,
                             const std::string& urdf_path,
                             const std::string& srdf_path, 
                             const std::string& packages_path)
        : Mobile::RobotData(dt, mobile_param), 
          Manipulator::RobotData(dt, urdf_path, srdf_path, packages_path),
          joint_idx_(joint_idx),
          actuator_idx_(actuator_idx)
        {
            mobi_dof_ = wheel_num_;
            mani_dof_ = dof_ - (virtual_dof_ + mobi_dof_); // TODO: if manipulator has gripper or other joint, then this code does not work well
            actuated_dof_ = mobi_dof_ + mani_dof_;

            // Initialize selection matrix
            S_.setZero(dof_,actuated_dof_);
            S_.block(joint_idx_.mani_start,actuator_idx_.mani_start,mani_dof_,mani_dof_).setIdentity();
            S_.block(joint_idx_.mobi_start,actuator_idx_.mobi_start,mobi_dof_,mobi_dof_).setIdentity();
            // Sdot_.setZero(dof_,actuated_dof_);
            
            // Initialize actuated joint space state
            q_virtual_.setZero(virtual_dof_);
            q_mobile_.setZero(mobi_dof_);
            q_mani_.setZero(mani_dof_);
            qdot_virtual_.setZero(virtual_dof_);
            qdot_mobile_.setZero(mobi_dof_);
            qdot_mani_.setZero(mani_dof_);
            q_actuated_.setZero(actuated_dof_);
            qdot_actuated_.setZero(actuated_dof_);
        
            // Initialize actuated joint space dynamics
            M_actuated_.setZero(actuated_dof_,actuated_dof_);
            M_inv_actuated_.setZero(actuated_dof_,actuated_dof_);
            g_actuated_.setZero(actuated_dof_);       
            c_actuated_.setZero(actuated_dof_);       
            NLE_actuated_.setZero(actuated_dof_);   
        }

        std::string RobotData::getVerbose() const
        {
            std::string tmp_info1 = Manipulator::RobotData::getVerbose();
            std::string tmp_info2 = Mobile::RobotData::getVerbose();
            std::ostringstream oss;
            oss << tmp_info1;
            oss << "\n"
                << "==================== Partition Indices ====================\n"
                << " joint index\n"
                << " name                | start\n"
                << "---------------------+------\n"
                << std::left << std::setw(20) << " virtual"      << " | " << std::right << joint_idx_.virtual_start << '\n'
                << std::left << std::setw(20) << " manipulator"  << " | " << std::right << joint_idx_.mani_start    << '\n'
                << std::left << std::setw(20) << " mobile"       << " | " << std::right << joint_idx_.mobi_start    << '\n'
                << std::left
                << "\n actuator index\n"
                << " name                | start\n"
                << "---------------------+------\n"
                << std::left << std::setw(20) << " manipulator"  << " | " << std::right << actuator_idx_.mani_start << '\n'
                << std::left << std::setw(20) << " mobile"       << " | " << std::right << actuator_idx_.mobi_start << '\n';

            oss << "\n"
                << "======================= DoF Summary =======================\n"
                << std::left << std::setw(20) << " total dof"        << " | " << std::right << dof_           << '\n'
                << std::left << std::setw(20) << " virtual dof"      << " | " << std::right << virtual_dof_   << '\n'
                << std::left << std::setw(20) << " mobile dof"       << " | " << std::right << mobi_dof_      << '\n'
                << std::left << std::setw(20) << " manipulator dof"  << " | " << std::right << mani_dof_      << '\n'
                << std::left << std::setw(20) << " actuated dof"     << " | " << std::right << actuated_dof_  << '\n';

            oss << "\n"
                << "======================= Mobile Summary =======================\n"
                << tmp_info2;

            return oss.str();

        }

        bool RobotData::updateState(const VectorXd& q_virtual,
                                    const VectorXd& q_mobile,
                                    const VectorXd& q_mani,
                                    const VectorXd& qdot_virtual,
                                    const VectorXd& qdot_mobile,
                                    const VectorXd& qdot_mani)
        {
            q_virtual_ = q_virtual;
            q_mobile_ = q_mobile;
            q_mani_ = q_mani;
            qdot_virtual_ = qdot_virtual;
            qdot_mobile_ = qdot_mobile;
            qdot_mani_ = qdot_mani;
            q_ = getJointVector(q_virtual, q_mobile, q_mani);
            qdot_ = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            q_actuated_ = getActuatorVector(q_mobile, q_mani);
            qdot_actuated_ = getActuatorVector(qdot_mobile, qdot_mani);
            
            
            if(!updateKinematics(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_)) return false;
            if(!updateDynamics(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_)) return false;
            return true;
        }

        bool RobotData::updateKinematics(const VectorXd& q_virtual,
                                         const VectorXd& q_mobile,
                                         const VectorXd& q_mani,
                                         const VectorXd& qdot_virtual,
                                         const VectorXd& qdot_mobile,
                                         const VectorXd& qdot_mani)
        {
            if(!Mobile::RobotData::updateState(q_mobile, qdot_mobile)) return false;
            double mobile_yaw = q_virtual(2);
            Matrix3d R_world2Base;
            R_world2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                            sin(mobile_yaw),  cos(mobile_yaw), 0,
                            0,                0,               1;
            S_.block(joint_idx_.virtual_start,actuator_idx_.mobi_start,virtual_dof_,mobi_dof_) = R_world2Base * Mobile::RobotData::getFKJacobian();
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::updateKinematics(q, qdot);
        }

        bool RobotData::updateDynamics(const VectorXd& q_virtual,
                                       const VectorXd& q_mobile,
                                       const VectorXd& q_mani,
                                       const VectorXd& qdot_virtual,
                                       const VectorXd& qdot_mobile,
                                       const VectorXd& qdot_mani)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);

            if(!Manipulator::RobotData::updateDynamics(q, qdot)) return false;
            
            M_actuated_ = S_.transpose() * M_ * S_;
            M_inv_actuated_ = DyrosMath::PinvCOD(M_actuated_);
            g_actuated_ = S_.transpose() * g_;
            NLE_actuated_ = S_.transpose() * NLE_;
            c_actuated_ = S_.transpose() * (NLE_ - g_);
            return true;
        }

        MatrixXd RobotData::computeMassMatrix(const VectorXd& q_virtual,
                                              const VectorXd& q_mobile,
                                              const VectorXd& q_mani)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            return Manipulator::RobotData::computeMassMatrix(q);
        }

        VectorXd RobotData::computeGravity(const VectorXd& q_virtual,
                                           const VectorXd& q_mobile,
                                           const VectorXd& q_mani)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            return Manipulator::RobotData::computeGravity(q);
        }

        VectorXd RobotData::computeCoriolis(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani,
                                            const VectorXd& qdot_virtual,
                                            const VectorXd& qdot_mobile,
                                            const VectorXd& qdot_mani)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::computeCoriolis(q,qdot);
        }

        VectorXd RobotData::computeNonlinearEffects(const VectorXd& q_virtual,
                                                    const VectorXd& q_mobile,
                                                    const VectorXd& q_mani,
                                                    const VectorXd& qdot_virtual,
                                                    const VectorXd& qdot_mobile,
                                                    const VectorXd& qdot_mani)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::computeNonlinearEffects(q,qdot);
        }


        MatrixXd RobotData::computeMassMatrixActuated(const VectorXd& q_virtual,
                                                      const VectorXd& q_mobile,
                                                      const VectorXd& q_mani)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return S.transpose() * Manipulator::RobotData::computeMassMatrix(q) * S;
        }

        VectorXd RobotData::computeGravityActuated(const VectorXd& q_virtual,
                                                   const VectorXd& q_mobile,
                                                   const VectorXd& q_mani)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return S.transpose() * Manipulator::RobotData::computeGravity(q);
        }

        VectorXd RobotData::computeCoriolisActuated(const VectorXd& q_virtual,
                                                    const VectorXd& q_mobile,
                                                    const VectorXd& q_mani,
                                                    const VectorXd& qdot_mobile,
                                                    const VectorXd& qdot_mani)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            VectorXd qdot = getJointVector(VectorXd::Zero(virtual_dof_), qdot_mobile, qdot_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return S.transpose() * (Manipulator::RobotData::computeNonlinearEffects(q, qdot) - Manipulator::RobotData::computeGravity(q));
        }

        VectorXd RobotData::computeNonlinearEffectsActuated(const VectorXd& q_virtual,
                                                            const VectorXd& q_mobile,
                                                            const VectorXd& q_mani,
                                                            const VectorXd& qdot_mobile,
                                                            const VectorXd& qdot_mani)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            VectorXd qdot = getJointVector(VectorXd::Zero(virtual_dof_), qdot_mobile, qdot_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return S.transpose() * Manipulator::RobotData::computeNonlinearEffects(q, qdot);
        }
        
        Affine3d RobotData::computePose(const VectorXd& q_virtual,
                                        const VectorXd& q_mobile,
                                        const VectorXd& q_mani, 
                                        const std::string& link_name)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            return Manipulator::RobotData::computePose(q, link_name);
        }

        MatrixXd RobotData::computeJacobian(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani, 
                                            const std::string& link_name)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            return Manipulator::RobotData::computeJacobian(q, link_name);
        }

        MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani,
                                                         const VectorXd& qdot_virtual,
                                                         const VectorXd& qdot_mobile,
                                                         const VectorXd& qdot_mani, 
                                                         const std::string& link_name)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::computeJacobianTimeVariation(q, qdot, link_name);
        }

        VectorXd RobotData::computeVelocity(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani,
                                            const VectorXd& qdot_virtual,
                                            const VectorXd& qdot_mobile,
                                            const VectorXd& qdot_mani, 
                                            const std::string& link_name)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::computeVelocity(q, qdot, link_name);
        }

        Manipulator::MinDistResult RobotData::computeMinDistance(const VectorXd& q_virtual,
                                                                const VectorXd& q_mobile,
                                                                const VectorXd& q_mani,
                                                                const VectorXd& qdot_virtual,
                                                                const VectorXd& qdot_mobile,
                                                                const VectorXd& qdot_mani, 
                                                                const bool& with_grad, 
                                                                const bool& with_graddot, 
                                                                const bool verbose)
        {
            VectorXd q = getJointVector(q_virtual, q_mobile, q_mani);
            VectorXd qdot = getJointVector(qdot_virtual, qdot_mobile, qdot_mani);
            return Manipulator::RobotData::computeMinDistance(q, qdot, with_grad, with_graddot, verbose);
        }

        Manipulator::ManipulabilityResult RobotData::computeManipulability(const VectorXd& q_mani, 
                                                                           const VectorXd& qdot_mani, 
                                                                           const bool& with_grad, 
                                                                           const bool& with_graddot, 
                                                                           const std::string& link_name)
        {
            Manipulator::ManipulabilityResult result; 
            result.setZero(mani_dof_);
        
            VectorXd q = VectorXd::Zero(dof_);
            q.segment(joint_idx_.mani_start,mani_dof_) = q_mani;
        
            MatrixXd J = computeJacobian(q, link_name);
            J = J.block(0,joint_idx_.mani_start,6,mani_dof_); // singularity does not depends on base frame, so J can be computed on world frame
            result.manipulability = sqrt((J*J.transpose()).determinant());
        
            if(with_grad || with_graddot)
            {
                pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
                if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
                {
                    std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                    return result;
                }
                
                MatrixXd JJt = J*J.transpose();
                MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
                std::vector<MatrixXd> dJ_dq;
                dJ_dq.resize(mani_dof_);
                pinocchio::Data data = pinocchio::Data(model_);
                
                for(size_t i=0; i<mani_dof_; ++i)
                {
                    VectorXd qdot_i = VectorXd::Zero(dof_);
                    qdot_i[joint_idx_.mani_start + i] = 1.0;
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot_i);
                    dJ_dq[i].setZero(6,dof_);
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
                    dJ_dq[i] = dJ_dq[i].block(0,joint_idx_.mani_start,6,mani_dof_);
        
                    result.grad(i) = result.manipulability * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                }
        
                if(with_graddot)
                {
                    VectorXd qdot = VectorXd::Zero(dof_);
                    qdot.segment(joint_idx_.mani_start,mani_dof_) = qdot_mani;
        
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
                    MatrixXd Jdot;
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
                    Jdot = Jdot.block(0,joint_idx_.mani_start,6,mani_dof_);
                    double mani_dot = result.manipulability * (Jdot * J.transpose() * JJt_inv).trace();
        
                    MatrixXd JJt_dot = 2 * Jdot* J.transpose();
                    MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);
        
                    for(size_t i=0; i<mani_dof_; ++i)
                    {
                        result.grad_dot(i) = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                        result.grad_dot(i) += result.manipulability * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                            dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
                    }
                }
            }
        
            return result;
        }

        MatrixXd RobotData::computeMobileFKJacobian(const VectorXd& q_mobile)
        {
            return Mobile::RobotData::computeFKJacobian(q_mobile);
        }

        VectorXd RobotData::computeMobileBaseVel(const VectorXd& q_mobile, const VectorXd& qdot_mobile)
        {
            return Mobile::RobotData::computeBaseVel(q_mobile, qdot_mobile);
        }

        MatrixXd RobotData::computeSelectionMatrix(const VectorXd& q_virtual, const VectorXd& q_mobile)
        {
            MatrixXd S;
            S.setZero(dof_,actuated_dof_);
            S.block(joint_idx_.mani_start,actuator_idx_.mani_start,mani_dof_,mani_dof_).setIdentity();
            S.block(joint_idx_.mobi_start,actuator_idx_.mobi_start,mobi_dof_,mobi_dof_).setIdentity();
            double mobile_yaw = q_virtual(2);
            Matrix3d R_world2Base;
            R_world2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                            sin(mobile_yaw),  cos(mobile_yaw), 0,
                            0,                0,               1;
        
            S.block(joint_idx_.virtual_start,actuator_idx_.mobi_start,virtual_dof_,mobi_dof_) = R_world2Base * computeFKJacobian(q_mobile);
            return S;
        }

        MatrixXd RobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                    const VectorXd& q_mobile,
                                                    const VectorXd& q_mani, 
                                                    const std::string& link_name)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return Manipulator::RobotData::computeJacobian(q, link_name) * S;
        }

        MatrixXd RobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                 const VectorXd& q_mobile,
                                                                 const VectorXd& q_mani,
                                                                 const VectorXd& qdot_virtual,
                                                                 const VectorXd& qdot_mobile,
                                                                 const VectorXd& qdot_mani, 
                                                                 const std::string& link_name)
        {
            VectorXd q = getJointVector(VectorXd::Zero(virtual_dof_), q_mobile, q_mani);
            VectorXd qdot = getJointVector(VectorXd::Zero(virtual_dof_), qdot_mobile, qdot_mani);
            MatrixXd S = computeSelectionMatrix(q_virtual, q_mobile);
            return Manipulator::RobotData::computeJacobianTimeVariation(q, qdot, link_name) * S; // neglect Sdot
        }

        MatrixXd RobotData::getJacobianActuated(const std::string& link_name)
        {
            return Manipulator::RobotData::getJacobian(link_name)*S_;
        }
        
        MatrixXd RobotData::getJacobianActuatedTimeVariation(const std::string& link_name)
        {
            return getJacobianTimeVariation(link_name) * S_; // neglect Sdot
        }
        

        VectorXd RobotData::getJointVector(const VectorXd& q_virtual,
                                           const VectorXd& q_mobile,
                                           const VectorXd& q_mani)
        {
            VectorXd q;
            q.setZero(dof_);
            q.segment(joint_idx_.virtual_start,virtual_dof_) = q_virtual;
            q.segment(joint_idx_.mobi_start,mobi_dof_) = q_mobile;
            q.segment(joint_idx_.mani_start,mani_dof_) = q_mani;
            return q;
        }

        VectorXd RobotData::getActuatorVector(const VectorXd& q_mobile, const VectorXd& q_mani)
        {
            VectorXd q_actuated;
            q_actuated.setZero(actuated_dof_);
            q_actuated.segment(actuator_idx_.mobi_start,mobi_dof_) = q_mobile;
            q_actuated.segment(actuator_idx_.mani_start,mani_dof_) = q_mani;
            return q_actuated;
        }

        Manipulator::ManipulabilityResult RobotData::getManipulability(const bool& with_grad, 
                                                                       const bool& with_graddot, 
                                                                       const std::string& link_name)
        {
            Manipulator::ManipulabilityResult result; 
            result.setZero(mani_dof_);
                
            MatrixXd J = getJacobian(link_name);
            J = J.block(0,joint_idx_.mani_start,6,mani_dof_); // singularity does not depends on base frame, so J can be computed on world frame
            result.manipulability = sqrt((J*J.transpose()).determinant());
        
            if(with_grad || with_graddot)
            {
                pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
                if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
                {
                    std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                    return result;
                }
                
                MatrixXd JJt = J*J.transpose();
                MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
                std::vector<MatrixXd> dJ_dq;
                dJ_dq.resize(mani_dof_);
                pinocchio::Data data = pinocchio::Data(model_);
                
                for(size_t i=0; i<mani_dof_; ++i)
                {
                    VectorXd qdot_i = VectorXd::Zero(dof_);
                    qdot_i[joint_idx_.mani_start + i] = 1.0;
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q_, qdot_i);
                    dJ_dq[i].setZero(6,dof_);
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
                    dJ_dq[i] = dJ_dq[i].block(0,joint_idx_.mani_start,6,mani_dof_);
        
                    result.grad(i) = result.manipulability * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                }
        
                if(with_graddot)
                {      
                    MatrixXd Jdot = getJacobianTimeVariation(link_name);
                    Jdot = Jdot.block(0,joint_idx_.mani_start,6,mani_dof_);
                    double mani_dot = result.manipulability * (Jdot * J.transpose() * JJt_inv).trace();
        
                    MatrixXd JJt_dot = 2 * Jdot* J.transpose();
                    MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);
        
                    for(size_t i=0; i<mani_dof_; ++i)
                    {
                        result.grad_dot(i) = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                        result.grad_dot(i) += result.manipulability * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                            dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
                    }
                }
            }
        
            return result;
        }
    } // namespace MobileManipulator
} // namespace drc
