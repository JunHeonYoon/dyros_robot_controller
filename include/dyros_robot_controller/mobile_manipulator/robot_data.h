// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
//
// Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
// Seoul National University, under a research agreement with ETRI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>
#include <filesystem>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include "math_type_define.h"

#include "dyros_robot_controller/type_define.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/mobile/robot_data.h"

using namespace Eigen;

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Abstract base class for mobile manipulator robot data.
         * 
         * This class combines the functionality of a mobile base and a manipulator base.
         * It provides methods for updating the state, computing kinematics and dynamics,
         * and accessing kinematic parameters for both the mobile base and the manipulator.
         * It supports a mobile manipulator with a floating base, manipulator joints, and mobile wheel joints.
        */
        class RobotData : public Manipulator::RobotData, public Mobile::RobotData
        {
            private:
                // =====================================================================
                // ManipulatorProxy – declared first for correct C++ init-order
                // =====================================================================
                /**
                 * @brief Thin proxy that exposes arm-only robot data in the mobile-base frame.
                 *
                 * It delegates storage to its parent MobileManipulator::RobotData and
                 * overrides every virtual getter to return manipulator-DOF quantities
                 * expressed relative to the mobile base rather than the world frame.
                 * State update is a no-op: the parent class keeps all data current.
                 */
                class ManipulatorProxy : public Manipulator::RobotData
                {
                    drc::MobileManipulator::RobotData& src_;
                public:
                    explicit ManipulatorProxy(drc::MobileManipulator::RobotData& src) : src_(src)
                    {
                        link_frame_names_ = src_.Manipulator::RobotData::getLinkFrameVector();
                        joint_frame_names_ = src_.Manipulator::RobotData::getJointFrameVector();
                        joint_names_ = src_.Manipulator::RobotData::getJointNames();
                        root_link_name_ = src_.Manipulator::RobotData::getRootLinkName();

                        link_frame_set_.clear();
                        joint_frame_set_.clear();
                        for (const auto& name : link_frame_names_) {
                            link_frame_set_.insert(name);
                        }
                        for (const auto& name : joint_frame_names_) {
                            joint_frame_set_.insert(name);
                        }
                    }

                    int      getDof()           const override { return src_.mani_dof_; }
                    VectorXd getJointPosition() const override { return src_.q_mani_; }
                    VectorXd getJointVelocity() const override { return src_.qdot_mani_; }
                    std::pair<VectorXd,VectorXd> getJointPositionLimit() const override
                    {
                        const auto whole_limit = src_.Manipulator::RobotData::getJointPositionLimit();
                        return std::make_pair(
                            whole_limit.first.segment(src_.joint_idx_.mani_start, src_.mani_dof_),
                            whole_limit.second.segment(src_.joint_idx_.mani_start, src_.mani_dof_));
                    }
                    std::pair<VectorXd,VectorXd> getJointVelocityLimit() const override
                    {
                        const auto whole_limit = src_.Manipulator::RobotData::getJointVelocityLimit();
                        return std::make_pair(
                            whole_limit.first.segment(src_.joint_idx_.mani_start, src_.mani_dof_),
                            whole_limit.second.segment(src_.joint_idx_.mani_start, src_.mani_dof_));
                    }
                    std::pair<VectorXd,VectorXd> getJointEffortLimit() const override
                    {
                        const auto whole_limit = src_.Manipulator::RobotData::getJointEffortLimit();
                        return std::make_pair(
                            whole_limit.first.segment(src_.joint_idx_.mani_start, src_.mani_dof_),
                            whole_limit.second.segment(src_.joint_idx_.mani_start, src_.mani_dof_));
                    }

                    MatrixXd getMassMatrix() const override
                    {
                        return src_.Manipulator::RobotData::getMassMatrix()
                                   .block(src_.joint_idx_.mani_start,
                                          src_.joint_idx_.mani_start,
                                          src_.mani_dof_, src_.mani_dof_);
                    }
                    MatrixXd getMassMatrixInv() const override { return getMassMatrix().inverse(); }

                    VectorXd getGravity() const override
                    {
                        return src_.Manipulator::RobotData::getGravity()
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }
                    VectorXd getCoriolis() const override
                    {
                        return src_.Manipulator::RobotData::getCoriolis()
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }
                    VectorXd getNonlinearEffects() const override
                    {
                        return src_.Manipulator::RobotData::getNonlinearEffects()
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }

                    /// Returns the link pose expressed in the mobile-base frame.
                    Affine3d getPose(const std::string& link_name) const override
                    {
                        const Affine3d T_base_w = src_.Manipulator::RobotData::getPose(
                            src_.getBaseLinkName());
                        return T_base_w.inverse() * src_.Manipulator::RobotData::getPose(link_name);
                    }

                    /// Returns the Jacobian with columns sliced to arm joints and rows
                    /// rotated into the mobile-base frame.
                    MatrixXd getJacobian(const std::string& link_name) override
                    {
                        const MatrixXd J_full = src_.Manipulator::RobotData::getJacobian(link_name);
                        const MatrixXd J_arm  = J_full.block(0, src_.joint_idx_.mani_start,
                                                              6, src_.mani_dof_);
                        const Matrix3d R = src_.Manipulator::RobotData::getPose(
                            src_.getBaseLinkName()).linear().transpose();
                        MatrixXd J(6, src_.mani_dof_);
                        J.topRows(3)    = R * J_arm.topRows(3);
                        J.bottomRows(3) = R * J_arm.bottomRows(3);
                        return J;
                    }

                    MatrixXd getJacobianTimeVariation(const std::string& link_name) override
                    {
                        const MatrixXd Jdot_full = src_.Manipulator::RobotData::getJacobianTimeVariation(link_name);
                        const MatrixXd Jdot_arm  = Jdot_full.block(0, src_.joint_idx_.mani_start,
                                                                    6, src_.mani_dof_);
                        const Matrix3d R = src_.Manipulator::RobotData::getPose(
                            src_.getBaseLinkName()).linear().transpose();
                        MatrixXd Jdot(6, src_.mani_dof_);
                        Jdot.topRows(3)    = R * Jdot_arm.topRows(3);
                        Jdot.bottomRows(3) = R * Jdot_arm.bottomRows(3);
                        return Jdot;
                    }

                    VectorXd getVelocity(const std::string& link_name) override
                    {
                        return getJacobian(link_name) * src_.qdot_mani_;
                    }

                    // ---- Compute overrides (explicit q/qdot, base frame output) ----

                    MatrixXd computeMassMatrix(const Eigen::Ref<const VectorXd>& q_mani) override
                    {
                        const VectorXd q_full = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        return src_.Manipulator::RobotData::computeMassMatrix(q_full)
                                   .block(src_.joint_idx_.mani_start, src_.joint_idx_.mani_start,
                                          src_.mani_dof_, src_.mani_dof_);
                    }

                    VectorXd computeGravity(const Eigen::Ref<const VectorXd>& q_mani) override
                    {
                        const VectorXd q_full = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        return src_.Manipulator::RobotData::computeGravity(q_full)
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }

                    VectorXd computeCoriolis(const Eigen::Ref<const VectorXd>& q_mani,
                                            const Eigen::Ref<const VectorXd>& qdot_mani) override
                    {
                        const VectorXd q_full    = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        const VectorXd qdot_full = src_.getJointVector(src_.qdot_virtual_, src_.qdot_mobile_, qdot_mani);
                        return src_.Manipulator::RobotData::computeCoriolis(q_full, qdot_full)
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }

                    VectorXd computeNonlinearEffects(const Eigen::Ref<const VectorXd>& q_mani,
                                                     const Eigen::Ref<const VectorXd>& qdot_mani) override
                    {
                        const VectorXd q_full    = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        const VectorXd qdot_full = src_.getJointVector(src_.qdot_virtual_, src_.qdot_mobile_, qdot_mani);
                        return src_.Manipulator::RobotData::computeNonlinearEffects(q_full, qdot_full)
                                   .segment(src_.joint_idx_.mani_start, src_.mani_dof_);
                    }

                    /// Returns the link pose expressed in the mobile-base frame for the given q_mani.
                    Affine3d computePose(const Eigen::Ref<const VectorXd>& q_mani,
                                        const std::string& link_name) override
                    {
                        const VectorXd q_full    = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        const Affine3d T_base_w  = src_.Manipulator::RobotData::computePose(q_full, src_.getBaseLinkName());
                        return T_base_w.inverse() * src_.Manipulator::RobotData::computePose(q_full, link_name);
                    }

                    /// Returns the Jacobian with arm-only columns in the mobile-base frame for the given q_mani.
                    MatrixXd computeJacobian(const Eigen::Ref<const VectorXd>& q_mani,
                                             const std::string& link_name) override
                    {
                        const VectorXd q_full   = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        const MatrixXd J_full   = src_.Manipulator::RobotData::computeJacobian(q_full, link_name);
                        const MatrixXd J_arm    = J_full.block(0, src_.joint_idx_.mani_start, 6, src_.mani_dof_);
                        const Matrix3d R = src_.Manipulator::RobotData::computePose(
                            q_full, src_.getBaseLinkName()).linear().transpose();
                        MatrixXd J(6, src_.mani_dof_);
                        J.topRows(3)    = R * J_arm.topRows(3);
                        J.bottomRows(3) = R * J_arm.bottomRows(3);
                        return J;
                    }

                    MatrixXd computeJacobianTimeVariation(const Eigen::Ref<const VectorXd>& q_mani,
                                                          const Eigen::Ref<const VectorXd>& qdot_mani,
                                                          const std::string& link_name) override
                    {
                        const VectorXd q_full    = src_.getJointVector(src_.q_virtual_, src_.q_mobile_, q_mani);
                        const VectorXd qdot_full = src_.getJointVector(src_.qdot_virtual_, src_.qdot_mobile_, qdot_mani);
                        const MatrixXd Jdot_full = src_.Manipulator::RobotData::computeJacobianTimeVariation(
                            q_full, qdot_full, link_name);
                        const MatrixXd Jdot_arm  = Jdot_full.block(0, src_.joint_idx_.mani_start, 6, src_.mani_dof_);
                        const Matrix3d R = src_.Manipulator::RobotData::computePose(
                            q_full, src_.getBaseLinkName()).linear().transpose();
                        MatrixXd Jdot(6, src_.mani_dof_);
                        Jdot.topRows(3)    = R * Jdot_arm.topRows(3);
                        Jdot.bottomRows(3) = R * Jdot_arm.bottomRows(3);
                        return Jdot;
                    }

                    VectorXd computeVelocity(const Eigen::Ref<const VectorXd>& q_mani,
                                             const Eigen::Ref<const VectorXd>& qdot_mani,
                                             const std::string& link_name) override
                    {
                        return computeJacobian(q_mani, link_name) * qdot_mani;
                    }

                    Manipulator::MinDistResult computeMinDistance(const Eigen::Ref<const VectorXd>& q_mani,
                                                               const Eigen::Ref<const VectorXd>& qdot_mani,
                                                               const bool& with_grad,
                                                               const bool& with_graddot,
                                                               const bool verbose) override
                    {
                        return src_.computeMinDistance(src_.q_virtual_, src_.q_mobile_, q_mani,
                                                       src_.qdot_virtual_, src_.qdot_mobile_, qdot_mani,
                                                       with_grad, with_graddot, verbose);
                    }

                    Manipulator::ManipulabilityResult computeManipulability(const Eigen::Ref<const VectorXd>& q_mani,
                                                                            const Eigen::Ref<const VectorXd>& qdot_mani,
                                                                            const bool& with_grad,
                                                                            const bool& with_graddot,
                                                                            const std::string& link_name) override
                    {
                        return src_.computeManipulability(q_mani, qdot_mani, with_grad, with_graddot, link_name);
                    }

                    /// State update is a no-op: the parent MobileManipulator::RobotData
                    /// keeps all data current via its own updateState().
                    bool updateState(const Eigen::Ref<const VectorXd>&,
                                     const Eigen::Ref<const VectorXd>&) override { return true; }
                };
                ManipulatorProxy mani_proxy_;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                // =====================================================================
                // === Sub-object accessors (preferred API) =============================
                // =====================================================================
                /** @brief Whole-body accessor (world frame, full DOF). Equivalent to *this. */
                RobotData&              moma;
                /** @brief Arm-only accessor (mobile-base frame, manipulator DOF only). */
                Manipulator::RobotData& mani;
                /** @brief Mobile-base accessor. */
                Mobile::RobotData&      mobi;
                /**
                 * @brief Constructor.
                 * @param dt            (double) Control loop time step in seconds.
                 * @param mobile_param  (RobotData::Mobile::KinematicParam) Kinematic parameter structure of the mobile base containing drive type and geometry.
                 * @param joint_idx     (RobotData::JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
                 * @param actuator_idx  (RobotData::ActuatorIndex) Actuator index structure containing starting indices for manipulator and mobile actuators.
                 * @param urdf_source   (std::string) URDF file path or URDF XML string (when use_xml is true).
                 * @param srdf_source   (std::string) SRDF file path or SRDF XML string (when use_xml is true).
                 * @param packages_path (std::string) Path to the packages directory.
                 * @param use_xml       (bool) If true, treat URDF/SRDF inputs as XML strings.
                 */
                RobotData(const double dt,
                          const Mobile::KinematicParam& mobile_param,
                          const JointIndex& joint_idx,
                          const ActuatorIndex& actuator_idx,
                          const std::string& urdf_source,
                          const std::string& srdf_source="",
                          const std::string& packages_path="",
                          const bool use_xml=false);

                using Manipulator::RobotData::getVerbose;
                using Mobile::RobotData::getVerbose;
                /**
                 * @brief Print current mobile manipulator state and parameters in formatted text.
                 * @return (std::string) Human-readable debug information string.
                 */
                std::string getVerbose() const;
                /**
                 * @brief Get control time step.
                 * @return (double) Control loop time step in seconds.
                 */
                double getDt() const { return Mobile::RobotData::getDt(); }
                                               
                using Manipulator::RobotData::updateState; 
                /**
                 * @brief Update internal mobile manipulator robot data.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if state update is successful.
                */
                bool updateState(const Eigen::Ref<const VectorXd>& q_virtual,
                                 const Eigen::Ref<const VectorXd>& q_mobile,
                                 const Eigen::Ref<const VectorXd>& q_mani,
                                 const Eigen::Ref<const VectorXd>& qdot_virtual,
                                 const Eigen::Ref<const VectorXd>& qdot_mobile,
                                 const Eigen::Ref<const VectorXd>& qdot_mani);

                // ================================ Compute Functions ================================
                // Wholebody joint space
                // @deprecated soon — use moma.computeXxx(q_total) or the cached moma.getXxx() / mani.getXxx()
                using Manipulator::RobotData::computeMassMatrix;
                using Manipulator::RobotData::computeGravity; 
                using Manipulator::RobotData::computeCoriolis; 
                using Manipulator::RobotData::computeNonlinearEffects; 
                
                /**
                 * @brief Compute the mass matrix of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::MatrixXd) Mass matrix of the whole body.
                */ 
                virtual MatrixXd computeMassMatrix(const Eigen::Ref<const VectorXd>& q_virtual,
                                                   const Eigen::Ref<const VectorXd>& q_mobile,
                                                   const Eigen::Ref<const VectorXd>& q_mani);
                /**
                 * @brief Compute the gravity vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Gravity vector of the whole body.
                */                                     
                virtual VectorXd computeGravity(const Eigen::Ref<const VectorXd>& q_virtual,
                                                const Eigen::Ref<const VectorXd>& q_mobile,
                                                const Eigen::Ref<const VectorXd>& q_mani);
                /**
                 * @brief Compute the coriolis vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Coriolis vector of the whole body.
                */                                 
                virtual VectorXd computeCoriolis(const Eigen::Ref<const VectorXd>& q_virtual,
                                                 const Eigen::Ref<const VectorXd>& q_mobile,
                                                 const Eigen::Ref<const VectorXd>& q_mani,
                                                 const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                 const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                 const Eigen::Ref<const VectorXd>& qdot_mani);
                /**
                 * @brief Compute the nonlinear effects vector of the whole body.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Joint velocities of the wheels.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the whole body.
                */                                  
                virtual VectorXd computeNonlinearEffects(const Eigen::Ref<const VectorXd>& q_virtual,
                                                         const Eigen::Ref<const VectorXd>& q_mobile,
                                                         const Eigen::Ref<const VectorXd>& q_mani,
                                                         const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                         const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                         const Eigen::Ref<const VectorXd>& qdot_mani);

                /**
                 * @brief Compute the mass matrix of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::MatrixXd) Mass matrix of the actuated joints.
                */                                         
                virtual MatrixXd computeMassMatrixActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                           const Eigen::Ref<const VectorXd>& q_mobile,
                                                           const Eigen::Ref<const VectorXd>& q_mani);
                /**
                 * @brief Compute the gravity vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Gravity vector of the actuated joints.
                */                                              
                virtual VectorXd computeGravityActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                        const Eigen::Ref<const VectorXd>& q_mobile,
                                                        const Eigen::Ref<const VectorXd>& q_mani);
                /**
                 * @brief Compute the coriolis vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Coriolis vector of the actuated joints.
                */                                         
                virtual VectorXd computeCoriolisActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                         const Eigen::Ref<const VectorXd>& q_mobile,
                                                         const Eigen::Ref<const VectorXd>& q_mani,
                                                         const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                         const Eigen::Ref<const VectorXd>& qdot_mani);
                /**
                 * @brief Compute the nonlinear effects vector of the whole body, excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the actuated joints.
                */                                         
                virtual VectorXd computeNonlinearEffectsActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                                 const Eigen::Ref<const VectorXd>& q_mobile,
                                                                 const Eigen::Ref<const VectorXd>& q_mani,
                                                                 const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                                 const Eigen::Ref<const VectorXd>& qdot_mani);

                // Wholebody task space
                // @deprecated soon — use moma.computePose/Jacobian/Velocity(q_total) or moma.getPose/Jacobian/Velocity()
                using Manipulator::RobotData::computePose;
                using Manipulator::RobotData::computeJacobian;
                using Manipulator::RobotData::computeJacobianTimeVariation;
                using Manipulator::RobotData::computeVelocity;
                using Manipulator::RobotData::computeMinDistance;

                /**
                 * @brief Compute the pose of the link in the task space.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::Affine3d) Pose of the link in the task space.
                */ 
                virtual Affine3d computePose(const Eigen::Ref<const VectorXd>& q_virtual,
                                             const Eigen::Ref<const VectorXd>& q_mobile,
                                             const Eigen::Ref<const VectorXd>& q_mani, 
                                             const std::string& link_name);
                /**
                 * @brief Compute the Jacobian of the link.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link.
                */                              
                virtual MatrixXd computeJacobian(const Eigen::Ref<const VectorXd>& q_virtual,
                                                 const Eigen::Ref<const VectorXd>& q_mobile,
                                                 const Eigen::Ref<const VectorXd>& q_mani, 
                                                 const std::string& link_name);
                /**
                 * @brief Compute the Jacobian time variation of the link.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link.
                */                                 
                virtual MatrixXd computeJacobianTimeVariation(const Eigen::Ref<const VectorXd>& q_virtual,
                                                              const Eigen::Ref<const VectorXd>& q_mobile,
                                                              const Eigen::Ref<const VectorXd>& q_mani,
                                                              const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                              const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                              const Eigen::Ref<const VectorXd>& qdot_mani, 
                                                              const std::string& link_name);
                /**
                 * @brief Compute the velocity of the link in the task space.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Velocity of the link in the task space.
                */                                               
                virtual VectorXd computeVelocity(const Eigen::Ref<const VectorXd>& q_virtual,
                                                 const Eigen::Ref<const VectorXd>& q_mobile,
                                                 const Eigen::Ref<const VectorXd>& q_mani,
                                                 const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                 const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                 const Eigen::Ref<const VectorXd>& qdot_mani, 
                                                 const std::string& link_name);
                /**
                 * @brief Compute the minimum pairwise distance between the robot's collision meshes and (optionally) its time variations.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param with_grad     (bool) If true, computes the gradient of the minimum distance.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the minimum distance.
                 * @param verbose       (bool) If true, prints the closest pair of links and their minimum distance.
                 * @return (RobotData::Manipulator::MinDistResult) Minimum distance result containing distance, gradient, and gradient time variation.
                */                                  
                virtual Manipulator::MinDistResult computeMinDistance(const Eigen::Ref<const VectorXd>& q_virtual,
                                                                      const Eigen::Ref<const VectorXd>& q_mobile,
                                                                      const Eigen::Ref<const VectorXd>& q_mani,
                                                                      const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                                      const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                                      const Eigen::Ref<const VectorXd>& qdot_mani, 
                                                                      const bool& with_grad, 
                                                                      const bool& with_graddot, 
                                                                      const bool verbose);
                /**
                 * @brief Compute the selection matrix that excludes floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @return (Eigen::MatrixXd) Selection matrix.
                 */                                                      
                virtual MatrixXd computeSelectionMatrix(const Eigen::Ref<const VectorXd>& q_virtual,
                                                        const Eigen::Ref<const VectorXd>& q_mobile);

                /**
                 * @brief Compute the Jacobian of the link excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link for the actuated joints.
                 */                                        
                virtual MatrixXd computeJacobianActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                         const Eigen::Ref<const VectorXd>& q_mobile,
                                                         const Eigen::Ref<const VectorXd>& q_mani, 
                                                         const std::string& link_name);
                /**
                 * @brief Compute the Jacobian time variation of the link excluding the floating base.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link for the actuated joints.
                 */                                         
                virtual MatrixXd computeJacobianTimeVariationActuated(const Eigen::Ref<const VectorXd>& q_virtual,
                                                                      const Eigen::Ref<const VectorXd>& q_mobile,
                                                                      const Eigen::Ref<const VectorXd>& q_mani,
                                                                      const Eigen::Ref<const VectorXd>& qdot_virtual,
                                                                      const Eigen::Ref<const VectorXd>& qdot_mobile,
                                                                      const Eigen::Ref<const VectorXd>& qdot_mani, 
                                                                      const std::string& link_name);
                                                        
                                                              
                // Manipulator taskspace
                // @deprecated soon — use mani.computeManipulability()
                using Manipulator::RobotData::computeManipulability;
                /**
                 * @brief Compute the manipulability of the link.(which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @param with_grad     (bool) If true, computes the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, computes the gradient time variation of the manipulability.
                 * @param link_name     (std::string) Name of the link.
                 * @return (RobotData::Manipulator::ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                */
                virtual Manipulator::ManipulabilityResult computeManipulability(const Eigen::Ref<const VectorXd>& q_mani, 
                                                                                const Eigen::Ref<const VectorXd>& qdot_mani, 
                                                                                const bool& with_grad, 
                                                                                const bool& with_graddot, 
                                                                                const std::string& link_name);
                                                                        
                // Mobile
                // @deprecated soon — use mobi.computeFKJacobian() / mobi.computeBaseVel()
                /**
                 * @brief Compute the Jacobian of the mobile base specific to the configured drive type.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @return (Eigen::MatrixXd) Jacobian of the mobile base.
                 */
                virtual MatrixXd computeMobileFKJacobian(const Eigen::Ref<const VectorXd>& q_mobile);
                /**
                 * @brief Compute the velocity of the mobile base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @return (Eigen::VectorXd) Base velocity vector [vx, vy, wz].
                 */
                virtual VectorXd computeMobileBaseVel(const Eigen::Ref<const VectorXd>& q_mobile, const Eigen::Ref<const VectorXd>& qdot_mobile);

                // ================================ Get Functions ================================
                /**
                 * @brief Get the number of actuated joints.
                 * @return (int) Number of actuated joints.
                */
                virtual int getActuatorDof() const {return actuated_dof_;}
                /**
                 * @brief Get the degrees of freedom of the manipulator.
                 * @return (int) Degrees of freedom of the manipulator.
                */
                virtual int getManipulatorDof() const {return mani_dof_;}
                /**
                 * @brief Get the mobile base link name (child of the last virtual joint).
                 *        Counterpart to Manipulator::RobotData::getRootLinkName().
                 * @return (std::string) Mobile base link name (e.g., "base_footprint").
                 */
                const std::string& getBaseLinkName() const { return base_link_name_; }
                /**
                 * @brief Get the degrees of freedom of the mobile base.
                 * @return (int) Degrees of freedom of the mobile base.
                */
                virtual int getMobileDof() const {return mobi_dof_;}
                /**
                 * @brief Get the full joint index.
                 * @return (RobotData::MobileManipulator::JointIndex) Joint index structure containing starting indices for virtual, manipulator, and mobile joints.
                */
                virtual JointIndex getJointIndex() const {return joint_idx_;}
                /**
                 * @brief Get the actuator joint index.
                 * @return (RobotData::MobileManipulator::ActuatorIndex) Joint index structure containing starting indices for manipulator and mobile joints.
                */
                virtual ActuatorIndex getActuatorIndex() const {return actuator_idx_;}
                // @deprecated soon — use mobi.getWheelPosition() / mobi.getWheelVelocity() / mobi.getBasePose() / mani.getJointPosition()
                /**
                 * @brief Get the wheel positions.
                 * @return (Eigen::VectorXd) Wheel positions.
                */
                virtual VectorXd getMobileJointPosition() const {return q_mobile_;}
                /**
                 * @brief Get the mobile base pose w.r.t world.
                 * @return (Eigen::VectorXd) Mobile base pose w.r.t world[x, y, yaw].
                */
                virtual VectorXd getVirtualJointPosition() const {return q_virtual_;}
                /**
                 * @brief Get the joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Joint positions of the manipulator.
                */
                virtual VectorXd getManiJointPosition() const {return q_mani_;}
                /**
                 * @brief Get the joint velocities of the actuated joints.
                 * @return (Eigen::VectorXd) Joint velocities of the actuated joints.
                */
                virtual VectorXd getJointVelocityActuated() const {return qdot_actuated_;}
                /**
                 * @brief Get the wheel velocities of the mobile base.
                 * @return (Eigen::VectorXd) Wheel velocities of the mobile base.
                */
                virtual VectorXd getMobileJointVelocity() const {return qdot_mobile_;}
                /**
                 * @brief Get the mobile base velocity w.r.t world.
                 * @return (Eigen::VectorXd) Mobile base velocity w.r.t world[vx, vy, wz].
                */
                virtual VectorXd getVirtualJointVelocity() const {return qdot_virtual_;}
                /**
                 * @brief Get the joint velocities of the manipulator.
                 * @return (Eigen::VectorXd) Joint velocities of the manipulator.
                */
                virtual VectorXd getManiJointVelocity() const {return qdot_mani_;}
                /**
                 * @brief Get the joint positions of the actuated joints.
                 * @return (Eigen::VectorXd) Joint positions of the actuated joints.
                */
                virtual VectorXd getJointPositionActuated() const {return q_actuated_;}
                
                // Wholebody joint space
                /**
                 * @brief Get the mass matrix of the actuated joints.
                 * @return (Eigen::MatrixXd) Mass matrix of the actuated joints.
                 */
                virtual MatrixXd getMassMatrixActuated() const {return M_actuated_;}
                /**
                 * @brief Get the inversed mass matrix of the actuated joints.
                 * @return (Eigen::MatrixXd) Inversed mass matrix of the actuated joints.
                 */
                virtual MatrixXd getMassMatrixActuatedInv() const {return M_inv_actuated_;}
                /**
                 * @brief Get the gravity vector of the actuated joints.
                 * @return (Eigen::VectorXd) Gravity vector of the actuated joints.
                 */
                virtual VectorXd getGravityActuated() const {return g_actuated_;}
                /**
                 * @brief Get the coriolis vector of the actuated joints.
                 * @return (Eigen::VectorXd) Coriolis vector of the actuated joints.
                 */
                virtual VectorXd getCoriolisActuated() const {return c_actuated_;}
                /**
                 * @brief Get the nonlinear effects vector of the actuated joints.
                 * @return (Eigen::VectorXd) Nonlinear effects vector of the actuated joints.
                 */
                virtual VectorXd getNonlinearEffectsActuated() const {return NLE_actuated_;}

                // Wholebody task space
                /**
                 * @brief Get the Jacobian of the link for the actuated joints.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian of the link for the actuated joints.
                 */
                virtual MatrixXd getJacobianActuated(const std::string& link_name);
                /**
                 * @brief Get the Jacobian time variation of the link for the actuated joints.
                 * @param link_name     (std::string) Name of the link.
                 * @return (Eigen::MatrixXd) Jacobian time variation of the link for the actuated joints.
                 */
                virtual MatrixXd getJacobianActuatedTimeVariation(const std::string& link_name);
                /**
                 * @brief Get the selection matrix that excludes floating base.
                 * @return (Eigen::MatrixXd) Selection matrix.
                 */
                virtual MatrixXd getSelectionMatrix(){return S_;}

                // Manipulator taskspace
                using Manipulator::RobotData::getManipulability;
                /**
                 * @brief Get the manipulability of the link.(which indicates how well the link can move at current joint configuration) and (optionally) its time variations.
                 * @param with_grad     (bool) If true, get the gradient of the manipulability.
                 * @param with_graddot  (bool) If true, get the time variation of the gradient.
                 * @param link_name     (std::string) Name of the link.
                 * @return (RobotData::Manipulator::ManipulabilityResult) Manipulability result containing manipulability, gradient, and gradient time variation.
                 */
                virtual Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, 
                                                                            const bool& with_graddot, 
                                                                            const std::string& link_name);

                // Mobile
                // @deprecated soon — use mobi.getFKJacobian() / mobi.getBaseVel()
                /**
                 * @brief Get the Jacobian of the mobile base specific to the configured drive type.
                 * @return (Eigen::MatrixXd) Jacobian of the mobile base.
                 */
                virtual MatrixXd getMobileFKJacobian() const {return J_mobile_;}
                /**
                 * @brief Get the velocity of the mobile base.
                 * @return (Eigen::VectorXd) Base velocity vector [vx, vy, wz].
                 */
                virtual VectorXd getMobileBaseVel() const {return Mobile::RobotData::getBaseVel();}
        

            protected:
                using Manipulator::RobotData::updateKinematics;
                using Manipulator::RobotData::updateDynamics;
                /**
                 * @brief Update the kinematic parameters of the mobile manipulator.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if the update was successful.
                 */
                virtual bool updateKinematics(const Eigen::Ref<const VectorXd>& q_virtual,
                                              const Eigen::Ref<const VectorXd>& q_mobile,
                                              const Eigen::Ref<const VectorXd>& q_mani,
                                              const Eigen::Ref<const VectorXd>& qdot_virtual,
                                              const Eigen::Ref<const VectorXd>& qdot_mobile,
                                              const Eigen::Ref<const VectorXd>& qdot_mani);
                /**
                 * @brief Update the dynamics of the mobile manipulator.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @param qdot_virtual  (Eigen::VectorXd) Joint velocities of the floating base.
                 * @param qdot_mobile   (Eigen::VectorXd) Wheel velocities.
                 * @param qdot_mani     (Eigen::VectorXd) Joint velocities of the manipulator.
                 * @return (bool) True if the update was successful.
                 */                              
                virtual bool updateDynamics(const Eigen::Ref<const VectorXd>& q_virtual,
                                            const Eigen::Ref<const VectorXd>& q_mobile,
                                            const Eigen::Ref<const VectorXd>& q_mani,
                                            const Eigen::Ref<const VectorXd>& qdot_virtual,
                                            const Eigen::Ref<const VectorXd>& qdot_mobile,
                                            const Eigen::Ref<const VectorXd>& qdot_mani);

                /**
                 * @brief Combine virtual, mobile, and manipulator joint vectors into a single joint vector.
                 * @param q_virtual     (Eigen::VectorXd) Joint positions of the floating base.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Combined joint vector.
                 */                             
                VectorXd getJointVector(const Eigen::Ref<const VectorXd>& q_virtual,
                                        const Eigen::Ref<const VectorXd>& q_mobile,
                                        const Eigen::Ref<const VectorXd>& q_mani);
                /**
                 * @brief Combine mobile and manipulator joint vectors into a single actuator joint vector.
                 * @param q_mobile      (Eigen::VectorXd) Wheel positions.
                 * @param q_mani        (Eigen::VectorXd) Joint positions of the manipulator.
                 * @return (Eigen::VectorXd) Combined actuator joint vector.
                 */                         
                VectorXd getActuatorVector(const Eigen::Ref<const VectorXd>& q_mobile,
                                           const Eigen::Ref<const VectorXd>& q_mani);

                int mani_dof_;          // Manipulator degrees of freedom
                int mobi_dof_;          // Mobile base degrees of freedom
                int virtual_dof_{3};    // Virtual joint degrees of freedom
                int actuated_dof_;      // Actuated joint degrees of freedom

                std::string base_link_name_;  // Child link of the last virtual joint (mobile base link)
        
                JointIndex joint_idx_;          // Starting index for joints
                ActuatorIndex actuator_idx_;    // Starting index for actuators
                        
                // Selection Matrix for virtual joints (Actuated Joint Velocity -> Total Joint Velocity)
                MatrixXd S_;
                // MatrixXd Sdot_;
                
                VectorXd q_virtual_;        // Joint positions of the floating base.
                VectorXd q_mobile_;         // Wheel positions
                VectorXd q_mani_;           // Joint positions of the manipulator.
                VectorXd qdot_virtual_;     // Joint velocities of the floating base.
                VectorXd qdot_mobile_;      // Wheel velocities
                VectorXd qdot_mani_;        // Joint velocities of the manipulator.

                // Actuated Joint state
                VectorXd q_actuated_;        // Joint positions of the actuated joints.
                VectorXd qdot_actuated_;     // Joint velocities of the actuated joints.
                
                // ActuatedJoint space Dynamics
                MatrixXd M_actuated_;     // Mass matrix for the actuated joints.
                MatrixXd M_inv_actuated_; // Inversed Mass matrix for the actuated joints.
                VectorXd g_actuated_;     // Gravity vector for the actuated joints.
                VectorXd c_actuated_;     // Coriolis vector for the actuated joints.
                VectorXd NLE_actuated_;   // Nonlinear effects vector for the actuated joints.
        };
    } // namespace MobileManipulator
} // namespace drc
