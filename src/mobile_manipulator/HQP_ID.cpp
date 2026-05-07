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

#include "dyros_robot_controller/mobile_manipulator/HQP_ID.h"
#include <cmath>
#include <iostream>

namespace drc
{
namespace MobileManipulator
{

// =============================================================================
//  HQP_ID_base
// =============================================================================

HQP_ID_base::HQP_ID_base(std::shared_ptr<MobileManipulator::RobotData> robot_data,
                           const double dt,
                           const int neqc,
                           const int nshared_ineq)
: QP::QPBase(), robot_data_(robot_data), dt_(dt), nshared_ineq_(nshared_ineq)
{
    if (dt_ <= 1e-9)
        std::cerr << "[HQP_ID_base] WARNING: constructed with dt=" << dt_ << std::endl;

    actuator_dof_ = robot_data_->getActuatorDof();
    mani_dof_     = robot_data_->getManipulatorDof();
    mobi_dof_     = robot_data_->getMobileDof();

    // ---- decision variable sizes ----
    si_index_.eta_dot_size              = actuator_dof_;
    si_index_.torque_size               = actuator_dof_;
    si_index_.slack_q_mani_min_size     = mani_dof_;
    si_index_.slack_q_mani_max_size     = mani_dof_;
    si_index_.slack_qdot_mani_min_size  = mani_dof_;
    si_index_.slack_qdot_mani_max_size  = mani_dof_;
    si_index_.slack_sing_size           = 1;
    si_index_.slack_sel_col_size        = 1;
    si_index_.slack_base_vel_min_size   = 3;
    si_index_.slack_base_vel_max_size   = 3;
    si_index_.slack_base_acc_min_size   = 3;
    si_index_.slack_base_acc_max_size   = 3;

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
                   si_index_.slack_base_acc_max_size;  // 2*a + 4*m + 14

    // ---- constraint sizes ----
    si_index_.con_dyn_size    = actuator_dof_;
    si_index_.con_hp_size     = neqc;
    si_index_.con_shared_size = nshared_ineq_;

    const int nbound     = nx;
    const int nineqc     = si_index_.con_shared_size;
    const int total_neqc = si_index_.con_dyn_size + si_index_.con_hp_size;

    QPBase::setQPsize(nx, nbound, nineqc, total_neqc);

    // ---- decision variable starts (sequential layout) ----
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

    // ---- equality constraint starts ----
    si_index_.con_dyn_start = 0;
    si_index_.con_hp_start  = si_index_.con_dyn_start + si_index_.con_dyn_size;

    // ---- inequality constraint starts ----
    si_index_.con_shared_start = 0;

    // ---- default weights ----
    w_tracking_.setOnes();
    w_vel_damping_.setOnes(actuator_dof_);
    w_acc_damping_.setOnes(actuator_dof_);
}

void HQP_ID_base::setDesiredTaskAcc(const std::map<std::string, Vector6d>& link_xddot_desired)
{
    link_xddot_desired_ = link_xddot_desired;
}

void HQP_ID_base::setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& etadot_ref)
{
    J_eq_ = J_eq;
    v_eq_ = J_eq * etadot_ref;  // Jdot * eta cancels in equality constraint
}

void HQP_ID_base::setSharedIneqConstraints(const SharedIneqDataID& shared)
{
    shared_ineq_ = shared;
}

void HQP_ID_base::setTrackingWeight(const Vector6d& w)
{
    w_tracking_           = w;
    use_uniform_tracking_ = true;
}

void HQP_ID_base::setTrackingWeight(const std::map<std::string, Vector6d>& link_w)
{
    link_w_tracking_      = link_w;
    use_uniform_tracking_ = false;
}

bool HQP_ID_base::getOptJoint(Eigen::Ref<VectorXd> opt_etadot,
                               Eigen::Ref<VectorXd> opt_torque,
                               QP::TimeDuration& time_status)
{
    if (opt_etadot.size() != actuator_dof_ || opt_torque.size() != actuator_dof_)
    {
        std::cerr << "[HQP_ID_base] size mismatch: opt_etadot=" << opt_etadot.size()
                  << " opt_torque=" << opt_torque.size()
                  << " actuator_dof_=" << actuator_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    if (si_index_.con_hp_size > 0 && (J_eq_.rows() != si_index_.con_hp_size || v_eq_.size() != si_index_.con_hp_size))
    {
        std::cerr << "[HQP_ID_base] higher-priority constraint data missing or wrong size. "
                  << "con_hp_size=" << si_index_.con_hp_size
                  << " J_eq_.rows()=" << J_eq_.rows()
                  << " v_eq_.size()=" << v_eq_.size() << std::endl;
        opt_etadot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    MatrixXd sol;
    if (!solveQP(sol, time_status))
    {
        std::cerr << "[HQP_ID_base] QP solve failed." << std::endl;
        opt_etadot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    const VectorXd etadot_sol = sol.block(si_index_.eta_dot_start, 0, si_index_.eta_dot_size, 1);
    const VectorXd torque_sol = sol.block(si_index_.torque_start,  0, si_index_.torque_size,  1);

    if (!etadot_sol.allFinite() || !torque_sol.allFinite())
    {
        std::cerr << "[HQP_ID_base] solution is non-finite." << std::endl;
        opt_etadot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    opt_etadot = etadot_sol;
    opt_torque = torque_sol;
    return true;
}

void HQP_ID_base::setCost()
{
    P_ds_.setZero(nx_, nx_);
    q_ds_.setZero(nx_);

    const VectorXd eta = robot_data_->getJointVelocityActuated();

    // Task-space acceleration tracking: Σ_i || J_i_tilda*eta_dot + J̇_i_tilda*eta - ẍ_des_i ||²_Wi
    for (const auto& [link_name, xddot_desired] : link_xddot_desired_)
    {
        const MatrixXd J_i_tilda     = robot_data_->getJacobianActuated(link_name);
        const MatrixXd J_i_tilda_dot = robot_data_->getJacobianActuatedTimeVariation(link_name);

        Vector6d w;
        if (use_uniform_tracking_)
        {
            w = w_tracking_;
        }
        else
        {
            const auto it = link_w_tracking_.find(link_name);
            w = (it != link_w_tracking_.end()) ? it->second : Vector6d::Ones();
        }

        P_ds_.block(si_index_.eta_dot_start, si_index_.eta_dot_start, si_index_.eta_dot_size, si_index_.eta_dot_size) +=
            2.0 * J_i_tilda.transpose() * w.asDiagonal() * J_i_tilda;
        q_ds_.segment(si_index_.eta_dot_start, si_index_.eta_dot_size) +=
            -2.0 * J_i_tilda.transpose() * w.asDiagonal() * (xddot_desired - J_i_tilda_dot * eta);
    }

    // Actuator acceleration damping: ||eta_dot||²_Wacc
    P_ds_.block(si_index_.eta_dot_start, si_index_.eta_dot_start, si_index_.eta_dot_size, si_index_.eta_dot_size) +=
        2.0 * w_acc_damping_.asDiagonal();

    // Actuator velocity damping: ||dt*eta_dot + eta||²_Wvel
    P_ds_.block(si_index_.eta_dot_start, si_index_.eta_dot_start, si_index_.eta_dot_size, si_index_.eta_dot_size) +=
        2.0 * dt_ * dt_ * w_vel_damping_.asDiagonal();
    q_ds_.segment(si_index_.eta_dot_start, si_index_.eta_dot_size) +=
        2.0 * dt_ * w_vel_damping_.asDiagonal() * eta;

    // Linear slack penalty: 1000 * Σs
    q_ds_.segment(si_index_.slack_q_mani_min_start,    si_index_.slack_q_mani_min_size)   .setConstant(1000.0);
    q_ds_.segment(si_index_.slack_q_mani_max_start,    si_index_.slack_q_mani_max_size)   .setConstant(1000.0);
    q_ds_.segment(si_index_.slack_qdot_mani_min_start, si_index_.slack_qdot_mani_min_size).setConstant(1000.0);
    q_ds_.segment(si_index_.slack_qdot_mani_max_start, si_index_.slack_qdot_mani_max_size).setConstant(1000.0);
    q_ds_(si_index_.slack_sing_start)    = 1000.0;
    q_ds_(si_index_.slack_sel_col_start) = 1000.0;
    q_ds_.segment(si_index_.slack_base_vel_min_start, si_index_.slack_base_vel_min_size).setConstant(1000.0);
    q_ds_.segment(si_index_.slack_base_vel_max_start, si_index_.slack_base_vel_max_size).setConstant(1000.0);
    q_ds_.segment(si_index_.slack_base_acc_min_start, si_index_.slack_base_acc_min_size).setConstant(1000.0);
    q_ds_.segment(si_index_.slack_base_acc_max_start, si_index_.slack_base_acc_max_size).setConstant(1000.0);
}

void HQP_ID_base::setBoundConstraint()
{
    l_bound_ds_.setConstant(nbc_, -OSQP_INFTY);
    u_bound_ds_.setConstant(nbc_,  OSQP_INFTY);

    // slack >= 0 (eta_dot and torque remain unbounded)
    l_bound_ds_.segment(si_index_.slack_q_mani_min_start,    si_index_.slack_q_mani_min_size)   .setZero();
    l_bound_ds_.segment(si_index_.slack_q_mani_max_start,    si_index_.slack_q_mani_max_size)   .setZero();
    l_bound_ds_.segment(si_index_.slack_qdot_mani_min_start, si_index_.slack_qdot_mani_min_size).setZero();
    l_bound_ds_.segment(si_index_.slack_qdot_mani_max_start, si_index_.slack_qdot_mani_max_size).setZero();
    l_bound_ds_(si_index_.slack_sing_start)    = 0.0;
    l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
    l_bound_ds_.segment(si_index_.slack_base_vel_min_start, si_index_.slack_base_vel_min_size).setZero();
    l_bound_ds_.segment(si_index_.slack_base_vel_max_start, si_index_.slack_base_vel_max_size).setZero();
    l_bound_ds_.segment(si_index_.slack_base_acc_min_start, si_index_.slack_base_acc_min_size).setZero();
    l_bound_ds_.segment(si_index_.slack_base_acc_max_start, si_index_.slack_base_acc_max_size).setZero();
}

void HQP_ID_base::setIneqConstraint()
{
    A_ineq_ds_.setZero(nineqc_, nx_);
    l_ineq_ds_.setConstant(nineqc_, -OSQP_INFTY);
    u_ineq_ds_.setConstant(nineqc_,  OSQP_INFTY);

    if (nshared_ineq_ > 0 && shared_ineq_.A.rows() == nshared_ineq_)
    {
        const int rs = si_index_.con_shared_start;

        // eta_dot columns
        A_ineq_ds_.block(rs, si_index_.eta_dot_start, nshared_ineq_, si_index_.eta_dot_size) = shared_ineq_.A;
        l_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.l;
        u_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.u;

        // slack columns — each constraint type has its own slack variable
        A_ineq_ds_.block(rs + shared_ineq_.q_mani_min_start,    si_index_.slack_q_mani_min_start,
                         shared_ineq_.q_mani_min_size,            si_index_.slack_q_mani_min_size)    = MatrixXd::Identity(shared_ineq_.q_mani_min_size,    si_index_.slack_q_mani_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.q_mani_max_start,    si_index_.slack_q_mani_max_start,
                         shared_ineq_.q_mani_max_size,            si_index_.slack_q_mani_max_size)    = MatrixXd::Identity(shared_ineq_.q_mani_max_size,    si_index_.slack_q_mani_max_size);
        A_ineq_ds_.block(rs + shared_ineq_.qdot_mani_min_start, si_index_.slack_qdot_mani_min_start,
                         shared_ineq_.qdot_mani_min_size,         si_index_.slack_qdot_mani_min_size) = MatrixXd::Identity(shared_ineq_.qdot_mani_min_size, si_index_.slack_qdot_mani_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.qdot_mani_max_start, si_index_.slack_qdot_mani_max_start,
                         shared_ineq_.qdot_mani_max_size,         si_index_.slack_qdot_mani_max_size) = MatrixXd::Identity(shared_ineq_.qdot_mani_max_size, si_index_.slack_qdot_mani_max_size);
        A_ineq_ds_(rs + shared_ineq_.sel_col_start, si_index_.slack_sel_col_start) = 1.0;
        A_ineq_ds_(rs + shared_ineq_.sing_start,    si_index_.slack_sing_start)    = 1.0;
        A_ineq_ds_.block(rs + shared_ineq_.base_vel_min_start, si_index_.slack_base_vel_min_start,
                         shared_ineq_.base_vel_min_size,          si_index_.slack_base_vel_min_size) = MatrixXd::Identity(shared_ineq_.base_vel_min_size, si_index_.slack_base_vel_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.base_vel_max_start, si_index_.slack_base_vel_max_start,
                         shared_ineq_.base_vel_max_size,          si_index_.slack_base_vel_max_size) = MatrixXd::Identity(shared_ineq_.base_vel_max_size, si_index_.slack_base_vel_max_size);
        A_ineq_ds_.block(rs + shared_ineq_.base_acc_min_start, si_index_.slack_base_acc_min_start,
                         shared_ineq_.base_acc_min_size,          si_index_.slack_base_acc_min_size) = MatrixXd::Identity(shared_ineq_.base_acc_min_size, si_index_.slack_base_acc_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.base_acc_max_start, si_index_.slack_base_acc_max_start,
                         shared_ineq_.base_acc_max_size,          si_index_.slack_base_acc_max_size) = MatrixXd::Identity(shared_ineq_.base_acc_max_size, si_index_.slack_base_acc_max_size);
    }
}

void HQP_ID_base::setEqConstraint()
{
    A_eq_ds_.setZero(neqc_, nx_);
    b_eq_ds_.setZero(neqc_);

    // Dynamics: M_tilda * eta_dot - torque = -nle_tilda
    const MatrixXd M_tilda   = robot_data_->getMassMatrixActuated();
    const MatrixXd nle_tilda = robot_data_->getNonlinearEffectsActuated();

    A_eq_ds_.block(si_index_.con_dyn_start, si_index_.eta_dot_start, si_index_.con_dyn_size, si_index_.eta_dot_size) = M_tilda;
    A_eq_ds_.block(si_index_.con_dyn_start, si_index_.torque_start,  si_index_.con_dyn_size, si_index_.torque_size)  = -MatrixXd::Identity(si_index_.con_dyn_size, si_index_.torque_size);
    b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -nle_tilda;

    // Higher-priority task accelerations: J_eq * eta_dot = v_eq
    if (si_index_.con_hp_size > 0 && J_eq_.rows() == si_index_.con_hp_size)
    {
        A_eq_ds_.block(si_index_.con_hp_start, si_index_.eta_dot_start, si_index_.con_hp_size, si_index_.eta_dot_size) = J_eq_;
        b_eq_ds_.segment(si_index_.con_hp_start, si_index_.con_hp_size) = v_eq_;
    }
}

// =============================================================================
//  HQPID
// =============================================================================

HQPID::HQPID(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt)
: robot_data_(robot_data), dt_(dt)
{
    actuator_dof_ = robot_data_->getActuatorDof();
    mani_dof_     = robot_data_->getManipulatorDof();
    mobi_dof_     = robot_data_->getMobileDof();

    si_index_.con_q_mani_min_size    = mani_dof_;
    si_index_.con_q_mani_max_size    = mani_dof_;
    si_index_.con_qdot_mani_min_size = mani_dof_;
    si_index_.con_qdot_mani_max_size = mani_dof_;
    si_index_.con_sel_col_size       = 1;
    si_index_.con_sing_size          = 1;
    si_index_.con_base_vel_min_size  = 3;
    si_index_.con_base_vel_max_size  = 3;
    si_index_.con_base_acc_min_size  = 3;
    si_index_.con_base_acc_max_size  = 3;

    si_index_.con_q_mani_min_start    = 0;
    si_index_.con_q_mani_max_start    = si_index_.con_q_mani_min_start    + si_index_.con_q_mani_min_size;
    si_index_.con_qdot_mani_min_start = si_index_.con_q_mani_max_start    + si_index_.con_q_mani_max_size;
    si_index_.con_qdot_mani_max_start = si_index_.con_qdot_mani_min_start + si_index_.con_qdot_mani_min_size;
    si_index_.con_sel_col_start       = si_index_.con_qdot_mani_max_start + si_index_.con_qdot_mani_max_size;
    si_index_.con_sing_start          = si_index_.con_sel_col_start       + si_index_.con_sel_col_size;
    si_index_.con_base_vel_min_start  = si_index_.con_sing_start          + si_index_.con_sing_size;
    si_index_.con_base_vel_max_start  = si_index_.con_base_vel_min_start  + si_index_.con_base_vel_min_size;
    si_index_.con_base_acc_min_start  = si_index_.con_base_vel_max_start  + si_index_.con_base_vel_max_size;
    si_index_.con_base_acc_max_start  = si_index_.con_base_acc_min_start  + si_index_.con_base_acc_min_size;

    nshared_ineq_ = si_index_.con_q_mani_min_size +
                    si_index_.con_q_mani_max_size +
                    si_index_.con_qdot_mani_min_size +
                    si_index_.con_qdot_mani_max_size +
                    si_index_.con_sel_col_size +
                    si_index_.con_sing_size +
                    si_index_.con_base_vel_min_size +
                    si_index_.con_base_vel_max_size +
                    si_index_.con_base_acc_min_size +
                    si_index_.con_base_acc_max_size;  // 4*m + 14

    w_vel_damping_.setOnes(actuator_dof_);
    w_acc_damping_.setOnes(actuator_dof_);
    uniform_w_tracking_.setOnes();
}

void HQPID::setDesiredTaskAcc(const std::vector<std::map<std::string, Vector6d>>& tasks)
{
    tasks_ = tasks;

    std::vector<int> current_structure;
    current_structure.reserve(tasks.size());
    for (const auto& level : tasks) current_structure.push_back(static_cast<int>(level.size()));

    if (current_structure != task_structure_)
    {
        initializeLevels(tasks);
        task_structure_ = current_structure;
    }
}

void HQPID::initializeLevels(const std::vector<std::map<std::string, Vector6d>>& tasks)
{
    qp_levels_.clear();
    int accumulated_eq_rows = 0;
    for (const auto& level : tasks)
    {
        qp_levels_.push_back(std::make_unique<HQP_ID_base>(robot_data_, dt_, accumulated_eq_rows, nshared_ineq_));
        accumulated_eq_rows += static_cast<int>(6 * level.size());
    }
}

SharedIneqDataID HQPID::computeSharedConstraints()
{
    const int mani_start_act = robot_data_->getActuatorIndex().mani_start;
    const int mani_start_jnt = robot_data_->getJointIndex().mani_start;
    const int mobi_start     = robot_data_->getActuatorIndex().mobi_start;

    SharedIneqDataID data;
    data.A.setZero(nshared_ineq_, actuator_dof_);
    data.l.setConstant(nshared_ineq_, -OSQP_INFTY);
    data.u.setConstant(nshared_ineq_,  OSQP_INFTY);

    // Embed row-range info so HQP_ID_base::setIneqConstraint() can assign slack columns
    data.q_mani_min_start    = si_index_.con_q_mani_min_start;    data.q_mani_min_size    = si_index_.con_q_mani_min_size;
    data.q_mani_max_start    = si_index_.con_q_mani_max_start;    data.q_mani_max_size    = si_index_.con_q_mani_max_size;
    data.qdot_mani_min_start = si_index_.con_qdot_mani_min_start; data.qdot_mani_min_size = si_index_.con_qdot_mani_min_size;
    data.qdot_mani_max_start = si_index_.con_qdot_mani_max_start; data.qdot_mani_max_size = si_index_.con_qdot_mani_max_size;
    data.sel_col_start       = si_index_.con_sel_col_start;       data.sel_col_size       = si_index_.con_sel_col_size;
    data.sing_start          = si_index_.con_sing_start;          data.sing_size          = si_index_.con_sing_size;
    data.base_vel_min_start  = si_index_.con_base_vel_min_start;  data.base_vel_min_size  = si_index_.con_base_vel_min_size;
    data.base_vel_max_start  = si_index_.con_base_vel_max_start;  data.base_vel_max_size  = si_index_.con_base_vel_max_size;
    data.base_acc_min_start  = si_index_.con_base_acc_min_start;  data.base_acc_min_size  = si_index_.con_base_acc_min_size;
    data.base_acc_max_start  = si_index_.con_base_acc_max_start;  data.base_acc_max_size  = si_index_.con_base_acc_max_size;

    const double alpha = 100.0;

    const VectorXd q_actuated    = robot_data_->getJointPositionActuated();
    const VectorXd qdot_actuated = robot_data_->getJointVelocityActuated();
    const VectorXd q_mani        = q_actuated.segment(mani_start_act,    mani_dof_);
    const VectorXd qdot_mani     = qdot_actuated.segment(mani_start_act, mani_dof_);

    // ---- manipulator joint position limits (2nd-order CBF) ----
    const auto     q_lim          = robot_data_->getJointPositionLimit();
    const VectorXd q_mani_min_raw  = q_lim.first.segment(mani_start_jnt,  mani_dof_);
    const VectorXd q_mani_max_raw  = q_lim.second.segment(mani_start_jnt, mani_dof_);
    const VectorXd q_mani_min =
        (q_mani_min_raw.array() < 0.0).select(q_mani_min_raw.array() * 0.9, q_mani_min_raw.array() * 1.1).matrix();
    const VectorXd q_mani_max =
        (q_mani_max_raw.array() > 0.0).select(q_mani_max_raw.array() * 0.9, q_mani_max_raw.array() * 1.1).matrix();

    // qddot_mani >= -(2a)*qdot_mani - a²*(q_mani - q_mani_min)
    data.A.block(si_index_.con_q_mani_min_start, mani_start_act, mani_dof_, mani_dof_) =  MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_q_mani_min_start, mani_dof_) = -(alpha + alpha) * qdot_mani - alpha * alpha * (q_mani - q_mani_min);

    // -qddot_mani >= +(2a)*qdot_mani - a²*(q_mani_max - q_mani)
    data.A.block(si_index_.con_q_mani_max_start, mani_start_act, mani_dof_, mani_dof_) = -MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_q_mani_max_start, mani_dof_) = +(alpha + alpha) * qdot_mani - alpha * alpha * (q_mani_max - q_mani);

    // ---- manipulator joint velocity limits (1st-order CBF) ----
    const auto     qdot_lim          = robot_data_->getJointVelocityLimit();
    const VectorXd qdot_mani_min_raw  = qdot_lim.first.segment(mani_start_jnt,  mani_dof_);
    const VectorXd qdot_mani_max_raw  = qdot_lim.second.segment(mani_start_jnt, mani_dof_);
    const VectorXd qdot_mani_min =
        (qdot_mani_min_raw.array() < 0.0).select(qdot_mani_min_raw.array() * 0.9, qdot_mani_min_raw.array() * 1.1).matrix();
    const VectorXd qdot_mani_max =
        (qdot_mani_max_raw.array() > 0.0).select(qdot_mani_max_raw.array() * 0.9, qdot_mani_max_raw.array() * 1.1).matrix();

    // qddot_mani >= -a*(qdot_mani - qdot_mani_min)
    data.A.block(si_index_.con_qdot_mani_min_start, mani_start_act, mani_dof_, mani_dof_) =  MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_qdot_mani_min_start, mani_dof_) = -alpha * (qdot_mani - qdot_mani_min);

    // -qddot_mani >= -a*(qdot_mani_max - qdot_mani)
    data.A.block(si_index_.con_qdot_mani_max_start, mani_start_act, mani_dof_, mani_dof_) = -MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_qdot_mani_max_start, mani_dof_) = -alpha * (qdot_mani_max - qdot_mani);

    // ---- self-collision avoidance CBF (2nd-order) ----
    const Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, true, false);
    const bool valid_col_grad =
        std::isfinite(min_dist_res.distance)                               &&
        min_dist_res.grad.size()     >= mani_start_jnt + mani_dof_         &&
        min_dist_res.grad_dot.size() >= mani_start_jnt + mani_dof_         &&
        min_dist_res.grad.allFinite()                                       &&
        min_dist_res.grad_dot.allFinite()                                   &&
        min_dist_res.grad.segment(mani_start_jnt, mani_dof_).squaredNorm() > 1e-12;

    if (valid_col_grad)
    {
        const VectorXd col_grad     = min_dist_res.grad.segment(mani_start_jnt,     mani_dof_);
        const VectorXd col_grad_dot = min_dist_res.grad_dot.segment(mani_start_jnt, mani_dof_);

        if (!col_grad_initialized_)
        {
            col_grad_filtered_     = col_grad;
            col_grad_dot_filtered_ = col_grad_dot;
            col_grad_initialized_  = true;
        }
        else
        {
            col_grad_filtered_     = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                                   + col_grad_filter_alpha_ * col_grad;
            col_grad_dot_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_dot_filtered_
                                   + col_grad_filter_alpha_ * col_grad_dot;
        }

        // ∇d·qddot_mani >= -∇̇d·qdot_mani - 2a*∇d·qdot_mani - a²*(d - eps)
        data.A.block(si_index_.con_sel_col_start, mani_start_act, 1, mani_dof_) = col_grad_filtered_.transpose();
        data.l(si_index_.con_sel_col_start) =
            -col_grad_dot_filtered_.dot(qdot_mani)
            - (alpha + alpha) * col_grad_filtered_.dot(qdot_mani)
            - alpha * alpha * (min_dist_res.distance - 0.01);
    }
    else
    {
        col_grad_initialized_ = false;
        // con_sel_col row stays trivial
    }

    // ---- singularity CBF (reserved – trivial row) ----
    // const ManipulabilityResult mani_data = robot_data_->getManipulability(true, true, link_name);
    // data.A.block(si_index_.con_sing_start, mani_start_act, 1, mani_dof_) = mani_data.grad.transpose();
    // data.l(si_index_.con_sing_start) = -mani_data.grad_dot.dot(qdot_mani)
    //     - 2*alpha*mani_data.grad.dot(qdot_mani) - alpha*alpha*(mani_data.manipulability - 0.01);

    // ---- mobile base velocity limit (1st-order CBF, soft) ----
    const auto&    param    = robot_data_->getKineParam();
    const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();
    const Vector3d base_vel = robot_data_->getMobileBaseVel();

    Vector3d vel_limit;
    vel_limit << param.max_lin_speed, param.max_lin_speed, param.max_ang_speed;
    const Vector3d vel_min = -vel_limit;
    const Vector3d vel_max =  vel_limit;

    // J_mob * eta_dot_mobi >= -a*(v_base - v_min)
    data.A.block(si_index_.con_base_vel_min_start, mobi_start, 3, mobi_dof_) =  J_mobile;
    data.l.segment(si_index_.con_base_vel_min_start, 3) = -alpha * (base_vel - vel_min);

    // -J_mob * eta_dot_mobi >= -a*(v_max - v_base)
    data.A.block(si_index_.con_base_vel_max_start, mobi_start, 3, mobi_dof_) = -J_mobile;
    data.l.segment(si_index_.con_base_vel_max_start, 3) = -alpha * (vel_max - base_vel);

    // ---- mobile base acceleration limit (soft hard limit) ----
    Vector3d acc_limit;
    acc_limit << param.max_lin_acc, param.max_lin_acc, param.max_ang_acc;

    // J_mob * eta_dot_mobi >= -acc_limit
    data.A.block(si_index_.con_base_acc_min_start, mobi_start, 3, mobi_dof_) =  J_mobile;
    data.l.segment(si_index_.con_base_acc_min_start, 3) = -acc_limit;

    // -J_mob * eta_dot_mobi >= -acc_limit
    data.A.block(si_index_.con_base_acc_max_start, mobi_start, 3, mobi_dof_) = -J_mobile;
    data.l.segment(si_index_.con_base_acc_max_start, 3) = -acc_limit;

    return data;
}

bool HQPID::getOptJoint(Eigen::Ref<VectorXd> opt_etadot, Eigen::Ref<VectorXd> opt_torque, QP::TimeDuration& time_status)
{
    if (opt_etadot.size() != actuator_dof_ || opt_torque.size() != actuator_dof_)
    {
        std::cerr << "[HQPID] size mismatch: opt_etadot=" << opt_etadot.size()
                  << " opt_torque=" << opt_torque.size()
                  << " actuator_dof_=" << actuator_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    const auto& tasks = tasks_;
    if (tasks.empty())
    {
        opt_etadot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return true;
    }

    // Compute all shared constraints once for this cycle
    const SharedIneqDataID shared = computeSharedConstraints();

    // Hierarchical solve
    time_status.setZero();
    VectorXd etadot_result = VectorXd::Zero(actuator_dof_);
    VectorXd torque_result = VectorXd::Zero(actuator_dof_);
    bool all_ok = true;

    MatrixXd J_eq_stacked(0, actuator_dof_);

    for (int k = 0; k < static_cast<int>(tasks.size()); k++)
    {
        const std::map<std::string, Vector6d>& level = tasks[k];

        qp_levels_[k]->setDesiredTaskAcc(level);

        if (k > 0) qp_levels_[k]->setHigherPriorityConstraints(J_eq_stacked, etadot_result);

        qp_levels_[k]->setSharedIneqConstraints(shared);
        qp_levels_[k]->setJointVelWeight(w_vel_damping_);
        qp_levels_[k]->setJointAccWeight(w_acc_damping_);

        if (use_uniform_tracking_) qp_levels_[k]->setTrackingWeight(uniform_w_tracking_);
        else                       qp_levels_[k]->setTrackingWeight(link_w_tracking_);

        VectorXd etadot_k(actuator_dof_), torque_k(actuator_dof_);
        QP::TimeDuration level_time;
        const bool ok = qp_levels_[k]->getOptJoint(etadot_k, torque_k, level_time);

        time_status.set_cost       += level_time.set_cost;
        time_status.set_bound      += level_time.set_bound;
        time_status.set_ineq       += level_time.set_ineq;
        time_status.set_eq         += level_time.set_eq;
        time_status.set_constraint += level_time.set_constraint;
        time_status.set_qp         += level_time.set_qp;
        time_status.set_solver     += level_time.set_solver;
        time_status.solve_qp       += level_time.solve_qp;

        if (!ok)
        {
            std::cerr << "[HQPID] level " << k << " solve failed, stopping hierarchy." << std::endl;
            all_ok = false;
            break;
        }

        etadot_result = etadot_k;
        torque_result = torque_k;

        // Append this level's actuated Jacobian rows to the stacked equality constraint matrix
        const int new_rows = static_cast<int>(6 * level.size());
        const int old_rows = static_cast<int>(J_eq_stacked.rows());
        MatrixXd J_eq_new(old_rows + new_rows, actuator_dof_);
        if (old_rows > 0) J_eq_new.topRows(old_rows) = J_eq_stacked;
        int row = 0;
        for (const auto& [link, _] : level)
        {
            J_eq_new.block(old_rows + row, 0, 6, actuator_dof_) = robot_data_->getJacobianActuated(link);
            row += 6;
        }
        J_eq_stacked = std::move(J_eq_new);
    }

    opt_etadot = etadot_result;
    opt_torque = torque_result;
    return all_ok;
}

} // namespace MobileManipulator
} // namespace drc
