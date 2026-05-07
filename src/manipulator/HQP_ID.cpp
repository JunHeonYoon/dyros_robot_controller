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

#include "dyros_robot_controller/manipulator/HQP_ID.h"
#include <cmath>
#include <iostream>

namespace drc
{
namespace Manipulator
{

// =============================================================================
//  HQP_ID_base
// =============================================================================

HQP_ID_base::HQP_ID_base(std::shared_ptr<Manipulator::RobotData> robot_data,
                           const double dt,
                           const int neqc,
                           const int nshared_ineq)
: QP::QPBase(), robot_data_(robot_data), dt_(dt), nshared_ineq_(nshared_ineq)
{
    if (dt_ <= 1e-9)
        std::cerr << "[HQP_ID_base] WARNING: constructed with dt=" << dt_ << std::endl;

    joint_dof_ = robot_data_->getDof();

    // ---- decision variable sizes ----
    si_index_.qddot_size          = joint_dof_;
    si_index_.torque_size         = joint_dof_;
    si_index_.slack_q_min_size    = joint_dof_;
    si_index_.slack_q_max_size    = joint_dof_;
    si_index_.slack_qdot_min_size = joint_dof_;
    si_index_.slack_qdot_max_size = joint_dof_;
    si_index_.slack_sing_size     = 1;
    si_index_.slack_sel_col_size      = 1;

    const int nx = si_index_.qddot_size +
                   si_index_.torque_size +
                   si_index_.slack_q_min_size +
                   si_index_.slack_q_max_size +
                   si_index_.slack_qdot_min_size +
                   si_index_.slack_qdot_max_size +
                   si_index_.slack_sing_size +
                   si_index_.slack_sel_col_size;  // 6*joint_dof_ + 2

    // ---- constraint sizes ----
    si_index_.con_dyn_size    = joint_dof_;
    si_index_.con_hp_size     = neqc;
    si_index_.con_shared_size = nshared_ineq_;

    const int nbound     = nx;
    const int nineqc     = si_index_.con_shared_size;
    const int total_neqc = si_index_.con_dyn_size + si_index_.con_hp_size;

    QPBase::setQPsize(nx, nbound, nineqc, total_neqc);

    // ---- decision variable starts (sequential layout) ----
    si_index_.qddot_start          = 0;
    si_index_.torque_start         = si_index_.qddot_start          + si_index_.qddot_size;
    si_index_.slack_q_min_start    = si_index_.torque_start         + si_index_.torque_size;
    si_index_.slack_q_max_start    = si_index_.slack_q_min_start    + si_index_.slack_q_min_size;
    si_index_.slack_qdot_min_start = si_index_.slack_q_max_start    + si_index_.slack_q_max_size;
    si_index_.slack_qdot_max_start = si_index_.slack_qdot_min_start + si_index_.slack_qdot_min_size;
    si_index_.slack_sing_start     = si_index_.slack_qdot_max_start + si_index_.slack_qdot_max_size;
    si_index_.slack_sel_col_start      = si_index_.slack_sing_start     + si_index_.slack_sing_size;

    // ---- equality constraint starts ----
    si_index_.con_dyn_start = 0;
    si_index_.con_hp_start  = si_index_.con_dyn_start + si_index_.con_dyn_size;

    // ---- inequality constraint starts ----
    si_index_.con_shared_start = 0;

    // ---- default weights ----
    w_tracking_.setOnes();
    w_vel_damping_.setOnes(joint_dof_);
    w_acc_damping_.setOnes(joint_dof_);
}

void HQP_ID_base::setDesiredTaskAcc(const std::map<std::string, Vector6d>& link_xddot_desired)
{
    link_xddot_desired_ = link_xddot_desired;
}

void HQP_ID_base::setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& qddot_ref)
{
    J_eq_ = J_eq;
    v_eq_ = J_eq * qddot_ref; // Jdot * qdot is canceled out for eqaulity constraint (J_eq * qddot + J_dot_eq * qdot  =  J_eq * qddot_ref + J_dot_eq * qdot)
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

bool HQP_ID_base::getOptJoint(Eigen::Ref<VectorXd> opt_qddot,
                              Eigen::Ref<VectorXd> opt_torque,
                              QP::TimeDuration& time_status)
{
    if (opt_qddot.size() != joint_dof_ || opt_torque.size() != joint_dof_)
    {
        std::cerr << "[HQP_ID_base] size mismatch: opt_qddot=" << opt_qddot.size()
                  << " opt_torque=" << opt_torque.size()
                  << " joint_dof_=" << joint_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    if (si_index_.con_hp_size > 0 && (J_eq_.rows() != si_index_.con_hp_size || v_eq_.size() != si_index_.con_hp_size))
    {
        std::cerr << "[HQP_ID_base] higher-priority constraint data missing or wrong size. "
                  << "con_hp_size=" << si_index_.con_hp_size
                  << " J_eq_.rows()=" << J_eq_.rows()
                  << " v_eq_.size()=" << v_eq_.size() << std::endl;
        opt_qddot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    MatrixXd sol;
    if (!solveQP(sol, time_status))
    {
        std::cerr << "[HQP_ID_base] QP solve failed." << std::endl;
        opt_qddot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    const VectorXd qddot_sol  = sol.block(si_index_.qddot_start,  0, si_index_.qddot_size,  1);
    const VectorXd torque_sol = sol.block(si_index_.torque_start, 0, si_index_.torque_size, 1);

    if (!qddot_sol.allFinite() || !torque_sol.allFinite())
    {
        std::cerr << "[HQP_ID_base] solution is non-finite." << std::endl;
        opt_qddot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return false;
    }

    opt_qddot  = qddot_sol;
    opt_torque = torque_sol;
    return true;
}

void HQP_ID_base::setCost()
{
    P_ds_.setZero(nx_, nx_);
    q_ds_.setZero(nx_);

    const VectorXd qdot = robot_data_->getJointVelocity();

    // Task-space acceleration tracking: Σ_i || J_i*qddot + J̇_i*qdot - ẍ_des_i ||²_Wi
    for (const auto& [link_name, xddot_desired] : link_xddot_desired_)
    {
        const MatrixXd J_i     = robot_data_->getJacobian(link_name);
        const MatrixXd J_i_dot = robot_data_->getJacobianTimeVariation(link_name);

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

        P_ds_.block(si_index_.qddot_start, si_index_.qddot_start, si_index_.qddot_size, si_index_.qddot_size) +=
            2.0 * J_i.transpose() * w.asDiagonal() * J_i;
        q_ds_.segment(si_index_.qddot_start, si_index_.qddot_size) +=
            -2.0 * J_i.transpose() * w.asDiagonal() * (xddot_desired - J_i_dot * qdot);
    }

    // Joint acceleration damping: ||qddot||²_Wacc
    P_ds_.block(si_index_.qddot_start, si_index_.qddot_start, si_index_.qddot_size, si_index_.qddot_size) +=
        2.0 * w_acc_damping_.asDiagonal();

    // Joint velocity damping: ||dt*qddot + qdot||²_Wvel
    P_ds_.block(si_index_.qddot_start, si_index_.qddot_start, si_index_.qddot_size, si_index_.qddot_size) +=
        2.0 * dt_ * dt_ * w_vel_damping_.asDiagonal();
    q_ds_.segment(si_index_.qddot_start, si_index_.qddot_size) +=
        2.0 * dt_ * w_vel_damping_.asDiagonal() * qdot;

    // Linear slack penalty: 100 * Σs
    q_ds_.segment(si_index_.slack_q_min_start,    si_index_.slack_q_min_size)   .setConstant(100.0);
    q_ds_.segment(si_index_.slack_q_max_start,    si_index_.slack_q_max_size)   .setConstant(100.0);
    q_ds_.segment(si_index_.slack_qdot_min_start, si_index_.slack_qdot_min_size).setConstant(100.0);
    q_ds_.segment(si_index_.slack_qdot_max_start, si_index_.slack_qdot_max_size).setConstant(100.0);
    q_ds_(si_index_.slack_sing_start) = 100.0;
    q_ds_(si_index_.slack_sel_col_start)  = 100.0;
}

void HQP_ID_base::setBoundConstraint()
{
    l_bound_ds_.setConstant(nbc_, -OSQP_INFTY);
    u_bound_ds_.setConstant(nbc_,  OSQP_INFTY);

    // torque limits
    const auto torque_lim = robot_data_->getJointEffortLimit();
    l_bound_ds_.segment(si_index_.torque_start, si_index_.torque_size) = torque_lim.first;
    u_bound_ds_.segment(si_index_.torque_start, si_index_.torque_size) = torque_lim.second;

    // slack >= 0
    l_bound_ds_.segment(si_index_.slack_q_min_start,    si_index_.slack_q_min_size)   .setZero();
    l_bound_ds_.segment(si_index_.slack_q_max_start,    si_index_.slack_q_max_size)   .setZero();
    l_bound_ds_.segment(si_index_.slack_qdot_min_start, si_index_.slack_qdot_min_size).setZero();
    l_bound_ds_.segment(si_index_.slack_qdot_max_start, si_index_.slack_qdot_max_size).setZero();
    l_bound_ds_(si_index_.slack_sing_start) = 0.0;
    l_bound_ds_(si_index_.slack_sel_col_start)  = 0.0;
}

void HQP_ID_base::setIneqConstraint()
{
    A_ineq_ds_.setZero(nineqc_, nx_);
    l_ineq_ds_.setConstant(nineqc_, -OSQP_INFTY);
    u_ineq_ds_.setConstant(nineqc_,  OSQP_INFTY);

    if (nshared_ineq_ > 0 && shared_ineq_.A.rows() == nshared_ineq_)
    {
        const int rs = si_index_.con_shared_start;  // row start in A_ineq_ds_

        // qddot columns
        A_ineq_ds_.block(rs, si_index_.qddot_start, nshared_ineq_, si_index_.qddot_size) = shared_ineq_.A;
        l_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.l;
        u_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.u;

        // slack columns — each constraint type has its own slack variable
        A_ineq_ds_.block(rs + shared_ineq_.q_min_start,    si_index_.slack_q_min_start,
                         shared_ineq_.q_min_size,           si_index_.slack_q_min_size)    = MatrixXd::Identity(shared_ineq_.q_min_size,    si_index_.slack_q_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.q_max_start,    si_index_.slack_q_max_start,
                         shared_ineq_.q_max_size,           si_index_.slack_q_max_size)    = MatrixXd::Identity(shared_ineq_.q_max_size,    si_index_.slack_q_max_size);
        A_ineq_ds_.block(rs + shared_ineq_.qdot_min_start, si_index_.slack_qdot_min_start,
                         shared_ineq_.qdot_min_size,        si_index_.slack_qdot_min_size) = MatrixXd::Identity(shared_ineq_.qdot_min_size, si_index_.slack_qdot_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.qdot_max_start, si_index_.slack_qdot_max_start,
                         shared_ineq_.qdot_max_size,        si_index_.slack_qdot_max_size) = MatrixXd::Identity(shared_ineq_.qdot_max_size, si_index_.slack_qdot_max_size);
        A_ineq_ds_(rs + shared_ineq_.sel_col_start,  si_index_.slack_sel_col_start)  = 1.0;
        A_ineq_ds_(rs + shared_ineq_.sing_start, si_index_.slack_sing_start) = 1.0;
    }
}

void HQP_ID_base::setEqConstraint()
{
    A_eq_ds_.setZero(neqc_, nx_);
    b_eq_ds_.setZero(neqc_);

    // Dynamics: M*qddot - I*torque = -nle
    const MatrixXd M   = robot_data_->getMassMatrix();
    const MatrixXd nle = robot_data_->getNonlinearEffects();

    A_eq_ds_.block(si_index_.con_dyn_start, si_index_.qddot_start,  si_index_.con_dyn_size, si_index_.qddot_size)  = M;
    A_eq_ds_.block(si_index_.con_dyn_start, si_index_.torque_start, si_index_.con_dyn_size, si_index_.torque_size) = -MatrixXd::Identity(si_index_.con_dyn_size, si_index_.torque_size);
    b_eq_ds_.segment(si_index_.con_dyn_start, si_index_.con_dyn_size) = -nle;

    // Higher-priority task accelerations: J_eq * qddot = a_eq
    if (si_index_.con_hp_size > 0 && J_eq_.rows() == si_index_.con_hp_size)
    {
        A_eq_ds_.block(si_index_.con_hp_start, si_index_.qddot_start, si_index_.con_hp_size, si_index_.qddot_size) = J_eq_;
        b_eq_ds_.segment(si_index_.con_hp_start, si_index_.con_hp_size) = v_eq_;
    }
}

// =============================================================================
//  HQPID
// =============================================================================

HQPID::HQPID(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt)
: robot_data_(robot_data), dt_(dt)
{
    joint_dof_ = robot_data_->getDof();

    si_index_.con_q_min_size    = joint_dof_;
    si_index_.con_q_max_size    = joint_dof_;
    si_index_.con_qdot_min_size = joint_dof_;
    si_index_.con_qdot_max_size = joint_dof_;
    si_index_.con_sel_col_size      = 1;
    si_index_.con_sing_size     = 1;

    si_index_.con_q_min_start    = 0;
    si_index_.con_q_max_start    = si_index_.con_q_min_start    + si_index_.con_q_min_size;
    si_index_.con_qdot_min_start = si_index_.con_q_max_start    + si_index_.con_q_max_size;
    si_index_.con_qdot_max_start = si_index_.con_qdot_min_start + si_index_.con_qdot_min_size;
    si_index_.con_sel_col_start      = si_index_.con_qdot_max_start + si_index_.con_qdot_max_size;
    si_index_.con_sing_start     = si_index_.con_sel_col_start      + si_index_.con_sel_col_size;

    nshared_ineq_ = si_index_.con_q_min_size + si_index_.con_q_max_size +
                    si_index_.con_qdot_min_size + si_index_.con_qdot_max_size +
                    si_index_.con_sel_col_size + si_index_.con_sing_size;  // 4n + 2

    w_vel_damping_.setOnes(joint_dof_);
    w_acc_damping_.setOnes(joint_dof_);
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
    const int n = joint_dof_;

    SharedIneqDataID data;
    data.A.setZero(nshared_ineq_, n);  // qddot columns only (slack columns added per-level)
    data.l.setConstant(nshared_ineq_, -OSQP_INFTY);
    data.u.setConstant(nshared_ineq_,  OSQP_INFTY);

    // Embed row-range info so HQP_ID_base::setIneqConstraint() can assign slack columns
    data.q_min_start    = si_index_.con_q_min_start;    data.q_min_size    = si_index_.con_q_min_size;
    data.q_max_start    = si_index_.con_q_max_start;    data.q_max_size    = si_index_.con_q_max_size;
    data.qdot_min_start = si_index_.con_qdot_min_start; data.qdot_min_size = si_index_.con_qdot_min_size;
    data.qdot_max_start = si_index_.con_qdot_max_start; data.qdot_max_size = si_index_.con_qdot_max_size;
    data.sel_col_start      = si_index_.con_sel_col_start;      data.sel_col_size      = si_index_.con_sel_col_size;
    data.sing_start     = si_index_.con_sing_start;     data.sing_size     = si_index_.con_sing_size;

    const double alpha = 100.0;

    const VectorXd q    = robot_data_->getJointPosition();
    const VectorXd qdot = robot_data_->getJointVelocity();

    // ---- joint position limits (2nd-order CBF) ----
    const auto     q_lim     = robot_data_->getJointPositionLimit();
    const VectorXd q_min_raw = q_lim.first;
    const VectorXd q_max_raw = q_lim.second;
    const VectorXd q_min = (q_min_raw.array() < 0.0).select(q_min_raw.array() * 0.9, q_min_raw.array() * 1.1).matrix();
    const VectorXd q_max = (q_max_raw.array() > 0.0).select(q_max_raw.array() * 0.9, q_max_raw.array() * 1.1).matrix();

    // qddot >= -(2a)*qdot - a²*(q - q_min)   [slack added in setIneqConstraint]
    data.A.block(si_index_.con_q_min_start, 0, n, n) =  MatrixXd::Identity(n, n);
    data.l.segment(si_index_.con_q_min_start, n) = -(alpha + alpha) * qdot - alpha * alpha * (q - q_min);

    // -qddot >= +(2a)*qdot - a²*(q_max - q)
    data.A.block(si_index_.con_q_max_start, 0, n, n) = -MatrixXd::Identity(n, n);
    data.l.segment(si_index_.con_q_max_start, n) = +(alpha + alpha) * qdot - alpha * alpha * (q_max - q);

    // ---- joint velocity limits (1st-order CBF) ----
    const auto     qdot_lim     = robot_data_->getJointVelocityLimit();
    const VectorXd qdot_min_raw = qdot_lim.first;
    const VectorXd qdot_max_raw = qdot_lim.second;
    const VectorXd qdot_min = (qdot_min_raw.array() < 0.0).select(qdot_min_raw.array() * 0.9, qdot_min_raw.array() * 1.1).matrix();
    const VectorXd qdot_max = (qdot_max_raw.array() > 0.0).select(qdot_max_raw.array() * 0.9, qdot_max_raw.array() * 1.1).matrix();

    // qddot >= -a*(qdot - qdot_min)
    data.A.block(si_index_.con_qdot_min_start, 0, n, n) =  MatrixXd::Identity(n, n);
    data.l.segment(si_index_.con_qdot_min_start, n) = -alpha * (qdot - qdot_min);

    // -qddot >= -a*(qdot_max - qdot)
    data.A.block(si_index_.con_qdot_max_start, 0, n, n) = -MatrixXd::Identity(n, n);
    data.l.segment(si_index_.con_qdot_max_start, n) = -alpha * (qdot_max - qdot);

    // ---- self-collision avoidance CBF (2nd-order) ----
    const MinDistResult min_dist_res = robot_data_->getMinDistance(true, true, false);
    const bool valid_col_grad =
        std::isfinite(min_dist_res.distance)         &&
        min_dist_res.grad.size()     == joint_dof_   &&
        min_dist_res.grad_dot.size() == joint_dof_   &&
        min_dist_res.grad.allFinite()                &&
        min_dist_res.grad_dot.allFinite()            &&
        min_dist_res.grad.squaredNorm() > 1e-12;

    if (valid_col_grad)
    {
        if (!col_grad_initialized_)
        {
            col_grad_filtered_     = min_dist_res.grad;
            col_grad_dot_filtered_ = min_dist_res.grad_dot;
            col_grad_initialized_  = true;
        }
        else
        {
            col_grad_filtered_     = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                                   + col_grad_filter_alpha_ * min_dist_res.grad;
            col_grad_dot_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_dot_filtered_
                                   + col_grad_filter_alpha_ * min_dist_res.grad_dot;
        }

        // ∇d·qddot >= -∇̇d·qdot - 2a*∇d·qdot - a²*(d - eps)
        data.A.row(si_index_.con_sel_col_start) = col_grad_filtered_.transpose();
        data.l(si_index_.con_sel_col_start) =
            -col_grad_dot_filtered_.dot(qdot)
            - (alpha + alpha) * col_grad_filtered_.dot(qdot)
            - alpha * alpha * (min_dist_res.distance - 0.01);
    }
    else
    {
        col_grad_initialized_ = false;
        // con_col row stays trivial
    }

    // ---- singularity CBF (reserved – trivial row) ----
    // const ManipulabilityResult mani = robot_data_->getManipulability(true, true, link_name);
    // data.A.row(si_index_.con_sing_start) = mani.grad.transpose();
    // data.l(si_index_.con_sing_start) = -mani.grad_dot.dot(qdot)
    //     - 2*alpha*mani.grad.dot(qdot) - alpha*alpha*(mani.manipulability - 0.01);

    return data;
}

bool HQPID::getOptJoint(Eigen::Ref<VectorXd> opt_qddot, Eigen::Ref<VectorXd> opt_torque, QP::TimeDuration& time_status)
{
    if (opt_qddot.size() != joint_dof_ || opt_torque.size() != joint_dof_)
    {
        std::cerr << "[HQPID] size mismatch: opt_qddot=" << opt_qddot.size()
                  << " opt_torque=" << opt_torque.size()
                  << " joint_dof_=" << joint_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    const auto& tasks = tasks_;
    if (tasks.empty())
    {
        opt_qddot.setZero();
        opt_torque.setZero();
        time_status.setZero();
        return true;
    }

    // Compute all shared constraints once for this cycle
    const SharedIneqDataID shared = computeSharedConstraints();

    // Hierarchical solve
    time_status.setZero();
    VectorXd qddot_result  = VectorXd::Zero(joint_dof_);
    VectorXd torque_result = VectorXd::Zero(joint_dof_);
    bool all_ok = true;

    MatrixXd J_eq_stacked(0, joint_dof_);

    for (int k = 0; k < static_cast<int>(tasks.size()); k++)
    {
        const std::map<std::string, Vector6d>& level = tasks[k];

        qp_levels_[k]->setDesiredTaskAcc(level);

        if (k > 0) qp_levels_[k]->setHigherPriorityConstraints(J_eq_stacked, qddot_result);

        qp_levels_[k]->setSharedIneqConstraints(shared);
        qp_levels_[k]->setJointVelWeight(w_vel_damping_);
        qp_levels_[k]->setJointAccWeight(w_acc_damping_);

        if (use_uniform_tracking_) qp_levels_[k]->setTrackingWeight(uniform_w_tracking_);
        else                       qp_levels_[k]->setTrackingWeight(link_w_tracking_);

        VectorXd qddot_k(joint_dof_), torque_k(joint_dof_);
        QP::TimeDuration level_time;
        const bool ok = qp_levels_[k]->getOptJoint(qddot_k, torque_k, level_time);

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

        qddot_result  = qddot_k;
        torque_result = torque_k;

        // Append this level's Jacobian rows to the stacked equality constraint matrix
        const int new_rows = static_cast<int>(6 * level.size());
        const int old_rows = static_cast<int>(J_eq_stacked.rows());
        MatrixXd J_eq_new(old_rows + new_rows, joint_dof_);
        if (old_rows > 0) J_eq_new.topRows(old_rows) = J_eq_stacked;
        int row = 0;
        for (const auto& [link, _] : level)
        {
            J_eq_new.block(old_rows + row, 0, 6, joint_dof_) = robot_data_->getJacobian(link);
            row += 6;
        }
        J_eq_stacked = std::move(J_eq_new);
    }

    opt_qddot  = qddot_result;
    opt_torque = torque_result;
    return all_ok;
}

} // namespace Manipulator
} // namespace drc
