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

#include "dyros_robot_controller/mobile_manipulator/HQP_IK.h"
#include <cmath>
#include <iostream>

namespace drc
{
namespace MobileManipulator
{

// =============================================================================
//  HQP_IK_base
// =============================================================================

HQP_IK_base::HQP_IK_base(std::shared_ptr<MobileManipulator::RobotData> robot_data,
                           const double dt,
                           const int neqc,
                           const int nshared_ineq)
: QP::QPBase(), robot_data_(robot_data), dt_(dt), nshared_ineq_(nshared_ineq)
{
    if (dt_ <= 1e-9)
        std::cerr << "[HQP_IK_base] WARNING: constructed with dt=" << dt_ << std::endl;

    actuator_dof_ = robot_data_->getActuatorDof();
    mani_dof_     = robot_data_->getManipulatorDof();
    mobi_dof_     = robot_data_->getMobileDof();

    si_index_.eta_size              = actuator_dof_;
    si_index_.slack_q_mani_min_size = mani_dof_;
    si_index_.slack_q_mani_max_size = mani_dof_;
    si_index_.slack_sing_size       = 1;
    si_index_.slack_sel_col_size    = 1;
    si_index_.con_shared_size       = nshared_ineq_;

    const int nx = si_index_.eta_size +
                   si_index_.slack_q_mani_min_size +
                   si_index_.slack_q_mani_max_size +
                   si_index_.slack_sing_size +
                   si_index_.slack_sel_col_size;  // actuator_dof_ + 2*mani_dof_ + 2
    const int nbound = nx;
    const int nineqc = si_index_.con_shared_size;

    QPBase::setQPsize(nx, nbound, nineqc, neqc);

    si_index_.eta_start              = 0;
    si_index_.slack_q_mani_min_start = si_index_.eta_start              + si_index_.eta_size;
    si_index_.slack_q_mani_max_start = si_index_.slack_q_mani_min_start + si_index_.slack_q_mani_min_size;
    si_index_.slack_sing_start       = si_index_.slack_q_mani_max_start + si_index_.slack_q_mani_max_size;
    si_index_.slack_sel_col_start    = si_index_.slack_sing_start       + si_index_.slack_sing_size;
    si_index_.con_shared_start       = 0;

    w_tracking_.setOnes();
    w_vel_damping_.setZero(actuator_dof_);
    w_acc_damping_.setOnes(actuator_dof_);
    eta_prev_.setZero(actuator_dof_);
}

void HQP_IK_base::setDesiredTaskVel(const std::map<std::string, Vector6d>& link_xdot_desired)
{
    link_xdot_desired_ = link_xdot_desired;
}

void HQP_IK_base::setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& eta_ref)
{
    J_eq_ = J_eq;
    v_eq_ = J_eq * eta_ref;
}

void HQP_IK_base::setSharedIneqConstraints(const SharedIneqData& shared)
{
    shared_ineq_ = shared;
}

void HQP_IK_base::setTrackingWeight(const Vector6d& w)
{
    w_tracking_           = w;
    use_uniform_tracking_ = true;
}

void HQP_IK_base::setTrackingWeight(const std::map<std::string, Vector6d>& link_w)
{
    link_w_tracking_      = link_w;
    use_uniform_tracking_ = false;
}

bool HQP_IK_base::getOptJointVel(Eigen::Ref<VectorXd> opt_eta, QP::TimeDuration& time_status)
{
    if (opt_eta.size() != actuator_dof_)
    {
        std::cerr << "[HQP_IK_base] size mismatch: opt_eta=" << opt_eta.size()
                  << " actuator_dof_=" << actuator_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    if (neqc_ > 0 && (J_eq_.rows() != neqc_ || v_eq_.size() != neqc_))
    {
        std::cerr << "[HQP_IK_base] equality constraint data missing or wrong size. "
                  << "neqc_=" << neqc_
                  << " J_eq_.rows()=" << J_eq_.rows()
                  << " v_eq_.size()=" << v_eq_.size() << std::endl;
        opt_eta.setZero();
        time_status.setZero();
        return false;
    }

    MatrixXd sol;
    if (!solveQP(sol, time_status))
    {
        std::cerr << "[HQP_IK_base] QP solve failed." << std::endl;
        opt_eta.setZero();
        time_status.setZero();
        return false;
    }

    const VectorXd eta_sol = sol.block(si_index_.eta_start, 0, si_index_.eta_size, 1);
    if (!eta_sol.allFinite())
    {
        std::cerr << "[HQP_IK_base] solution is non-finite." << std::endl;
        opt_eta.setZero();
        time_status.setZero();
        return false;
    }

    opt_eta = eta_sol;
    return true;
}

void HQP_IK_base::setCost()
{
    P_ds_.setZero(nx_, nx_);
    q_ds_.setZero(nx_);

    // Task-space velocity tracking: Σ_i ||J_i_tilda*eta - xdot_i||²_Wi
    for (const auto& [link_name, xdot_desired] : link_xdot_desired_)
    {
        const MatrixXd J_i_tilda = robot_data_->getJacobianActuated(link_name);

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

        P_ds_.block(si_index_.eta_start, si_index_.eta_start, si_index_.eta_size, si_index_.eta_size) +=
            2.0 * J_i_tilda.transpose() * w.asDiagonal() * J_i_tilda;
        q_ds_.segment(si_index_.eta_start, si_index_.eta_size) +=
            -2.0 * J_i_tilda.transpose() * w.asDiagonal() * xdot_desired;
    }

    // for slack
    q_ds_.segment(si_index_.slack_q_mani_min_start, si_index_.slack_q_mani_min_size).setConstant(1000.0);
    q_ds_.segment(si_index_.slack_q_mani_max_start, si_index_.slack_q_mani_max_size).setConstant(1000.0);
    q_ds_(si_index_.slack_sing_start)    = 1000.0;
    q_ds_(si_index_.slack_sel_col_start) = 1000.0;

    // Actuator velocity damping: ||eta||²_Wvel
    P_ds_.block(si_index_.eta_start, si_index_.eta_start, si_index_.eta_size, si_index_.eta_size) +=
        2.0 * w_vel_damping_.asDiagonal();

    // Actuator acceleration damping: ||(eta - eta_prev)/dt||²_Wacc
    if (dt_ > 1e-9)
    {
        const double dt_sq_inv = 1.0 / (dt_ * dt_);
        P_ds_.block(si_index_.eta_start, si_index_.eta_start, si_index_.eta_size, si_index_.eta_size) +=
            2.0 * dt_sq_inv * w_acc_damping_.asDiagonal();
        q_ds_.segment(si_index_.eta_start, si_index_.eta_size) +=
            -2.0 * dt_sq_inv * w_acc_damping_.asDiagonal() * eta_prev_;
    }
    else
    {
        std::cerr << "[HQP_IK_base::setCost] dt_ near zero, skipping acc damping." << std::endl;
    }
}

void HQP_IK_base::setBoundConstraint()
{
    l_bound_ds_.setConstant(nbc_, -OSQP_INFTY);
    u_bound_ds_.setConstant(nbc_,  OSQP_INFTY);

    const int mani_start = robot_data_->getJointIndex().mani_start;

    // Manipulator joint velocity limit (mani part of eta only)
    const auto     qdot_lim         = robot_data_->getJointVelocityLimit();
    const VectorXd qdot_mani_min_raw = qdot_lim.first.segment(mani_start,  mani_dof_);
    const VectorXd qdot_mani_max_raw = qdot_lim.second.segment(mani_start, mani_dof_);
    const VectorXd qdot_mani_min =
        (qdot_mani_min_raw.array() < 0.0).select(qdot_mani_min_raw.array() * 0.9, qdot_mani_min_raw.array() * 1.1).matrix();
    const VectorXd qdot_mani_max =
        (qdot_mani_max_raw.array() > 0.0).select(qdot_mani_max_raw.array() * 0.9, qdot_mani_max_raw.array() * 1.1).matrix();

    l_bound_ds_.segment(si_index_.eta_start + mani_start, mani_dof_) = qdot_mani_min;
    u_bound_ds_.segment(si_index_.eta_start + mani_start, mani_dof_) = qdot_mani_max;

    // for slack
    l_bound_ds_.segment(si_index_.slack_q_mani_min_start, si_index_.slack_q_mani_min_size).setZero();
    l_bound_ds_.segment(si_index_.slack_q_mani_max_start, si_index_.slack_q_mani_max_size).setZero();
    l_bound_ds_(si_index_.slack_sing_start)    = 0.0;
    l_bound_ds_(si_index_.slack_sel_col_start) = 0.0;
}

void HQP_IK_base::setIneqConstraint()
{
    A_ineq_ds_.setZero(nineqc_, nx_);
    l_ineq_ds_.setConstant(nineqc_, -OSQP_INFTY);
    u_ineq_ds_.setConstant(nineqc_,  OSQP_INFTY);

    if (nshared_ineq_ > 0 && shared_ineq_.A.rows() == nshared_ineq_)
    {
        const int rs = si_index_.con_shared_start;  // row start in A_ineq_ds_

        // eta columns
        A_ineq_ds_.block(rs, si_index_.eta_start, nshared_ineq_, si_index_.eta_size) = shared_ineq_.A;
        l_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.l;
        u_ineq_ds_.segment(rs, nshared_ineq_) = shared_ineq_.u;

        // slack columns — each constraint type has its own slack variable
        A_ineq_ds_.block(rs + shared_ineq_.q_mani_min_start, si_index_.slack_q_mani_min_start,
                         shared_ineq_.q_mani_min_size,        si_index_.slack_q_mani_min_size) = MatrixXd::Identity(shared_ineq_.q_mani_min_size, si_index_.slack_q_mani_min_size);
        A_ineq_ds_.block(rs + shared_ineq_.q_mani_max_start, si_index_.slack_q_mani_max_start,
                         shared_ineq_.q_mani_max_size,        si_index_.slack_q_mani_max_size) = MatrixXd::Identity(shared_ineq_.q_mani_max_size, si_index_.slack_q_mani_max_size);
        A_ineq_ds_(rs + shared_ineq_.sel_col_start, si_index_.slack_sel_col_start) = 1.0;
        A_ineq_ds_(rs + shared_ineq_.sing_start,    si_index_.slack_sing_start)    = 1.0;
        // base_vel rows have no slack — hard bounded constraint
    }
}

void HQP_IK_base::setEqConstraint()
{
    A_eq_ds_.setZero(neqc_, nx_);
    b_eq_ds_.setZero(neqc_);

    if (neqc_ > 0 && J_eq_.rows() == neqc_)
    {
        A_eq_ds_.block(0, si_index_.eta_start, neqc_, si_index_.eta_size) = J_eq_;
        b_eq_ds_ = v_eq_;
    }
}

// =============================================================================
//  HQPIK
// =============================================================================

HQPIK::HQPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt)
: robot_data_(robot_data), dt_(dt)
{
    actuator_dof_ = robot_data_->getActuatorDof();
    mani_dof_     = robot_data_->getManipulatorDof();
    mobi_dof_     = robot_data_->getMobileDof();

    si_index_.con_q_mani_min_size = mani_dof_;
    si_index_.con_q_mani_max_size = mani_dof_;
    si_index_.con_sel_col_size    = 1;
    si_index_.con_sing_size       = 1;
    si_index_.con_base_vel_size   = 3;

    si_index_.con_q_mani_min_start = 0;
    si_index_.con_q_mani_max_start = si_index_.con_q_mani_min_start + si_index_.con_q_mani_min_size;
    si_index_.con_sel_col_start    = si_index_.con_q_mani_max_start + si_index_.con_q_mani_max_size;
    si_index_.con_sing_start       = si_index_.con_sel_col_start    + si_index_.con_sel_col_size;
    si_index_.con_base_vel_start   = si_index_.con_sing_start       + si_index_.con_sing_size;

    nshared_ineq_ = si_index_.con_q_mani_min_size +
                    si_index_.con_q_mani_max_size +
                    si_index_.con_sel_col_size +
                    si_index_.con_sing_size +
                    si_index_.con_base_vel_size;

    eta_prev_.setZero(actuator_dof_);
    w_vel_damping_.setZero(actuator_dof_);
    w_acc_damping_.setOnes(actuator_dof_);
    uniform_w_tracking_.setOnes();
}

void HQPIK::setDesiredTaskVel(const std::vector<std::map<std::string, Vector6d>>& tasks)
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

void HQPIK::initializeLevels(const std::vector<std::map<std::string, Vector6d>>& tasks)
{
    qp_levels_.clear();
    int accumulated_eq_rows = 0;
    for (const auto& level : tasks)
    {
        qp_levels_.push_back(std::make_unique<HQP_IK_base>(robot_data_, dt_, accumulated_eq_rows, nshared_ineq_));
        accumulated_eq_rows += static_cast<int>(6 * level.size());
    }
}

SharedIneqData HQPIK::computeSharedConstraints()
{
    const int mobi_start = robot_data_->getActuatorIndex().mobi_start;
    const int mani_start_act = robot_data_->getActuatorIndex().mani_start;
    const int mani_start_jnt = robot_data_->getJointIndex().mani_start;

    SharedIneqData data;
    data.A.setZero(nshared_ineq_, actuator_dof_);
    data.l.setConstant(nshared_ineq_, -OSQP_INFTY);
    data.u.setConstant(nshared_ineq_,  OSQP_INFTY);

    // Embed row-range info so HQP_IK_base::setIneqConstraint() can assign slack columns
    data.q_mani_min_start = si_index_.con_q_mani_min_start;  data.q_mani_min_size = si_index_.con_q_mani_min_size;
    data.q_mani_max_start = si_index_.con_q_mani_max_start;  data.q_mani_max_size = si_index_.con_q_mani_max_size;
    data.sel_col_start    = si_index_.con_sel_col_start;     data.sel_col_size    = si_index_.con_sel_col_size;
    data.sing_start       = si_index_.con_sing_start;        data.sing_size       = si_index_.con_sing_size;
    data.base_vel_start   = si_index_.con_base_vel_start;    data.base_vel_size   = si_index_.con_base_vel_size;

    const double alpha = 100.0;

    // Manipulator joint position lower-limit CBF: qdot_mani >= -alpha*(q_mani - q_mani_min)
    const auto     q_lim         = robot_data_->getJointPositionLimit();
    const VectorXd q_mani_min_raw = q_lim.first.segment(mani_start_jnt,  mani_dof_);
    const VectorXd q_mani_max_raw = q_lim.second.segment(mani_start_jnt, mani_dof_);
    const VectorXd q_mani_min =
        (q_mani_min_raw.array() < 0.0).select(q_mani_min_raw.array() * 0.9, q_mani_min_raw.array() * 1.1).matrix();
    const VectorXd q_mani_max =
        (q_mani_max_raw.array() > 0.0).select(q_mani_max_raw.array() * 0.9, q_mani_max_raw.array() * 1.1).matrix();
    const VectorXd q_mani = robot_data_->getJointPosition().segment(mani_start_jnt, mani_dof_);

    data.A.block(si_index_.con_q_mani_min_start, mani_start_act, mani_dof_, mani_dof_) =  MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_q_mani_min_start, mani_dof_)                          = -alpha * (q_mani - q_mani_min);

    // Manipulator joint position upper-limit CBF: -qdot_mani >= -alpha*(q_mani_max - q_mani)
    data.A.block(si_index_.con_q_mani_max_start, mani_start_act, mani_dof_, mani_dof_) = -MatrixXd::Identity(mani_dof_, mani_dof_);
    data.l.segment(si_index_.con_q_mani_max_start, mani_dof_)                          = -alpha * (q_mani_max - q_mani);

    // Self-collision avoidance CBF: grad_mani^T * qdot_mani >= -alpha*(dist - eps)
    const Manipulator::MinDistResult min_dist_res = robot_data_->getMinDistance(true, false, false);
    const bool valid_col_grad =
        std::isfinite(min_dist_res.distance)                  &&
        min_dist_res.grad.size() >= mani_start_jnt + mani_dof_ &&
        min_dist_res.grad.allFinite()                           &&
        min_dist_res.grad.segment(mani_start_jnt, mani_dof_).squaredNorm() > 1e-12;

    if (valid_col_grad)
    {
        const VectorXd col_grad = min_dist_res.grad.segment(mani_start_jnt, mani_dof_);
        if (!col_grad_initialized_)
        {
            col_grad_filtered_    = col_grad;
            col_grad_initialized_ = true;
        }
        else
        {
            col_grad_filtered_ = (1.0 - col_grad_filter_alpha_) * col_grad_filtered_
                               + col_grad_filter_alpha_          * col_grad;
        }
        data.A.block(si_index_.con_sel_col_start, mani_start_act, 1, mani_dof_) = col_grad_filtered_.transpose();
        data.l(si_index_.con_sel_col_start)                                      = -alpha * (min_dist_res.distance - 0.01);
        data.u(si_index_.con_sel_col_start)                                      = OSQP_INFTY;
    }
    else
    {
        col_grad_initialized_ = false;
        // si_index_.con_sel_col_start row stays trivial
    }

    // Singularity avoidance CBF (reserved – activate by uncommenting below)
    // const ManipulabilityResult mani = robot_data_->getManipulability(true, false, link_name);
    // if (mani.grad.size() >= mani_start_jnt + mani_dof_ && mani.grad.allFinite())
    // {
    //     const VectorXd sing_grad = mani.grad.segment(mani_start_jnt, mani_dof_);
    //     const double eps_sing = 0.01;
    //     data.A.block(si_index_.con_sing_start, mani_start_act, 1, mani_dof_) = sing_grad.transpose();
    //     data.l(si_index_.con_sing_start) = -alpha * (mani.manipulability - eps_sing);
    //     data.u(si_index_.con_sing_start) = OSQP_INFTY;
    // }

    // Mobile base velocity limit (hard — no slack): J_mobile * eta_mobi in [-vel_limit, +vel_limit]
    const auto&    param    = robot_data_->getKineParam();
    const MatrixXd J_mobile = robot_data_->getMobileFKJacobian();

    data.A.block(si_index_.con_base_vel_start, mobi_start, si_index_.con_base_vel_size, mobi_dof_) = J_mobile;
    Vector3d vel_limit;
    vel_limit << param.max_lin_speed, param.max_lin_speed, param.max_ang_speed;
    data.l.segment(si_index_.con_base_vel_start, si_index_.con_base_vel_size) = -vel_limit;
    data.u.segment(si_index_.con_base_vel_start, si_index_.con_base_vel_size) =  vel_limit;

    return data;
}

bool HQPIK::getOptJointVel(Eigen::Ref<VectorXd> opt_eta, QP::TimeDuration& time_status)
{
    SuhanBenchmark total_timer;

    if (opt_eta.size() != actuator_dof_)
    {
        std::cerr << "[HQPIK] size mismatch: opt_eta=" << opt_eta.size()
                  << " actuator_dof_=" << actuator_dof_ << std::endl;
        time_status.setZero();
        return false;
    }

    const auto& tasks = tasks_;
    if (tasks.empty())
    {
        opt_eta.setZero();
        time_status.setZero();
        return true;
    }

    // Compute all shared constraints once for this cycle
    SuhanBenchmark shared_timer;
    const SharedIneqData shared = computeSharedConstraints();
    const double shared_time = shared_timer.elapsed();

    // Hierarchical solve
    time_status.setZero();
    time_status.set_ineq       += shared_time;
    time_status.set_constraint += shared_time;
    time_status.set_qp         += shared_time;
    VectorXd eta_result = eta_prev_;  // fallback if first level fails
    bool all_ok = true;

    MatrixXd J_eq_stacked(0, actuator_dof_);

    for (int k = 0; k < static_cast<int>(tasks.size()); k++)
    {
        const std::map<std::string, Vector6d>& level = tasks[k];

        qp_levels_[k]->setDesiredTaskVel(level);

        if (k > 0) qp_levels_[k]->setHigherPriorityConstraints(J_eq_stacked, eta_result);

        qp_levels_[k]->setSharedIneqConstraints(shared);
        qp_levels_[k]->setJointVelWeight(w_vel_damping_);
        qp_levels_[k]->setJointAccWeight(w_acc_damping_);
        qp_levels_[k]->setPrevJointVel(eta_prev_);

        if (use_uniform_tracking_) qp_levels_[k]->setTrackingWeight(uniform_w_tracking_);
        else                       qp_levels_[k]->setTrackingWeight(link_w_tracking_);

        VectorXd eta_k(actuator_dof_);
        QP::TimeDuration level_time;
        const bool ok = qp_levels_[k]->getOptJointVel(eta_k, level_time);

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
            std::cerr << "[HQPIK] level " << k << " solve failed, stopping hierarchy." << std::endl;
            all_ok = false;
            break;
        }

        eta_result = eta_k;

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

    eta_prev_ = eta_result;
    opt_eta   = eta_result;
    time_status.total = total_timer.elapsed();
    return all_ok;
}

} // namespace MobileManipulator
} // namespace drc
