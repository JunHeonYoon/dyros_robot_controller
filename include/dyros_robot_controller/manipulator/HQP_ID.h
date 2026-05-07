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
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"
#include <map>
#include <vector>
#include <memory>
#include <string>

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Precomputed shared inequality constraint data passed to every HQPID level.
         *
         * Computed once per HQP cycle in HQPID::computeSharedConstraints() and shared
         * across all priority levels.
         *
         * A has (nshared × n) columns covering only the qddot decision variables.
         * The slack columns are added per-level inside HQP_ID_base::setIneqConstraint()
         * using the embedded row-range fields — matching the pattern of HQPIK's SharedIneqData.
         * Inactive rows have l = -OSQP_INFTY, u = +OSQP_INFTY, A.row = 0.
         *
         * Row layout (nshared = 4*n + 2):
         *
         *   ```
         *   [0,    n)  : q_min CBF    — 2nd-order:  qddot + s >= -(2a)*qdot - a^2*(q - q_min)
         *   [n,   2n)  : q_max CBF    — 2nd-order: -qddot + s >= +(2a)*qdot - a^2*(q_max - q)
         *   [2n,  3n)  : qdot_min CBF — 1st-order:  qddot + s >= -a*(qdot - qdot_min)
         *   [3n,  4n)  : qdot_max CBF — 1st-order: -qddot + s >= -a*(qdot_max - qdot)
         *   [4n]       : self-collision CBF — 2nd-order
         *   [4n+1]     : singularity CBF (reserved – trivial until enabled)
         *   ```
         */
        struct SharedIneqDataID
        {
            MatrixXd A;  // (nshared × n): qddot columns only
            VectorXd l;  // (nshared): lower bounds
            VectorXd u;  // (nshared): upper bounds
            // Row ranges so HQP_ID_base::setIneqConstraint() can add the correct slack columns
            int q_min_start,    q_min_size;
            int q_max_start,    q_max_size;
            int qdot_min_start, qdot_min_size;
            int qdot_max_start, qdot_max_size;
            int sel_col_start,  sel_col_size;
            int sing_start,     sing_size;
        };

        /**
         * @brief QP subproblem for one priority level of the Hierarchical QP ID solver.
         *
         * Inherits from QPBase. Each instance handles exactly one priority level:
         *
         *
         *   ```
         *   Decision variables: [qddot(n), torque(n), slack_q_min(n), slack_q_max(n),
         *                        slack_qdot_min(n), slack_qdot_max(n), slack_sing(1), slack_sel_col(1)]
         *     → nx = 6n + 2
         *
         *   Cost:
         *     Σ||J_i*qddot + J_i_dot*qdot − xddot_des_i||²_Wi
         *     + ||qddot||²_Wacc
         *     + ||dt*qddot + qdot||²_Wvel
         *     + 100·Σs   (linear slack penalty)
         *
         *   Equality:
         *     Dynamics:        M·qddot − torque = −nle
         *     Higher-priority: J_eq·qddot = v_eq   (v_eq = J_eq·qddot_ref computed inside setter)
         *
         *   Inequality: shared precomputed constraints supplied by HQPID (qddot columns + slack columns)
         *   Bound:      torque ∈ [τ_min, τ_max],  slack ≥ 0,  qddot unbounded
         *   ```
         */
        class HQP_ID_base : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 * @param neqc (int) Number of equality constraint rows from all higher-priority levels
                 *                   (= Σ 6 * tasks_per_level for levels 0..k-1).
                 * @param nshared_ineq (int) Number of shared inequality constraint rows supplied by HQPID.
                 */
                HQP_ID_base(std::shared_ptr<Manipulator::RobotData> robot_data,
                             const double dt,
                             const int neqc,
                             const int nshared_ineq);

                /**
                 * @brief Set the desired task space acceleration for each link.
                 * @param link_xddot_desired (std::map<std::string, Vector6d>) Desired task space acceleration (6D twist) per links.
                 */
                void setDesiredTaskAcc(const std::map<std::string, Vector6d>& link_xddot_desired);

                /**
                 * @brief Set the accumulated equality constraints from all higher-priority levels.
                 *        Must be called before getOptJoint() for every level with neqc > 0.
                 * @param J_eq (Eigen::MatrixXd) Stacked Jacobian rows (neqc × n).
                 * @param qddot_ref (Eigen::VectorXd) Optimal joint acceleration from the most-recently solved level (n).
                 *                   The achieved task accelerations are computed as J_eq * qddot_ref.
                 */
                void setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& qddot_ref);

                /**
                 * @brief Supply precomputed shared inequality constraints.
                 * @param shared (SharedIneqDataID) Shared data whose row count must match nshared_ineq at construction.
                 */
                void setSharedIneqConstraints(const SharedIneqDataID& shared);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space acceleration tracking for all the links.
                 */
                void setTrackingWeight(const Vector6d& w_tracking);

                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 */
                void setTrackingWeight(const std::map<std::string, Vector6d>& link_w_tracking);

                /**
                 * @brief Set joint velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set joint acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP.
                 * @param opt_qddot (Eigen::VectorXd) Optimal joint accelerations.
                 * @param opt_torque (Eigen::VectorXd) Optimal joint torques.
                 * @param time_status (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<VectorXd> opt_qddot,
                                 Eigen::Ref<VectorXd> opt_torque,
                                 QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qddot_start,           qddot_size;
                    int torque_start,          torque_size;
                    int slack_q_min_start,     slack_q_min_size;
                    int slack_q_max_start,     slack_q_max_size;
                    int slack_qdot_min_start,  slack_qdot_min_size;
                    int slack_qdot_max_start,  slack_qdot_max_size;
                    int slack_sing_start,      slack_sing_size;
                    int slack_sel_col_start,   slack_sel_col_size;
                    // equality constraints
                    int con_dyn_start,    con_dyn_size;   // dynamics
                    int con_hp_start,     con_hp_size;    // higher-priority task accel
                    // inequality constraints
                    int con_shared_start, con_shared_size;
                } si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                           // control time step size
                int joint_dof_;                                       // Number of joints in the manipulator
                int nshared_ineq_;

                std::map<std::string, Vector6d> link_xddot_desired_;  // Desired task acceleration per links

                MatrixXd J_eq_;  // stacked higher-priority Jacobians (neqc × n)
                VectorXd v_eq_;  // achieved task accels = J_eq_ * qddot_ref (neqc)

                SharedIneqDataID shared_ineq_;

                bool use_uniform_tracking_ = true;
                Vector6d w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for joint velocity damping;     || q_ddot*dt + qdot ||
                VectorXd w_acc_damping_;  // weight for joint acceleration damping; || q_ddot ||

                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *   ```
                 *          min      Σ_i || x_i_ddot_des - J_i*qddot - J_i_dot*qdot ||_W1_i^2
                 *   [qddot,tau,s]     + || qddot ||_W2^2
                 *                     + || dt*qddot + qdot ||_W3^2
                 *                     + 100*s
                 *
                 *   =>    min       1/2 [ qddot ]^T * [ 2*Σ(J_i.T*W1_i*J_i) + 2*W2 + 2*dt^2*W3  0  0 ] * [ qddot ]
                 *   [qddot,tau,s]       [  tau  ]     [                       0                 0  0 ]   [  tau  ]
                 *                       [   s   ]     [                       0                 0  0 ]   [   s   ]
                 *
                 *                     + [ -2*Σ(J_i.T*W1_i*(x_i_ddot_des - J_i_dot*qdot)) + 2*dt*W3*qdot ].T * [ qddot ]
                 *                       [                             0                                 ]     [  tau  ]
                 *                       [                           100                                 ]     [   s   ]
                 *   ```
                 */
                void setCost() override;
                /**
                 * @brief Set variable bounds.
                 *
                 *   ```
                 *   subject to [ -inf    ] <= [ qddot  ] <= [ inf     ]
                 *              [ tau_min ]    [ torque ]    [ tau_max ]
                 *              [   0     ]    [    s   ]    [ inf     ]
                 *   ```
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set inequality constraints for joint limits and self collision.
                 *        Constraints are precomputed in HQPID::computeSharedConstraints() and applied here.
                 *
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 2nd-order CBF condition with slack: hddot(x) >= -2*a*hdot(x) - a^2*h(x) - s
                 *
                 *  1. (2nd-order CBF) Manipulator joint angle limit:
                 *     h_p_min(q) = q - q_min >= 0  ->  hdot = qdot    ->  hddot = qddot
                 *     h_p_max(q) = q_max - q >= 0  ->  hdot = -qdot   ->  hddot = -qddot
                 *
                 *   ```
                 *   => subject to [  I  0  I ] * [ qddot  ] >= [ -2*a*qdot - a^2*(q - q_min) ]
                 *                 [ -I  0  I ]   [ torque ]    [  2*a*qdot - a^2*(q_max - q) ]
                 *                                [    s   ]
                 *   ```
                 *
                 *  2. (1st-order CBF) Manipulator joint velocity limit:
                 *     h_v_min(qdot) = qdot - qdot_min >= 0  ->  hdot = qddot
                 *     h_v_max(qdot) = qdot_max - qdot >= 0  ->  hdot = -qddot
                 *
                 *   ```
                 *   => subject to [  I  0  I ] * [ qddot  ] >= [ -a*(qdot - qdot_min) ]
                 *                 [ -I  0  I ]   [ torque ]    [ -a*(qdot_max - qdot) ]
                 *                                [    s   ]
                 *   ```
                 *
                 *  3. Singularity avoidance:
                 *     A CBF row is allocated for this term, but it is currently inactive in the implementation.
                 *
                 *  4. (2nd-order CBF, active only when a valid distance gradient is available)
                 *     Self collision avoidance:
                 *     h_selcol(q) = self_dist(q) - eps_selcol_min >= 0
                 *     hdot_selcol = ∇_q self_dist^T * qdot
                 *     hddot_selcol = (∇_q self_dist)_dot^T * qdot + ∇_q self_dist^T * qddot
                 *
                 *   ```
                 *   => subject to [ ∇_(q) self_dist.T  0  I ] * [ qddot  ] >= [ -(∇_(q) self_dist.T)dot*qdot - 2*a*∇_(q) self_dist.T*qdot - a^2*(self_dist - eps_selcol_min) ]
                 *                                               [ torque ]
                 *                                               [    s   ]
                 *   ```
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which enforce dynamics and preserve all higher-priority task accelerations.
                 *
                 *   ```
                 *   subject to M * qddot + nle = torque   (dynamics)
                 *              J_eq * qddot = v_eq        (higher-priority tasks)
                 *
                 *   => subject to [   M    -I  0 ] * [ qddot  ] = [ -nle ]
                 *                 [ J_eq    0  0 ]   [ torque ]   [ v_eq ]
                 *                                    [    s   ]
                 *   ```
                 */
                void setEqConstraint() override;
        };

        /**
         * @brief Hierarchical QP Inverse Dynamics solver.
         *
         * Accepts a priority-ordered task hierarchy (std::vector<std::map<std::string, Vector6d>>).
         * For each priority level it:
         *   1. Solves a QP minimising the level's task-tracking error in acceleration space.
         *   2. Passes the achieved task accelerations of every solved level as strict equality
         *      constraints to all lower-priority levels, preserving higher-priority tasks.
         *
         * All inequality constraints (joint limits, velocity limits, collision, singularity)
         * are computed once per cycle via computeSharedConstraints() and supplied to all levels.
         */
        class HQPID
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                HQPID(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 */
                void setTrackingWeight(const Vector6d& w_tracking)
                {
                    uniform_w_tracking_   = w_tracking;
                    use_uniform_tracking_ = true;
                }

                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 */
                void setTrackingWeight(const std::map<std::string, Vector6d>& link_w_tracking)
                {
                    link_w_tracking_      = link_w_tracking;
                    use_uniform_tracking_ = false;
                }

                /**
                 * @brief Set joint velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set joint acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setWeight(const Vector6d w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(link_w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }

                /**
                 * @brief Set the desired task space acceleration for each link.
                 * @param tasks (std::vector<std::map<std::string, Vector6d>>) Priority-ordered task hierarchy
                 *   (index 0 = highest priority). Each element is one priority level: a map from
                 *   link_name to desired 6D task acceleration [ax,ay,az,αx,αy,αz].
                 *
                 *   ```cpp
                 *   tasks[0] = { {"link7", xddot_ee} };    // level 0 (highest): end-effector acceleration
                 *   tasks[1] = { {"link3", xddot_elbow} }; // level 1 (lower):   elbow acceleration
                 *   ```
                 */
                void setDesiredTaskAcc(const std::vector<std::map<std::string, Vector6d>>& tasks);

                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP hierarchically.
                 * @param opt_qddot (Eigen::VectorXd) Optimal joint accelerations.
                 * @param opt_torque (Eigen::VectorXd) Optimal joint torques.
                 * @param time_status (TimeDuration) Aggregated timing output (sum of all level timings).
                 * @return (bool) True if every level was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<VectorXd> opt_qddot, Eigen::Ref<VectorXd> opt_torque, QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the row indices and sizes within SharedIneqDataID.
                 *        To add a new constraint type: add start/size fields here,
                 *        initialize them in the constructor, and implement the rows
                 *        in computeSharedConstraints().
                 */
                struct SharedIneqIndex
                {
                    int con_q_min_start,    con_q_min_size;    // joint position lower-limit CBF (2nd order)
                    int con_q_max_start,    con_q_max_size;    // joint position upper-limit CBF (2nd order)
                    int con_qdot_min_start, con_qdot_min_size; // joint velocity lower-limit CBF (1st order)
                    int con_qdot_max_start, con_qdot_max_size; // joint velocity upper-limit CBF (1st order)
                    int con_sel_col_start,  con_sel_col_size;  // self-collision CBF (2nd order)
                    int con_sing_start,     con_sing_size;     // singularity CBF (reserved)
                } si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                           // control time step size
                int    joint_dof_;                                    // Number of joints in the manipulator
                int    nshared_ineq_;  // total shared inequality rows = sum of si_index_ sizes

                std::vector<std::unique_ptr<HQP_ID_base>> qp_levels_;
                std::vector<int> task_structure_;  // task count per level; change detection
                std::vector<std::map<std::string, Vector6d>> tasks_;  // stored by setDesiredTaskAcc()

                bool use_uniform_tracking_ = true;
                Vector6d uniform_w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for joint velocity damping;     || q_ddot*dt + qdot ||
                VectorXd w_acc_damping_;  // weight for joint acceleration damping; || q_ddot ||

                // self-collision CBF gradient exponential filter
                // smooths discontinuous jumps when the closest collision pair changes
                VectorXd col_grad_filtered_;
                VectorXd col_grad_dot_filtered_;
                bool     col_grad_initialized_  = false;
                double   col_grad_filter_alpha_ = 0.2;  // filter coefficient (0~1): larger = faster tracking, less smoothing

                /**
                 * @brief Compute SharedIneqDataID once per cycle.
                 *        Returns exactly nshared_ineq_ rows; inactive rows are trivial.
                 */
                SharedIneqDataID computeSharedConstraints();

                /**
                 * @brief (Re)create all HQP_ID_base level instances to match the given hierarchy.
                 *        Called automatically when task_structure_ changes.
                 * @param tasks (std::vector<std::map<std::string, Vector6d>>) Same hierarchy as setDesiredTaskAcc().
                 */
                void initializeLevels(const std::vector<std::map<std::string, Vector6d>>& tasks);
        };

    } // namespace Manipulator
} // namespace drc
