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
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"
#include <map>
#include <vector>
#include <memory>
#include <string>

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Precomputed shared inequality constraint data passed to every HQPID level.
         *
         * Computed once per HQP cycle in HQPID::computeSharedConstraints() and shared
         * across all priority levels.
         *
         * A has (nshared × actuator_dof) columns covering only the eta_dot decision variables.
         * The slack columns are added per-level inside HQP_ID_base::setIneqConstraint()
         * using the embedded row-range fields. Inactive rows have l = -OSQP_INFTY, u = +OSQP_INFTY, A.row = 0.
         *
         * Row layout (nshared = 4*mani_dof + 14):
         *
         *   ```
         *   [0,           mani_dof)  : q_mani_min CBF  — 2nd-order: qddot_mani + s >= -(2a)*qdot_mani - a^2*(q_mani - q_mani_min)
         *   [mani_dof,  2*mani_dof)  : q_mani_max CBF  — 2nd-order: -qddot_mani + s >= +(2a)*qdot_mani - a^2*(q_mani_max - q_mani)
         *   [2*mani_dof, 3*mani_dof) : qdot_mani_min CBF — 1st-order: qddot_mani + s >= -a*(qdot_mani - qdot_mani_min)
         *   [3*mani_dof, 4*mani_dof) : qdot_mani_max CBF — 1st-order: -qddot_mani + s >= -a*(qdot_mani_max - qdot_mani)
         *   [4*mani_dof]             : self-collision CBF — 2nd-order
         *   [4*mani_dof+1]           : singularity CBF (reserved – trivial until enabled)
         *   [4*mani_dof+2, 4*mani_dof+5) : base_vel_min CBF — 1st-order: J_mob*eta_dot + s >= -a*(v_base - v_min)
         *   [4*mani_dof+5, 4*mani_dof+8) : base_vel_max CBF — 1st-order: -J_mob*eta_dot + s >= -a*(v_max - v_base)
         *   [4*mani_dof+8, 4*mani_dof+11): base_acc_min     — hard limit: J_mob*eta_dot + s >= -acc_limit
         *   [4*mani_dof+11,4*mani_dof+14): base_acc_max     — hard limit: -J_mob*eta_dot + s >= -acc_limit
         *   ```
         */
        struct SharedIneqDataID
        {
            MatrixXd A;  // (nshared × actuator_dof): eta_dot columns only
            VectorXd l;  // (nshared): lower bounds
            VectorXd u;  // (nshared): upper bounds
            // Row ranges so HQP_ID_base::setIneqConstraint() can add the correct slack columns
            int q_mani_min_start,    q_mani_min_size;
            int q_mani_max_start,    q_mani_max_size;
            int qdot_mani_min_start, qdot_mani_min_size;
            int qdot_mani_max_start, qdot_mani_max_size;
            int sel_col_start,       sel_col_size;
            int sing_start,          sing_size;
            int base_vel_min_start,  base_vel_min_size;
            int base_vel_max_start,  base_vel_max_size;
            int base_acc_min_start,  base_acc_min_size;
            int base_acc_max_start,  base_acc_max_size;
        };

        /**
         * @brief QP subproblem for one priority level of the Hierarchical QP ID solver (MobileManipulator).
         *
         * Inherits from QPBase. Each instance handles exactly one priority level.
         *
         *   ```
         *   Decision variables: [eta_dot(a), torque(a),
         *                        slack_q_mani_min(m), slack_q_mani_max(m),
         *                        slack_qdot_mani_min(m), slack_qdot_mani_max(m),
         *                        slack_sing(1), slack_sel_col(1),
         *                        slack_base_vel_min(3), slack_base_vel_max(3),
         *                        slack_base_acc_min(3), slack_base_acc_max(3)]
         *     → nx = 2a + 4m + 14   (a = actuator_dof, m = mani_dof)
         *
         *   Cost:
         *     Σ||J_i_tilda*eta_dot + J_i_tilda_dot*eta − xddot_des_i||²_Wi
         *     + ||eta_dot||²_Wacc
         *     + ||dt*eta_dot + eta||²_Wvel
         *     + 1000·Σs   (linear slack penalty)
         *
         *   Equality:
         *     Dynamics:        M_tilda·eta_dot − torque = −nle_tilda
         *     Higher-priority: J_eq_tilda·eta_dot = v_eq  (v_eq = J_eq_tilda·etadot_ref)
         *
         *   Inequality: shared precomputed constraints supplied by HQPID (eta_dot columns + slack columns)
         *   Bound:      slack ≥ 0,  eta_dot and torque unbounded
         *   ```
         */
        class HQP_ID_base : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 * @param neqc (int) Number of equality constraint rows from all higher-priority levels
                 *                   (= Σ 6 * tasks_per_level for levels 0..k-1).
                 * @param nshared_ineq (int) Number of shared inequality constraint rows supplied by HQPID.
                 */
                HQP_ID_base(std::shared_ptr<MobileManipulator::RobotData> robot_data,
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
                 * @param J_eq (Eigen::MatrixXd) Stacked actuated Jacobian rows (neqc × actuator_dof).
                 * @param etadot_ref (Eigen::VectorXd) Optimal actuator acceleration from the most-recently solved level.
                 *                    The achieved task accelerations are computed as J_eq * etadot_ref.
                 */
                void setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& etadot_ref);

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
                 * @brief Set actuator-space velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set actuator-space acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Get the optimal actuator acceleration and torque by solving QP.
                 * @param opt_etadot (Eigen::VectorXd) Optimal actuator accelerations.
                 * @param opt_torque (Eigen::VectorXd) Optimal actuator torques.
                 * @param time_status (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<VectorXd> opt_etadot,
                                 Eigen::Ref<VectorXd> opt_torque,
                                 QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int eta_dot_start,             eta_dot_size;
                    int torque_start,              torque_size;
                    int slack_q_mani_min_start,    slack_q_mani_min_size;
                    int slack_q_mani_max_start,    slack_q_mani_max_size;
                    int slack_qdot_mani_min_start, slack_qdot_mani_min_size;
                    int slack_qdot_mani_max_start, slack_qdot_mani_max_size;
                    int slack_sing_start,          slack_sing_size;
                    int slack_sel_col_start,       slack_sel_col_size;
                    int slack_base_vel_min_start,  slack_base_vel_min_size;
                    int slack_base_vel_max_start,  slack_base_vel_max_size;
                    int slack_base_acc_min_start,  slack_base_acc_min_size;
                    int slack_base_acc_max_start,  slack_base_acc_max_size;
                    // equality constraints
                    int con_dyn_start,    con_dyn_size;   // dynamics
                    int con_hp_start,     con_hp_size;    // higher-priority task accel
                    // inequality constraints
                    int con_shared_start, con_shared_size;
                } si_index_;

                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int actuator_dof_;                                           // Number of actuators in the mobile manipulator
                int mani_dof_;                                               // Number of joints in the manipulator
                int mobi_dof_;                                               // Number of degrees of freedom in the mobile base
                int nshared_ineq_;

                std::map<std::string, Vector6d> link_xddot_desired_;  // Desired task acceleration per links

                MatrixXd J_eq_;  // stacked higher-priority actuated Jacobians (neqc × actuator_dof)
                VectorXd v_eq_;  // achieved task accels = J_eq_ * etadot_ref (neqc)

                SharedIneqDataID shared_ineq_;

                bool use_uniform_tracking_ = true;
                Vector6d w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for actuator velocity damping; || eta_dot*dt + eta ||
                VectorXd w_acc_damping_;  // weight for actuator acceleration damping; || eta_dot ||

                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *   ```
                 *          min      Σ_i || x_i_ddot_des - J_i_tilda*eta_dot - J_i_tilda_dot*eta ||_W1_i^2
                 *   [eta_dot,tau,s]   + || eta_dot ||_W2^2
                 *                     + || dt*eta_dot + eta ||_W3^2
                 *                     + 1000*s
                 *
                 *   =>    min        1/2 [ eta_dot ]^T * [ 2*Σ(J_i_tilda.T*W1_i*J_i_tilda) + 2*W2 + 2*dt^2*W3  0  0 ] * [ eta_dot ]
                 *   [eta_dot,tau,s]      [   tau   ]     [                               0                     0  0 ]   [   tau   ]
                 *                        [    s    ]     [                               0                     0  0 ]   [    s    ]
                 *
                 *                      + [ -2*Σ(J_i_tilda.T*W1_i*(x_i_ddot_des - J_i_tilda_dot*eta)) + 2*dt*W3*eta ].T * [ eta_dot ]
                 *                        [                                  0                                      ]     [   tau   ]
                 *                        [                                 1000                                    ]     [    s    ]
                 *   ```
                 */
                void setCost() override;
                /**
                 * @brief Set variable bounds — keep all slack variables non-negative; eta_dot and torque are unbounded.
                 *
                 *   ```
                 *   subject to [ -inf ] <= [ eta_dot ] <= [ inf ]
                 *              [ -inf ]    [  torque ]    [ inf ]
                 *              [   0  ]    [    s    ]    [ inf ]
                 *   ```
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set inequality constraints for joint limits, singularity, self collision, and base motion limits.
                 *        Constraints are precomputed in HQPID::computeSharedConstraints() and applied here.
                 *
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 * 2nd-order CBF condition with slack: hddot(x) >= -2*a*hdot(x) - a^2*h(x) - s
                 *
                 *  1. (2nd-order CBF) Manipulator joint angle limit:
                 *     h_p_min(q_mani) = q_mani - q_mani_min >= 0  ->  hdot = qdot_mani   ->  hddot = qddot_mani
                 *     h_p_max(q_mani) = q_mani_max - q_mani >= 0  ->  hdot = -qdot_mani  ->  hddot = -qddot_mani
                 *
                 *   ```
                 *   => subject to [  I_mani  0  I ] * [ eta_dot ] >= [ -2*a*qdot_mani - a^2*(q_mani - q_mani_min) ]
                 *                 [ -I_mani  0  I ]   [  torque ]    [  2*a*qdot_mani - a^2*(q_mani_max - q_mani) ]
                 *                                 [    s    ]
                 *   ```
                 *
                 *  2. (1st-order CBF) Manipulator joint velocity limit:
                 *     h_v_min(qdot_mani) = qdot_mani - qdot_mani_min >= 0  ->  hdot = qddot_mani
                 *     h_v_max(qdot_mani) = qdot_mani_max - qdot_mani >= 0  ->  hdot = -qddot_mani
                 *
                 *   ```
                 *   => subject to [  I_mani  0  I ] * [ eta_dot ] >= [ -a*(qdot_mani - qdot_mani_min) ]
                 *                 [ -I_mani  0  I ]   [  torque ]    [ -a*(qdot_mani_max - qdot_mani) ]
                 *                                 [    s    ]
                 *   ```
                 *
                 *  3. Singularity avoidance:
                 *     A CBF row is allocated for this term, but it is currently inactive in the implementation.
                 *
                 *  4. (2nd-order CBF) Self collision avoidance (active only when a valid distance gradient is available):
                 *     h_selcol(q_mani) = self_dist(q_mani) - eps_selcol_min >= 0
                 *     hdot_selcol = ∇_(q_mani) self_dist^T * qdot_mani
                 *     hddot_selcol = (∇_(q_mani) self_dist.T)dot * qdot_mani + ∇_(q_mani) self_dist.T * qddot_mani
                 *
                 *   ```
                 *   => subject to [ ∇_(q_mani) self_dist.T  0  I ] * [ eta_dot ] >= [ -(∇_(q_mani) self_dist.T)dot*qdot_mani - 2*a*∇_(q_mani) self_dist.T*qdot_mani - a^2*(self_dist - eps_selcol_min) ]
                 *                                                    [  torque ]
                 *                                                    [    s    ]
                 *   ```
                 *
                 *  5. (1st-order CBF) Mobile base velocity limit:
                 *     h_vel(v_base) = (v_base - v_min, v_max - v_base) >= 0  ->  hdot = ±J_mob * eta_dot
                 *
                 *   ```
                 *   => subject to [  J_mob  0  I ] * [ eta_dot ] >= [ -a*(v_base - v_min)  ]
                 *                 [ -J_mob  0  I ]   [  torque ]    [ -a*(v_max  - v_base) ]
                 *                                    [    s    ]
                 *   ```
                 *
                 *  6. Mobile base acceleration limit (soft hard limit):
                 *
                 *   ```
                 *   => subject to [  J_mob  0  I ] * [ eta_dot ] >= [ -acc_limit ]
                 *                 [ -J_mob  0  I ]   [  torque ]    [ -acc_limit ]
                 *                                    [    s    ]
                 *   ```
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which enforce dynamics and preserve all higher-priority task accelerations.
                 *
                 *   ```
                 *   subject to M_tilda * eta_dot + nle_tilda = torque   (dynamics)
                 *              J_eq_tilda * eta_dot = v_eq              (higher-priority tasks)
                 *
                 *   => subject to [   M_tilda    -I  0 ] * [ eta_dot ] = [ -nle_tilda ]
                 *                 [ J_eq_tilda    0  0 ]   [  torque ]   [    v_eq    ]
                 *                                          [    s    ]
                 *   ```
                 */
                void setEqConstraint() override;
        };

        /**
         * @brief Hierarchical QP Inverse Dynamics solver for mobile manipulators.
         *
         * Accepts a priority-ordered task hierarchy (std::vector<std::map<std::string, Vector6d>>).
         * For each priority level it:
         *   1. Solves a QP minimising the level's task-tracking error in acceleration space.
         *   2. Passes the achieved task accelerations of every solved level as strict equality
         *      constraints to all lower-priority levels, preserving higher-priority tasks.
         *
         * All inequality constraints (joint limits, velocity limits, collision, singularity,
         * base velocity/acceleration limits) are computed once per cycle via
         * computeSharedConstraints() and supplied to all levels.
         */
        class HQPID
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                HQPID(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);

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
                 * @brief Set actuator-space velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set actuator-space acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set the weight vectors for the cost terms.
                 * @param w_tracking (Vector6d) Weight for task space acceleration tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 */
                void setWeight(const Vector6d& w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }

                /**
                 * @brief Set the weight vectors for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space acceleration tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping; its size must same as actuator_dof.
                 */
                void setWeight(const std::map<std::string, Vector6d>& link_w_tracking,
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
                 * @brief Get the optimal actuator acceleration and torque by solving QP hierarchically.
                 * @param opt_etadot (Eigen::VectorXd) Optimal actuator accelerations.
                 * @param opt_torque (Eigen::VectorXd) Optimal actuator torques.
                 * @param time_status (TimeDuration) Aggregated timing output (sum of all level timings).
                 * @return (bool) True if every level was solved successfully.
                 */
                bool getOptJoint(Eigen::Ref<VectorXd> opt_etadot, Eigen::Ref<VectorXd> opt_torque, QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the row indices and sizes within SharedIneqDataID.
                 *        To add a new constraint type: add start/size fields here,
                 *        initialize them in the constructor, and implement the rows
                 *        in computeSharedConstraints().
                 */
                struct SharedIneqIndex
                {
                    int con_q_mani_min_start,    con_q_mani_min_size;    // joint position lower-limit CBF (2nd order)
                    int con_q_mani_max_start,    con_q_mani_max_size;    // joint position upper-limit CBF (2nd order)
                    int con_qdot_mani_min_start, con_qdot_mani_min_size; // joint velocity lower-limit CBF (1st order)
                    int con_qdot_mani_max_start, con_qdot_mani_max_size; // joint velocity upper-limit CBF (1st order)
                    int con_sel_col_start,       con_sel_col_size;       // self-collision CBF (2nd order)
                    int con_sing_start,          con_sing_size;          // singularity CBF (reserved)
                    int con_base_vel_min_start,  con_base_vel_min_size;  // mobile base velocity lower CBF (1st order)
                    int con_base_vel_max_start,  con_base_vel_max_size;  // mobile base velocity upper CBF (1st order)
                    int con_base_acc_min_start,  con_base_acc_min_size;  // mobile base acceleration lower limit
                    int con_base_acc_max_start,  con_base_acc_max_size;  // mobile base acceleration upper limit
                } si_index_;

                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int    actuator_dof_;                                        // Number of actuators in the mobile manipulator
                int    mani_dof_;                                            // Number of joints in the manipulator
                int    mobi_dof_;                                            // Number of degrees of freedom in the mobile base
                int    nshared_ineq_;  // total shared inequality rows = sum of si_index_ sizes

                std::vector<std::unique_ptr<HQP_ID_base>> qp_levels_;
                std::vector<int> task_structure_;  // task count per level; change detection
                std::vector<std::map<std::string, Vector6d>> tasks_;  // stored by setDesiredTaskAcc()

                bool use_uniform_tracking_ = true;
                Vector6d uniform_w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for actuator velocity damping; || eta_dot*dt + eta ||
                VectorXd w_acc_damping_;  // weight for actuator acceleration damping; || eta_dot ||

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

    } // namespace MobileManipulator
} // namespace drc
