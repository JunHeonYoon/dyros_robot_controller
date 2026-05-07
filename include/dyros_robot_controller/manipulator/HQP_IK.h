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
         * @brief Precomputed shared inequality constraint data passed to every HQP level.
         *
         * Computed once per HQP cycle in HQPIK::computeSharedConstraints() and shared
         * across all priority levels.
         *
         * A has (nshared × n) columns covering only the qdot decision variables.
         * The slack columns are added per-level inside HQP_IK_base::setIneqConstraint()
         * using the embedded row-range fields — matching the pattern of HQPID's SharedIneqDataID.
         * Inactive rows have l = -OSQP_INFTY, u = +OSQP_INFTY, A.row = 0.
         *
         * Row layout (nshared = 2*n + 2):
         *
         *   ```
         *   [0,   n)  : q_min CBF  — qdot + s >= -alpha*(q - q_min)
         *   [n,  2n)  : q_max CBF  — -qdot + s >= -alpha*(q_max - q)
         *   [2n]      : self-collision CBF
         *   [2n+1]    : singularity CBF  (reserved – trivial until enabled)
         *   ```
         */
        struct SharedIneqData
        {
            MatrixXd A;  // (nshared x n): qdot columns only
            VectorXd l;  // (nshared): lower bounds
            VectorXd u;  // (nshared): upper bounds
            // Row ranges so HQP_IK_base::setIneqConstraint() can add the correct slack columns
            int q_min_start,   q_min_size;
            int q_max_start,   q_max_size;
            int sel_col_start, sel_col_size;
            int sing_start,    sing_size;
        };

        /**
         * @brief QP subproblem for one priority level of the Hierarchical QP IK solver.
         *
         * Inherits from QPBase. Each instance handles exactly one priority level:
         *   - Cost      : task-space velocity tracking + joint acceleration damping
         *   - Equality  : preservation of all higher-priority task velocities (strict)
         *   - Inequality: shared precomputed constraints (joint limits CBF, collision, singularity)
         *   - Bound     : joint velocity limits
         *
         *
         *   ```
         *   Decision variables: [qdot(n), slack_q_min(n), slack_q_max(n), slack_sing(1), slack_sel_col(1)]
         *     → nx = 2n + 2
         *   ```
         */
        class HQP_IK_base : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 * @param neqc (int) Total equality constraint rows from all higher-priority levels
                 *                   (= sum of 6 * tasks_per_level for levels 0..k-1).
                 * @param nshared_ineq (int) Number of shared inequality constraint rows supplied by HQPIK.
                 */
                HQP_IK_base(std::shared_ptr<Manipulator::RobotData> robot_data,
                             const double dt,
                             const int neqc,
                             const int nshared_ineq);

                /**
                 * @brief Set the desired task space velocity for the link.
                 * @param link_xdot_desired (std::map<std::string, Vector6d>) Desired task space velocity (6D twist) per links.
                 */
                void setDesiredTaskVel(const std::map<std::string, Vector6d>& link_xdot_desired);

                /**
                 * @brief Set the accumulated equality constraints from all higher-priority levels.
                 *        Must be called before getOptJointVel() for every level with neqc > 0.
                 * @param J_eq (Eigen::MatrixXd) Stacked Jacobian rows (neqc x n).
                 * @param qdot_ref (Eigen::VectorXd) Optimal joint velocity from the most-recently solved level (n).
                 *                  The achieved task velocities are computed internally as J_eq * qdot_ref.
                 */
                void setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& qdot_ref);

                /**
                 * @brief Supply shared precomputed constraints (joint limits CBF, collision, singularity, …).
                 * @param shared (SharedIneqData) Shared data whose row count must match nshared_ineq at construction.
                 */
                void setSharedIneqConstraints(const SharedIneqData& shared);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space velocity tracking for all the links.
                 */
                void setTrackingWeight(const Vector6d& w_tracking);

                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
                 */
                void setTrackingWeight(const std::map<std::string, Vector6d>& link_w_tracking);

                /**
                 * @brief Set joint velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set joint acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping toward qdot_now; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set previous optimal joint velocity used for the acceleration damping term.
                 * @param qdot_prev (Eigen::VectorXd) Previous joint velocity of size joint_dof_.
                 */
                void setPrevJointVel(const Eigen::Ref<const VectorXd>& qdot_prev) { qdot_prev_ = qdot_prev; }

                /**
                 * @brief Get the optimal joint velocity by solving QP.
                 * @param opt_qdot (Eigen::VectorXd) Optimal joint velocity.
                 * @param time_status (TimeDuration) Time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJointVel(Eigen::Ref<VectorXd> opt_qdot, QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qdot_start,          qdot_size;
                    int slack_q_min_start,   slack_q_min_size;
                    int slack_q_max_start,   slack_q_max_size;
                    int slack_sing_start,    slack_sing_size;
                    int slack_sel_col_start, slack_sel_col_size;
                    // inequality constraints
                    int con_shared_start,    con_shared_size;
                } si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                           // control time step size
                int joint_dof_;                                       // Number of joints in the manipulator
                int nshared_ineq_;

                std::map<std::string, Vector6d> link_xdot_desired_;  // Desired task velocity per links

                MatrixXd J_eq_;    // accumulated higher-priority Jacobians  (neqc_ x n)
                VectorXd v_eq_;    // accumulated achieved task velocities    (neqc_)

                SharedIneqData shared_ineq_;

                bool use_uniform_tracking_ = true;
                Vector6d w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for joint velocity damping; || qdot ||
                VectorXd w_acc_damping_;  // weight for joint acceleration damping; || (qdot - qdot_now) / dt ||
                VectorXd qdot_prev_;

                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *   ```
                 *      min      Σ_i || x_i_dot_des - J_i*q_dot ||_W1_i^2
                 *    [qdot,s]     + || q_dot ||_W2^2
                 *                 + || (q_dot - q_dot_now) / dt ||_W3^2
                 *                 + 100*s
                 *
                 *   =>    min     1/2 [ qdot ]^T * [ 2*Σ(J_i.T*W1_i*J_i) + 2*W2 + 2/dt^2*W3  0 ] * [ qdot ]
                 *       [qdot,s]      [   s  ]     [                  0                       0 ]   [   s  ]
                 *
                 *                   + [ -2*Σ(J_i.T*W1_i*x_i_dot_des) - 2/dt^2*W3*q_dot_now ].T * [ qdot ]
                 *                     [                          100                       ]     [  s   ]
                 *   ```
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint which limits manipulator joint velocities and keeps all slack variables non-negative.
                 *
                 *   ```
                 *   subject to [ qdot_min ] <= [ qdot ] <= [ qdot_max ]
                 *              [    0     ]    [   s  ]    [   inf    ]
                 *   ```
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit joint angles and avoid self collision by 1st-order CBF.
                 *        Constraints are precomputed in HQPIK::computeSharedConstraints() and applied here.
                 *
                 * 1st-order CBF condition with slack: hdot(x) >= -a*h(x) - s
                 *
                 *  1. Manipulator joint angle limit:
                 *     h_min(q) = q - q_min >= 0  ->  hdot_min(q) = qdot
                 *     h_max(q) = q_max - q >= 0  ->  hdot_max(q) = -qdot
                 *
                 *   ```
                 *   => subject to [  I  I ] * [ qdot ] >= [ -a*(q - q_min) ]
                 *                 [ -I  I ]   [  s   ]    [ -a*(q_max - q) ]
                 *   ```
                 *
                 *  2. Singularity avoidance:
                 *     A CBF row is allocated for this term, but it is currently inactive in the implementation.
                 *
                 *  3. Self collision avoidance (active only when a valid distance gradient is available):
                 *     h_selcol(q) = self_dist(q) - eps_selcol_min >= 0
                 *     hdot_selcol = ∇_q self_dist^T * qdot
                 *
                 *   ```
                 *   => subject to [ ∇_(q) self_dist.T  I ] * [ qdot ] >= [ -a*(self_dist - eps_selcol_min) ]
                 *                                            [  s   ]
                 *   ```
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which preserve all higher-priority task velocities.
                 *
                 *   ```
                 *   subject to J_eq * qdot = v_eq  (v_eq = J_eq * qdot_ref, achieved velocities of higher-priority tasks)
                 *
                 *   => subject to [ J_eq  0 ] * [ qdot ] = [ v_eq ]
                 *                               [  s   ]
                 *   ```
                 */
                void setEqConstraint() override;
        };

        /**
         * @brief Hierarchical QP Inverse Kinematics solver.
         *
         * Accepts a priority-ordered task hierarchy (std::vector<std::map<std::string, Vector6d>>).
         * For each priority level it:
         *   1. Solves a QP minimising the level's task-tracking error.
         *   2. Passes the achieved task velocity of every solved level as a strict equality
         *      constraint to all lower-priority levels, so higher-priority tasks are preserved.
         *
         * All inequality constraints (joint limits, collision, singularity) are computed once per
         * cycle via computeSharedConstraints() and supplied to all HQP_IK_base instances.
         */
        class HQPIK
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                HQPIK(std::shared_ptr<Manipulator::RobotData> robot_data, const double dt);

                /**
                 * @brief Set task tracking weights only.
                 * @param w_tracking (Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 */
                void setTrackingWeight(const Vector6d& w_tracking)
                {
                    uniform_w_tracking_   = w_tracking;
                    use_uniform_tracking_ = true;
                }

                /**
                 * @brief Set task tracking weights only.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
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
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping toward qdot_now; its size must same as dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping toward qdot_now; its size must same as dof.
                 */
                void setWeight(const Vector6d w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task space velocity tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for joint acceleration damping toward qdot_now; its size must same as dof.
                 */
                void setWeight(const std::map<std::string, Vector6d> link_w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(link_w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }

                /**
                 * @brief Set the desired task space velocity for the link.
                 * @param tasks (std::vector<std::map<std::string, Vector6d>>) Priority-ordered task hierarchy
                 *   (index 0 = highest priority). Each element is one priority level: a map from
                 *   link_name to desired 6D task velocity [vx,vy,vz,wx,wy,wz].
                 *
                 *   ```cpp
                 *   tasks[0] = { {"link7", xdot_ee} };    // level 0 (highest): end-effector velocity
                 *   tasks[1] = { {"link3", xdot_elbow} }; // level 1 (lower):   elbow velocity
                 *   ```
                 */
                void setDesiredTaskVel(const std::vector<std::map<std::string, Vector6d>>& tasks);

                /**
                 * @brief Get the optimal joint velocity by solving QP hierarchically.
                 * @param opt_qdot (Eigen::VectorXd) Optimal joint velocity.
                 * @param time_status (TimeDuration) Aggregated timing output (sum of all level timings).
                 * @return (bool) True if every level was solved successfully.
                 */
                bool getOptJointVel(Eigen::Ref<VectorXd> opt_qdot, QP::TimeDuration&    time_status);

            private:
                /**
                 * @brief Struct to hold the row indices and sizes within SharedIneqData.
                 *        To add a new constraint type: add start/size fields here,
                 *        initialize them in the constructor, and implement the rows
                 *        in computeSharedConstraints().
                 */
                struct SharedIneqIndex
                {
                    int con_q_min_start,    con_q_min_size;    // joint position lower-limit CBF
                    int con_q_max_start,    con_q_max_size;    // joint position upper-limit CBF
                    int con_sel_col_start,  con_sel_col_size;  // self-collision CBF
                    int con_sing_start,     con_sing_size;     // singularity CBF (reserved)
                } si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                           // control time step size
                int    joint_dof_;                                    // Number of joints in the manipulator
                int    nshared_ineq_;  // total shared inequality rows = sum of si_index_ sizes

                std::vector<std::unique_ptr<HQP_IK_base>> qp_levels_;
                std::vector<int> task_structure_;  // task count per level; change detection
                std::vector<std::map<std::string, Vector6d>> tasks_;  // stored by setDesiredTaskVel()

                bool use_uniform_tracking_ = true;
                Vector6d uniform_w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for joint velocity damping; || qdot ||
                VectorXd w_acc_damping_;  // weight for joint acceleration damping; || (qdot - qdot_now) / dt ||
                VectorXd qdot_prev_;

                // self-collision CBF gradient exponential filter
                // smooths discontinuous jumps when the closest collision pair changes
                VectorXd col_grad_filtered_;
                bool     col_grad_initialized_  = false;
                double   col_grad_filter_alpha_ = 0.2;  // filter coefficient (0~1): larger = faster tracking, less smoothing

                /**
                 * @brief Compute SharedIneqData once per cycle.
                 *        Returns exactly nshared_ineq_ rows; inactive rows are trivial.
                 */
                SharedIneqData computeSharedConstraints();

                /**
                 * @brief (Re)create all HQP_IK_base level instances to match the given hierarchy.
                 *        Called automatically when task_structure_ changes.
                 * @param tasks (std::vector<std::map<std::string, Vector6d>>) Same hierarchy as setDesiredTaskVel().
                 */
                void initializeLevels(const std::vector<std::map<std::string, Vector6d>>& tasks);
        };

    } // namespace Manipulator
} // namespace drc
