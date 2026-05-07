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
         * @brief Precomputed shared inequality constraint data passed to every HQP level.
         *
         * Computed once per HQP cycle in HQPIK::computeSharedConstraints() and shared
         * across all priority levels.
         *
         * A has (nshared × actuator_dof) columns covering only the eta decision variables.
         * The slack columns are added per-level inside HQP_IK_base::setIneqConstraint()
         * using the embedded row-range fields. Inactive rows have l = -OSQP_INFTY, u = +OSQP_INFTY, A.row = 0.
         * The base_vel rows have no slack (hard constraint, bounded on both sides).
         *
         * Row layout (nshared = 2*mani_dof + 2 + 3):
         *
         *   ```
         *   [0,         mani_dof)  : q_mani_min CBF — qdot_mani + s >= -alpha*(q_mani - q_mani_min)
         *   [mani_dof, 2*mani_dof) : q_mani_max CBF — -qdot_mani + s >= -alpha*(q_mani_max - q_mani)
         *   [2*mani_dof]           : self-collision CBF
         *   [2*mani_dof+1]         : singularity CBF (reserved – trivial until enabled)
         *   [2*mani_dof+2, 2*mani_dof+5) : mobile base velocity limit (no slack)
         *   ```
         */
        struct SharedIneqData
        {
            MatrixXd A;  // (nshared x actuator_dof): eta columns only
            VectorXd l;  // (nshared): lower bounds
            VectorXd u;  // (nshared): upper bounds
            // Row ranges so HQP_IK_base::setIneqConstraint() can add the correct slack columns
            int q_mani_min_start, q_mani_min_size;
            int q_mani_max_start, q_mani_max_size;
            int sel_col_start,    sel_col_size;
            int sing_start,       sing_size;
            int base_vel_start,   base_vel_size;  // no slack — hard velocity limit
        };

        /**
         * @brief QP subproblem for one priority level of the Hierarchical QP IK solver for mobile manipulators.
         *
         * Inherits from QPBase. Each instance handles exactly one priority level:
         *   - Cost      : task-space velocity tracking + actuator acceleration damping
         *   - Equality  : preservation of all higher-priority task velocities (strict)
         *   - Inequality: shared precomputed constraints (joint limits CBF, collision, singularity, base velocity)
         *   - Bound     : manipulator joint velocity limits, slack >= 0
         *
         *
         *   ```
         *   Decision variables: [eta(actuator_dof), slack_q_mani_min(mani_dof), slack_q_mani_max(mani_dof),
         *                        slack_sing(1), slack_sel_col(1)]
         *     → nx = actuator_dof + 2*mani_dof + 2
         *   ```
         */
        class HQP_IK_base : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 * @param neqc (int) Total equality constraint rows from all higher-priority levels
                 *                   (= sum of 6 * tasks_per_level for levels 0..k-1).
                 * @param nshared_ineq (int) Number of shared inequality constraint rows supplied by HQPIK.
                 */
                HQP_IK_base(std::shared_ptr<MobileManipulator::RobotData> robot_data,
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
                 * @param J_eq (Eigen::MatrixXd) Stacked Jacobian rows (neqc x actuator_dof).
                 * @param eta_ref (Eigen::VectorXd) Optimal actuator velocity from the most-recently solved level (actuator_dof).
                 *                 The achieved task velocities are computed internally as J_eq * eta_ref.
                 */
                void setHigherPriorityConstraints(const MatrixXd& J_eq, const VectorXd& eta_ref);

                /**
                 * @brief Supply shared precomputed constraints (joint limits CBF, collision, singularity, base velocity).
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
                 * @brief Set actuator velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set actuator acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping toward eta_now; its size must same as actuator_dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set previous optimal actuator velocity used for the acceleration damping term.
                 * @param eta_prev (Eigen::VectorXd) Previous actuator velocity of size actuator_dof.
                 */
                void setPrevJointVel(const Eigen::Ref<const VectorXd>& eta_prev) { eta_prev_ = eta_prev; }

                /**
                 * @brief Get the optimal actuator velocity by solving QP.
                 * @param opt_eta (Eigen::VectorXd) Optimal actuator velocity.
                 * @param time_status (TimeDuration) Time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJointVel(Eigen::Ref<VectorXd> opt_eta, QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int eta_start,              eta_size;
                    int slack_q_mani_min_start, slack_q_mani_min_size;
                    int slack_q_mani_max_start, slack_q_mani_max_size;
                    int slack_sing_start,        slack_sing_size;
                    int slack_sel_col_start,    slack_sel_col_size;
                    // inequality constraints
                    int con_shared_start,       con_shared_size;
                } si_index_;

                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int actuator_dof_;                                           // Number of actuators in the mobile manipulator
                int mani_dof_;                                               // Number of joints in the manipulator
                int mobi_dof_;                                               // Number of degrees of freedom in the mobile base
                int nshared_ineq_;

                std::map<std::string, Vector6d> link_xdot_desired_;  // Desired task velocity per links

                MatrixXd J_eq_;    // accumulated higher-priority Jacobians  (neqc_ x actuator_dof)
                VectorXd v_eq_;    // accumulated achieved task velocities    (neqc_)

                SharedIneqData shared_ineq_;

                bool use_uniform_tracking_ = true;
                Vector6d w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for actuator velocity damping; || eta ||
                VectorXd w_acc_damping_;  // weight for actuator acceleration damping; || (eta - eta_now) / dt ||
                VectorXd eta_prev_;

                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables (s) to increase feasibility of QP.
                 *
                 *   ```
                 *      min     Σ_i || x_i_dot_des - J_i_tilda*eta ||_W1_i^2
                 *    [eta,s]     + || (eta - eta_now) / dt ||_W3^2
                 *                + 1000*s
                 *
                 *   =>    min     1/2 [ eta ]^T * [ 2*Σ(J_i_tilda.T*W1_i*J_i_tilda) + 2/dt^2*W3  0 ] * [ eta ]
                 *       [eta,s]       [  s  ]     [                  0                           0 ]   [  s  ]
                 *
                 *                   + [ -2*Σ(J_i_tilda.T*W1_i*x_i_dot_des) - 2/dt^2*W3*eta_now ].T * [ eta ]
                 *                     [                             1000                       ]     [  s  ]
                 *   ```
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint which limits manipulator joint velocities and keeps all slack variables non-negative.
                 *
                 *   ```
                 *   subject to [ qdot_mani_min ] <= [ eta ] <= [ qdot_mani_max ]  (manipulator part only)
                 *              [       0       ]    [  s  ]    [      inf      ]
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
                 *     h_min(q_mani) = q_mani - q_mani_min >= 0  ->  hdot_min = qdot_mani
                 *     h_max(q_mani) = q_mani_max - q_mani >= 0  ->  hdot_max = -qdot_mani
                 *
                 *   ```
                 *   => subject to [  I_mani  I ] * [ eta ] >= [ -a*(q_mani - q_mani_min) ]
                 *                 [ -I_mani  I ]   [  s  ]    [ -a*(q_mani_max - q_mani) ]
                 *   ```
                 *
                 *  2. Singularity avoidance:
                 *     A CBF row is allocated for this term, but it is currently inactive in the implementation.
                 *
                 *  3. Self collision avoidance (active only when a valid distance gradient is available):
                 *     h_selcol(q_mani) = self_dist(q_mani) - eps_selcol_min >= 0
                 *     hdot_selcol = ∇_q_mani self_dist^T * qdot_mani
                 *
                 *   ```
                 *   => subject to [ ∇_(q_mani) self_dist.T  I ] * [ eta ] >= [ -a*(self_dist - eps_selcol_min) ]
                 *                                                 [  s  ]
                 *   ```
                 *
                 *  4. Mobile base velocity limit (no slack):
                 *
                 *   ```
                 *   => subject to [ J_mobile ] * [ eta ] in [-vel_limit, +vel_limit]
                 *   ```
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraints which preserve all higher-priority task velocities.
                 *
                 *   ```
                 *   subject to J_eq * eta = v_eq  (v_eq = J_eq * eta_ref, achieved velocities of higher-priority tasks)
                 *
                 *   => subject to [ J_eq  0 ] * [ eta ] = [ v_eq ]
                 *                               [  s  ]
                 *   ```
                 */
                void setEqConstraint() override;
        };

        /**
         * @brief Hierarchical QP Inverse Kinematics solver for mobile manipulators.
         *
         * Accepts a priority-ordered task hierarchy (std::vector<std::map<std::string, Vector6d>>).
         * For each priority level it:
         *   1. Solves a QP minimising the level's task-tracking error.
         *   2. Passes the achieved task velocity of every solved level as a strict equality
         *      constraint to all lower-priority levels, so higher-priority tasks are preserved.
         *
         * All inequality constraints (joint limits, collision, singularity, base velocity) are computed
         * once per cycle via computeSharedConstraints() and supplied to all HQP_IK_base instances.
         */
        class HQPIK
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>) Shared pointer to the RobotData class.
                 * @param dt (double) Control loop time step in seconds.
                 */
                HQPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data, const double dt);

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
                 * @brief Set actuator velocity damping weights only.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 */
                void setJointVelWeight(const Eigen::Ref<const VectorXd>& w_vel_damping) { w_vel_damping_ = w_vel_damping; }

                /**
                 * @brief Set actuator acceleration damping weights only.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping toward eta_now; its size must same as actuator_dof.
                 */
                void setJointAccWeight(const Eigen::Ref<const VectorXd>& w_acc_damping) { w_acc_damping_ = w_acc_damping; }

                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param w_tracking (Eigen::Vector6d) Weight for task space velocity tracking for all the links in the URDF.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping toward eta_now; its size must same as actuator_dof.
                 */
                void setWeight(const Vector6d& w_tracking,
                               const Eigen::Ref<const VectorXd>& w_vel_damping,
                               const Eigen::Ref<const VectorXd>& w_acc_damping) { setTrackingWeight(w_tracking); setJointVelWeight(w_vel_damping); setJointAccWeight(w_acc_damping); }
                /**
                 * @brief Set the weight vector for the cost terms.
                 * @param link_w_tracking (std::map<std::string, Vector6d>) Weight for task velocity tracking per links.
                 * @param w_vel_damping (Eigen::VectorXd) Weight for actuator velocity damping; its size must same as actuator_dof.
                 * @param w_acc_damping (Eigen::VectorXd) Weight for actuator acceleration damping toward eta_now; its size must same as actuator_dof.
                 */
                void setWeight(const std::map<std::string, Vector6d>& link_w_tracking,
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
                 * @brief Get the optimal actuator velocity by solving QP hierarchically.
                 * @param opt_eta (Eigen::VectorXd) Optimal actuator velocity.
                 * @param time_status (TimeDuration) Aggregated timing output (sum of all level timings).
                 * @return (bool) True if every level was solved successfully.
                 */
                bool getOptJointVel(Eigen::Ref<VectorXd> opt_eta, QP::TimeDuration& time_status);

            private:
                /**
                 * @brief Struct to hold the row indices and sizes within SharedIneqData.
                 *        To add a new constraint type: add start/size fields here,
                 *        initialize them in the constructor, and implement the rows
                 *        in computeSharedConstraints().
                 */
                struct SharedIneqIndex
                {
                    int con_q_mani_min_start, con_q_mani_min_size;  // joint position lower-limit CBF
                    int con_q_mani_max_start, con_q_mani_max_size;  // joint position upper-limit CBF
                    int con_sel_col_start,    con_sel_col_size;     // self-collision CBF
                    int con_sing_start,       con_sing_size;        // singularity CBF (reserved)
                    int con_base_vel_start,   con_base_vel_size;    // mobile base velocity limit (no slack)
                } si_index_;

                std::shared_ptr<MobileManipulator::RobotData> robot_data_;  // Shared pointer to the robot data class.
                double dt_;                                                  // control time step size
                int    actuator_dof_;                                        // Number of actuators in the mobile manipulator
                int    mani_dof_;                                            // Number of joints in the manipulator
                int    mobi_dof_;                                            // Number of degrees of freedom in the mobile base
                int    nshared_ineq_;  // total shared inequality rows = sum of si_index_ sizes

                std::vector<std::unique_ptr<HQP_IK_base>> qp_levels_;
                std::vector<int> task_structure_;  // task count per level; change detection
                std::vector<std::map<std::string, Vector6d>> tasks_;  // stored by setDesiredTaskVel()

                bool use_uniform_tracking_ = true;
                Vector6d uniform_w_tracking_;
                std::map<std::string, Vector6d> link_w_tracking_;

                VectorXd w_vel_damping_;  // weight for actuator velocity damping; || eta ||
                VectorXd w_acc_damping_;  // weight for actuator acceleration damping; || (eta - eta_now) / dt ||
                VectorXd eta_prev_;

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

    } // namespace MobileManipulator
} // namespace drc
