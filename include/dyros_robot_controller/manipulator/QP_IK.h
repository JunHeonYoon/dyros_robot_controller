#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Class for solving inverse kinematics QP problems for manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse kinematics problems for manipulators using Quadratic Programming.
         */
        class QPIK : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>)
                 *                   Shared pointer to the RobotData class.
                 */
                QPIK(std::shared_ptr<Manipulator::RobotData> robot_data);
                /**
                 * @brief Set the wight vector for the cost terms
                 * @param w_tracking (Eigen::VectorXd) Weight for task velocity tracking; its size must be 6.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint velocity damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setWeight(const VectorXd w_tracking, const VectorXd w_damping);
                /**
                 * @brief Set the desired task space velocity for the link.
                 * @param xdot_desired (Eigen::VectorXd) Desired task space velocity.
                 * @param link_name    (std::string) Name of the link.
                 */
                void setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name);
                /**
                 * @brief Get the optimal joint velocity by solving QP.
                 * @param opt_qdot    (Eigen::VectorXd) Optimal joint velocity.
                 * @param time_status (TimeDuration) Time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJointVel(VectorXd &opt_qdot, QP::TimeDuration &time_status);

            private:
                /**
                 * @brief Struct to hold the indices of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qdot_start; 
                    int slack_q_min_start;
                    int slack_q_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;

                    int qdot_size;
                    int slack_q_min_size;
                    int slack_q_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;

                    // inequality
                    int con_q_min_start;
                    int con_q_max_start;
                    int con_sing_start;    // singularity
                    int con_sel_col_start; // self collision

                    int con_q_min_size;
                    int con_q_max_size;
                    int con_sing_size;
                    int con_sel_col_size;
                }si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.

                int joint_dof_;              // Number of joints in the manipulator
                VectorXd xdot_desired_;      // Desired task velocity
                std::string link_name_;      // Name of the link

                VectorXd w_tracking_; // weight for tracking; ||x_dot_des - J*q_dot||
                VectorXd w_damping_;  // weight for damping;  || q_dot ||
                

                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 *        Use slack variables to increase feasibility of QP.
                 * 
                 *       min     || x_dot_des - J*q_dot ||_W1^2 + W2 * || q_dot ||_W2^2
                 *       qdot
                 *
                 * =>    min     1/2 * qdot.T * (2*J.T*W1*J + W2) * qdot + (-2*J.T*W1*x_dot_des).T * qdot
                 *       qdot
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint which limits manipulator joint velocities and keeps all slack variables non-negative.
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit manipulator joint angles and avoid singularity and self collision.
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Not implemented.
                 */
                void setEqConstraint() override;
        };
    } // namespace QP
} // namespace drc