#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace Manipulator
    {
        /**
         * @brief Class for solving inverse dynamics QP problems for manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse dynamics problems for manipulators using Quadratic Programming.
         */
        class QPID : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<Manipulator::RobotData>)
                 *                   Shared pointer to the RobotData class.
                 */
                QPID(std::shared_ptr<Manipulator::RobotData> robot_data);
                /**
                 * @brief Set the wight vector for the cost terms
                 * @param w_tracking (Eigen::VectorXd) Weight for task acceleration tracking; its size must be 6.
                 * @param w_damping  (Eigen::VectorXd) Weight for joint acceleration damping; its size must same as dof.
                 */
                // TODO: add document to notion
                void setWeight(const VectorXd w_tracking, const VectorXd w_damping);
                /**
                 * @brief Set the desired task space acceleration for the link.
                 * @param xddot_desired (Eigen::VectorXd) Desired task space acceleration.
                 * @param link_name     (std::string) Name of the link.
                 */
                void setDesiredTaskAcc(const VectorXd &xddot_desired, const std::string &link_name);
                /**
                 * @brief Get the optimal joint acceleration and torque by solving QP.
                 * @param opt_qddot   (Eigen::VectorXd) Optimal joint accelerations.
                 * @param opt_torque  (Eigen::VectorXd) Optimal joint torques.
                 * @param time_status (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the problem was solved successfully.
                 */
                bool getOptJoint(VectorXd &opt_qddot, VectorXd &opt_torque, QP::TimeDuration &time_status);

            private:
                /**
                 * @brief Struct to hold the indicies of the QP variables and constraints.
                 */
                struct QPIndex
                {
                    // decision variables
                    int qddot_start; 
                    int torque_start;
                    int slack_q_min_start;
                    int slack_q_max_start;
                    int slack_qdot_min_start;
                    int slack_qdot_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;

                    int qddot_size;
                    int torque_size;
                    int slack_q_min_size;
                    int slack_q_max_size;
                    int slack_qdot_min_size;
                    int slack_qdot_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // equality
                    int con_dyn_start; // dynamics constraint

                    int con_dyn_size;
                    
                    // inequality
                    int con_q_min_start;    // min joint angle
                    int con_q_max_start;    // max joint angle
                    int con_qdot_min_start; // min joint velocity
                    int con_qdot_max_start; // max joint velocity
                    int con_sing_start;     // singularity
                    int con_sel_col_start;  // self collision


                    int con_q_min_size;    // min joint angle size
                    int con_q_max_size;    // max joint angle size
                    int con_qdot_min_size; // min joint velocity size
                    int con_qdot_max_size; // max joint velocity size
                    int con_sing_size;     // singularity size
                    int con_sel_col_size;  // self collision size

                }si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.
                
                int joint_dof_;             // Number of joints in the manipulator
                VectorXd xddot_desired_;    // Desired task space acceleration
                std::string link_name_;     // Name of the link

                VectorXd w_tracking_; // weight for tracking; || x_ddot_des - J*qddot - Jdot*qdot ||
                VectorXd w_damping_;  // weight for damping;  || q_ddot ||
                
                /**
                 * @brief Set the cost function which minimizes task space acceleration error.
                 *        Use slack variables to increase feasibility of QP.
                 *
                 *         min       || x_ddot_des - J*qddot - Jdot*qdot ||_W1^2 + ||q_ddot||_W2^2
                 * [qddot, torque]
                 *
                 * =>      min        1/2 * [ qddot  ]^T * [2*J.T*W1*J + W2  0] * [ qddot  ] + [-2*J.T*W1*(x_ddot_des - Jdot*qdot)].T * [ qddot  ]
                 * [qddot, torque]          [ torque ]     [         0       0]   [ torque ]   [                0                 ]     [ torque ]
                 */
                void setCost() override;
                /**
                 * @brief Set the bound constraint to keep all slack variables non-negative.
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit manipulator joint angles and velocities, avoid singularity and self collision.
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Set the equality constraint which forces joint accelerations and torques to match the equations of motion.
                 *
                 * subject to M * qddot + g = torque
                 *
                 * => subject to [M -I][ qddot  ] = [-g]
                 *                     [ torque ]
                 */
                void setEqConstraint() override;
        };
    } // namespace QP
} // namespacce drc
