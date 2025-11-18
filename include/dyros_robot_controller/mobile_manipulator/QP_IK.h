#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace MobileManipulator
    {
        /**
         * @brief Class for solving inverse kinematics QP problems for mobile manipulators.
         * 
         * This class inherits from QPBase and implements methods to set up and solve
         * inverse kinematics problems for mobile manipulators using Quadratic Programming.
         */
        class QPIK : public QP::QPBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 * @param robot_data (std::shared_ptr<MobileManipulator::RobotData>)
                 *                   Shared pointer to the RobotData class.
                 */
                QPIK(std::shared_ptr<MobileManipulator::RobotData> robot_data);
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
                    int eta_start; // qdot_actuated
                    int slack_q_mani_min_start;
                    int slack_q_mani_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;
    
                    int eta_size;
                    int slack_q_mani_min_size;
                    int slack_q_mani_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // inequality
                    int con_q_mani_min_start;    // min q_manipulator
                    int con_q_mani_max_start;    // max q_manipulator
                    int con_sing_start;          // singularity
                    int con_sel_col_start;       // self collision
    
                    int con_q_mani_min_size;    // min q_manipulator size
                    int con_q_mani_max_size;    // max q_manipulator size
                    int con_sing_size;          // singularity size
                    int con_sel_col_size;       // self collision size
                }si_index_;
    
                std::shared_ptr<MobileManipulator::RobotData> robot_data_;   // Shared pointer to the robot data class.
                
                int actuator_dof_;      // Number of actuators in the mobile manipulator
                int mani_dof_;          // Number of joints in the manipulator
                int mobi_dof_;          // Number of degrees of freedom in the mobile base
                VectorXd xdot_desired_; // Desired task velocity
                std::string link_name_; // Name of the link
                
                /**
                 * @brief Set the cost function which minimizes task space velocity error.
                 * 
                 *       min     || x_dot_des - J_tilda*eta ||_2^2 + W * || eta ||_2^2
                 *       eta
                 *
                 * =>    min     1/2 * eta.T * (2*J_tilda.T*J_tilda + W) * eta + (-2*J_tilda.T*x_dot_des).T * eta
                 *       eta
                 */
                void setCost() override;
                /**
                 * @brief Not yet implemented.
                 */
                void setBoundConstraint() override;
                /**
                 * @brief Set the inequality constraints which limit joint angles and avoid singularity and self collision.
                 */
                void setIneqConstraint() override;
                /**
                 * @brief Not implemented.
                 */
                void setEqConstraint() override;
        };
    } // namespace MobileManipulator
} // namespace drc