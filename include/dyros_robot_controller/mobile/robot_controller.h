#pragma once
#include "dyros_robot_controller/mobile/robot_data.h"
#include <memory>
#include <algorithm>

namespace drc
{
    namespace Mobile
    {
        /**
         * @brief Controller-side base class for mobile robot controller.
         *
         * This class computes wheel velocities based on desired base velocity
         * using inverse kinematics Jacobians. Supports Differential, Mecanum, and Caster drive types.
        */
        class RobotController
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor.
                 *
                 * @param dt         (double) Control loop time step in seconds.
                 * @param robot_data (std::shared_ptr<Mobile::RobotData>)
                 *                   Shared pointer to the RobotData class.
                */
                RobotController(const double& dt, std::shared_ptr<Mobile::RobotData> robot_data);

                /**
                 * @brief Compute wheel velocities from desired base velocity using inverse kinematics.
                 *
                 * @param base_vel (Eigen::VectorXd) Desired base velocity [vx, vy, wz], size = 3.
                 * @return (Eigen::VectorXd) Computed wheel velocities [rad/s], size = number of wheels.
                */
                virtual VectorXd computeWheelVel(const VectorXd& base_vel);

                /**
                 * @brief Compute inverse kinematics Jacobian (maps base velocity to wheel velocity).
                 *
                 * @return (Eigen::MatrixXd) Jacobian matrix of size [wheel_num x 3].
                */
                virtual MatrixXd computeIKJacobian();

                /**
                 * @brief Generate velocity command with optional constraints (e.g., saturation).
                 *
                 * @param desired_base_vel (Eigen::VectorXd) Desired base velocity [vx, vy, wz], size = 3.
                 * @return (Eigen::VectorXd) Wheel velocity command (possibly saturated), size = number of wheels.
                */
                virtual VectorXd VelocityCommand(const VectorXd& desired_base_vel);

            protected:
                double dt_;                                                  // Control time step in seconds.
                int wheel_num_;                                              // Number of wheels in the robot.
                std::shared_ptr<Mobile::RobotData> robot_data_;  // Pointer to mobile robot data.
                Mobile::KinematicParam param_;                    // Cached kinematic parameters of the robot.

            private:
                /**
                 * @brief Compute inverse kinematics Jacobian for differential drive.
                 * @return (Eigen::MatrixXd) Jacobian [wheel_num x 3].
                */
                MatrixXd DifferentialIKJacobian();

                /**
                 * @brief Compute inverse kinematics Jacobian for mecanum wheels.
                 * @return (Eigen::MatrixXd) Jacobian [wheel_num x 3].
                */
                MatrixXd MecanumIKJacobian();

                /**
                 * @brief Compute inverse kinematics Jacobian for caster-type base.
                 * @return (Eigen::MatrixXd) Jacobian [wheel_num x 3].
                */
                MatrixXd CasterIKJacobian();

                Eigen::Vector3d base_cmd_prev_  = Eigen::Vector3d::Zero();  // v0 (k-1)
                Eigen::Vector3d base_cmd_prev2_ = Eigen::Vector3d::Zero();  // v1 (k-2)
                bool base_cmd_hist_init_ = false;
        };
    } // namespace Mobile
} // namespace drc