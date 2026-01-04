#pragma once
#include "dyros_robot_controller/type_define.h"
#include <memory>
#include <sstream>
#include <iomanip>

using namespace Eigen;

namespace drc
{
    namespace Mobile
    {
        /**
         * @brief Abstract base class for mobile robot data.
         * 
         * This class provides a general interface and shared logic for various mobile robot types
         * (e.g., Differential, Mecanum, Caster). It supports state update, forward velocity computation,
         * and Jacobian calculation based on the specified kinematic parameters.
        */
        class RobotData
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                /**
                 * @brief Constructor with kinematic parameters.
                 * @param param (RobotData::Mobile::KinematicParam) Kinematic parameter object containing drive type and geometry.
                */
                RobotData(const KinematicParam& param);

                virtual std::string getVerbose() const;

                /**
                 * @brief Update internal mobile robot data.
                 * @param wheel_pos (Eigen::VectorXd) Wheel positions [rad]; its size must be same as wheel_num_.
                 * @param wheel_vel (Eigen::VectorXd) Wheel velocities [rad/s]; its size must be same as wheel_num_.
                 * @return True if state update is successful.
                */
                virtual bool updateState(const VectorXd& wheel_pos, const VectorXd& wheel_vel);

                // ================================ Compute Functions ================================
                /**
                 * @brief Compute the robot base velocity in the base frame.
                 * @param wheel_pos (Eigen::VectorXd) Wheel positions [rad]; its size must be same as wheel_num_.
                 * @param wheel_vel (Eigen::VectorXd) Wheel velocities [rad/s]; its size must be same as wheel_num_.
                 * @return (Eigen::Vector3d) Base velocity vector [vx, vy, wz].
                */
                virtual Vector3d computeBaseVel(const VectorXd& wheel_pos, const VectorXd& wheel_vel);
                /**
                 * @brief Compute the forward kinematics Jacobian of the base.
                 *        Maps wheel velocities to base velocity.
                 * @param wheel_pos (Eigen::VectorXd) Wheel positions [rad]; its size must be same as wheel_num_.
                 * @return (Eigen::MatrixXd) Base velocity Jacobian matrix.
                */
                virtual MatrixXd computeFKJacobian(const VectorXd& wheel_pos);

                // ================================ Get Functions ================================
                /**
                 * @brief Get the kinematic parameters used for this base.
                 * @return (RobotData::Mobile::KinematicParam) Reference to KinematicParam.
                */
                virtual const KinematicParam& getKineParam() const {return param_;}

                /**
                 * @brief Get the number of wheels.
                 * @return (int) Integer number of wheels.
                 */
                virtual const int& getWheelNum() const {return wheel_num_;}

                /**
                 * @brief Get the current wheel positions.
                 * @return (Eigen::VectorXd) Vector of wheel positions.
                */ 
                virtual const VectorXd& getWheelPosition() const {return wheel_pos_;} 

                /**
                 * @brief Get the current wheel velocities.
                 * @return (Eigen::VectorXd) Vector of wheel velocities.
                */
                virtual const VectorXd& getWheelVelocity() const {return wheel_vel_;} 

                /**
                 * @brief Get the last computed base velocity.
                 * @return (Eigen::Vector3d) Base velocity vector [vx, vy, wz].
                */
                virtual const Vector3d& getBaseVel() const {return base_vel_;}

                /**
                 * @brief Get the last computed base Jacobian.
                 * @return (Eigen::MatrixXd) Jacobian matrix from wheel velocity to base velocity.
                */
                virtual const MatrixXd& getFKJacobian() const {return J_mobile_;}
                
            protected:
                KinematicParam param_;    // Kinematic parameters for this mobile base.
                int wheel_num_;           // Number of wheels used in the robot.

                VectorXd wheel_pos_;      // Last updated wheel positions.
                VectorXd wheel_vel_;      // Last updated wheel velocities.

                MatrixXd J_mobile_;       // Forward kinematics Jacobian matrix.
                Vector3d base_vel_;       // Last computed base velocity.
            private:
                /**
                 * @brief Compute Jacobian for differential drive base.
                 * @return (Eigen::MatrixXd) Jacobian matrix [3 x N_wheels].
                */
                MatrixXd DifferentialFKJacobian();

                /**
                 * @brief Compute Jacobian for mecanum wheel base.
                 * @return (Eigen::MatrixXd) Jacobian matrix [3 x N_wheels].
                */
                MatrixXd MecanumFKJacobian();

                /**
                 * @brief Compute Jacobian for caster-based base.
                 * @param wheel_pos Wheel positions (to resolve caster angles).
                 * @return (Eigen::MatrixXd) Jacobian matrix [3 x N_wheels].
                */
                MatrixXd CasterFKJacobian(const VectorXd& wheel_pos);

        };
    } // namespace RobotData
} // namespace drc