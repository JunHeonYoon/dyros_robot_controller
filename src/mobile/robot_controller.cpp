#include "dyros_robot_controller/mobile/robot_controller.h"


namespace
{
    static inline double clampScalar(double x, double lo, double hi)
    {
        return std::min(std::max(x, lo), hi);
    }

    // value clamp: v in [min_v, max_v]
    static inline double limitValue(double v, double min_v, double max_v)
    {
        return clampScalar(v, min_v, max_v);
    }

    // first derivative (acc) clamp: dv in [dv_min, dv_max]
    static inline double limitFirstDerivative(double v, double v0, double dt,
                                            double min_acc, double max_acc)
    {
        const double dv_min = min_acc * dt;
        const double dv_max = max_acc * dt;
        const double dv = clampScalar(v - v0, dv_min, dv_max);
        return v0 + dv;
    }

    // second derivative (jerk) clamp: only when (dv - dv0)*dv > 0
    static inline double limitSecondDerivative(double v, double v0, double v1, double dt, double min_jerk, double max_jerk)
    {
        const double dv  = v  - v0;
        const double dv0 = v0 - v1;

        // Same condition as control_toolbox: apply jerk limit only while accelerating or reverse-accelerating
        if ((dv - dv0) * dv > 0.0)
        {
            const double dt2 = dt * dt;
            const double da_min = min_jerk * dt2;
            const double da_max = max_jerk * dt2;

            const double da = clampScalar(dv - dv0, da_min, da_max);
            return v0 + dv0 + da;
        }
        return v;
    }

} // namespace

namespace drc
{
    namespace Mobile
    {
        RobotController::RobotController(std::shared_ptr<Mobile::RobotData> robot_data)
        : dt_(robot_data->getDt()), robot_data_(robot_data)
        {
            param_ = robot_data_->getKineParam();
            wheel_num_ = robot_data_->getWheelNum();
        }

        VectorXd RobotController::VelocityCommand(const VectorXd& desired_base_vel)
        {
            assert(desired_base_vel.size() == 3); // [vx, vy, omega]

            Eigen::Vector3d v_des = desired_base_vel;

            // ---- initialize history on first call ----
            if (!base_cmd_hist_init_)
            {
                base_cmd_prev_  = Eigen::Vector3d::Zero();
                base_cmd_prev2_ = Eigen::Vector3d::Zero();
                base_cmd_hist_init_ = true;
            }

            // We will apply per-axis jerk/acc limits first,
            // then apply linear speed norm limit and angular speed limit.

            Eigen::Vector3d v_limited = v_des;

            // // -------- JERK LIMIT (per-axis) --------
            // // Linear: use max_lin_jerk on vx, vy (symmetric)
            // const double min_lin_jerk = -param_.max_lin_jerk;
            // const double max_lin_jerk =  param_.max_lin_jerk;

            // v_limited(0) = limitSecondDerivative(v_limited(0), base_cmd_prev_(0), base_cmd_prev2_(0),
            //                                     dt_, min_lin_jerk, max_lin_jerk);
            // v_limited(1) = limitSecondDerivative(v_limited(1), base_cmd_prev_(1), base_cmd_prev2_(1),
            //                                     dt_, min_lin_jerk, max_lin_jerk);

            // // Angular: use max_ang_jerk (symmetric)
            // const double min_ang_jerk = -param_.max_ang_jerk;
            // const double max_ang_jerk =  param_.max_ang_jerk;

            // v_limited(2) = limitSecondDerivative(v_limited(2), base_cmd_prev_(2), base_cmd_prev2_(2),
            //                                     dt_, min_ang_jerk, max_ang_jerk);

            // -------- ACC LIMIT (per-axis) --------
            const double min_lin_acc = -param_.max_lin_acc;
            const double max_lin_acc =  param_.max_lin_acc;

            v_limited(0) = limitFirstDerivative(v_limited(0), base_cmd_prev_(0), dt_, min_lin_acc, max_lin_acc);
            v_limited(1) = limitFirstDerivative(v_limited(1), base_cmd_prev_(1), dt_, min_lin_acc, max_lin_acc);

            const double min_ang_acc = -param_.max_ang_acc;
            const double max_ang_acc =  param_.max_ang_acc;

            v_limited(2) = limitFirstDerivative(v_limited(2), base_cmd_prev_(2), dt_, min_ang_acc, max_ang_acc);

            // -------- VELOCITY LIMIT --------
            // 1) linear speed norm limit (keep your original policy)
            Eigen::Vector2d lin = v_limited.head<2>();
            const double lin_speed = lin.norm();
            if (lin_speed > param_.max_lin_speed && lin_speed > 1e-9)
            {
                lin *= (param_.max_lin_speed / lin_speed);
            }
            v_limited.head<2>() = lin;

            // 2) angular speed scalar limit
            v_limited(2) = limitValue(v_limited(2), -param_.max_ang_speed, param_.max_ang_speed);

            // ---- update history with the FINAL limited command ----
            base_cmd_prev2_ = base_cmd_prev_;
            base_cmd_prev_  = v_limited;

            // Convert base cmd -> wheel cmd
            return computeWheelVel(v_limited);
        }

        VectorXd RobotController::computeWheelVel(const VectorXd& base_vel)
        {
            assert(base_vel.size() == 3); // Ensure base_vel has three elements for vx, vy, omega       
            return computeIKJacobian() * base_vel;
        }

        MatrixXd RobotController::computeIKJacobian()
        {
            switch (param_.type) 
            {
                case Mobile::DriveType::Differential:
                    return DifferentialIKJacobian();
                case Mobile::DriveType::Mecanum:
                    return MecanumIKJacobian();
                case Mobile::DriveType::Caster:
                    return CasterIKJacobian();
                default:
                    throw std::runtime_error("Unknown DriveType");
            }
        }

        MatrixXd RobotController::DifferentialIKJacobian()
        {
            Matrix<double,2,3> J_inv;
            J_inv.setZero();
            J_inv << 1/param_.wheel_radius, 0, -param_.base_width/(2*param_.wheel_radius),
                     1/param_.wheel_radius, 0,  param_.base_width/(2*param_.wheel_radius);
            
            return J_inv;
        }

        MatrixXd RobotController::MecanumIKJacobian()
        {
            MatrixXd J_inv(wheel_num_, 3);
            J_inv.setZero();

            for(size_t i=0; i<wheel_num_; i++)
            {
                double r = param_.wheel_radius;
                double gamma = param_.roller_angles[i];
                double p_x = param_.base2wheel_positions[i](0);
                double p_y = param_.base2wheel_positions[i](1);
                double p_t = param_.base2wheel_angles[i];

                MatrixXd A1, A2, A3;
                A1.setZero(2,3);
                A2.setZero(2,2);
                A3.setZero(1,2);

                A1 << 1, 0, -p_y,
                        0, 1, p_x;
                A2 <<  cos(p_t), sin(p_t),
                        -sin(p_t), cos(p_t);
                A3 << 1, tan(gamma);
                J_inv.row(i) = (1.0 / r) * A3 * A2 * A1;
            }

            return J_inv;
        }

        MatrixXd RobotController::CasterIKJacobian()
        {
            VectorXd steer_angle(int(wheel_num_/2));
            for(size_t i=0; i<steer_angle.size(); i++) steer_angle(i) = robot_data_->getWheelPosition()(2*i);

            MatrixXd J_tilda(wheel_num_, 3);
            J_tilda.setZero();

            for(size_t i=0; i<steer_angle.size(); i++)
            {
                double r = param_.wheel_radius;
                double b = param_.wheel_offset;
                double px = param_.base2wheel_positions[i](0);
                double py = param_.base2wheel_positions[i](1);
                double phi = steer_angle(i);

                J_tilda.block(2*i,0,2,3) << -sin(phi)/b, cos(phi)/b, (px*cos(phi) + py*sin(phi))/b -1,
                                             cos(phi)/r, sin(phi)/r, (px*sin(phi) - py*cos(phi))/r;
            }
            return J_tilda;
        }
    } // namespace Mobile
} // namespace drc