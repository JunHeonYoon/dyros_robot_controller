#pragma once 
#include "dyros_robot_controller/mobile/robot_data.h"

namespace drc
{
    namespace Mobile
    {
        RobotData::RobotData(const double dt, const KinematicParam& param)
        : param_(param), dt_(dt)
        {
            if(param_.type == DriveType::Differential)
            {
                wheel_num_ = 2;
            }
            else if(param_.type == DriveType::Mecanum)
            {
                assert(param_.roller_angles.size() == param_.base2wheel_positions.size() &&
                param_.roller_angles.size() == param_.base2wheel_angles.size() &&
                param_.base2wheel_positions.size() == param_.base2wheel_angles.size());
                wheel_num_ = param_.roller_angles.size();
            }
            else if(param_.type == DriveType::Caster)
            {
                wheel_num_ = int(param_.base2wheel_positions.size()*2);
            }

            wheel_pos_.setZero(wheel_num_);
            wheel_vel_.setZero(wheel_num_);

            J_mobile_.setZero(3, wheel_num_);
            base_vel_.setZero();

            base_pose_.setIdentity();
        }

        std::string RobotData::getVerbose() const
        {
            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss << std::setprecision(4);

            auto type_to_str = [](DriveType t) -> const char* {
                switch (t) 
                {
                    case DriveType::Differential: return "Differential";
                    case DriveType::Mecanum:      return "Mecanum";
                    case DriveType::Caster:       return "Caster";
                    default:                      return "Unknown";
                }
            };

            // Summary (table)
            oss << " name                | value\n"
                << "---------------------+---------------------------\n"
                << std::left << std::setw(20) << "type"          << " | " << type_to_str(param_.type) << '\n'
                << std::left << std::setw(20) << "wheel_num"     << " | " << wheel_num_ << '\n'
                << std::left << std::setw(20) << "wheel_radius"  << " | " << param_.wheel_radius << '\n';


            if (param_.type == DriveType::Differential)
                oss << std::left << std::setw(20) << "base_width" << " | " << param_.base_width << '\n';
            if (param_.type == DriveType::Caster)
                oss << std::left << std::setw(20) << "offset"     << " | " << param_.wheel_offset << '\n';

            oss << '\n';

            // roller_angles (table)
            if (param_.type == DriveType::Mecanum)
            {
                oss << "roller_angles (rad)\n"
                    << " idx | value\n"
                    << "-----+------------\n";
                for (size_t i = 0; i < param_.roller_angles.size(); ++i)
                    oss << std::right << std::setw(4) << i << " | " << std::setw(10) << param_.roller_angles[i] << '\n';
                oss << '\n';
            }


            // base2wheel_positions (table)
            if(param_.type == DriveType::Mecanum || param_.type == DriveType::Caster)
            {
                Eigen::IOFormat vecfmt(4, 0, "  ", "", "[", "]"); // [x  y  z]
                oss << "base2wheel_positions\n"
                    << " idx | position\n"
                    << "-----+-------------------------\n";
                for (size_t i = 0; i < param_.base2wheel_positions.size(); ++i)
                    oss << std::right << std::setw(4) << i << " | "
                        << param_.base2wheel_positions[i].transpose().format(vecfmt) << '\n';
                oss << '\n';
            }

            // base2wheel_angles (table)
            if(param_.type == DriveType::Mecanum)
            {
                oss << "base2wheel_angles (rad)\n"
                    << " idx | value\n"
                    << "-----+------------\n";
                for (size_t i = 0; i < param_.base2wheel_angles.size(); ++i)
                    oss << std::right << std::setw(4) << i << " | " << std::setw(10) << param_.base2wheel_angles[i] << '\n';
                oss << '\n';
            }

            return oss.str();
        }

        bool RobotData::updateState(const Eigen::Ref<const VectorXd>& wheel_pos, const Eigen::Ref<const VectorXd>& wheel_vel)
        {
            assert(wheel_pos.size() == wheel_num_ && wheel_vel.size() == wheel_num_);
            
            wheel_pos_ = wheel_pos;
            wheel_vel_ = wheel_vel;
            
            J_mobile_ = computeFKJacobian(wheel_pos);
            base_vel_ = J_mobile_ * wheel_vel_;
            base_pose_ = computeBasePose(wheel_pos, wheel_vel);

            return true;
        }

        void RobotData::initBasePose(double x, double y, double yaw)
        {
            base_pose_.setIdentity();

            base_pose_.translation() << x, y;

            Eigen::Rotation2Dd R(yaw);
            base_pose_.linear() = R.toRotationMatrix();
        }

        Vector3d RobotData::computeBaseVel(const Eigen::Ref<const VectorXd>& wheel_pos, const Eigen::Ref<const VectorXd>& wheel_vel)
        {
            assert(wheel_pos.size() == wheel_num_ && wheel_vel.size() == wheel_num_);
            return computeFKJacobian(wheel_pos) * wheel_vel; // Return the forward kinematics result as base velocity
        }

        MatrixXd RobotData::computeFKJacobian(const Eigen::Ref<const VectorXd>& wheel_pos)
        {
            switch (param_.type) 
            {
                case DriveType::Differential:
                    return DifferentialFKJacobian();
                case DriveType::Mecanum:
                    return MecanumFKJacobian();
                case DriveType::Caster:
                    return CasterFKJacobian(wheel_pos);
                default:
                    throw std::runtime_error("Unknown DriveType");
            }
        }

        Affine2d RobotData::computeBasePose(const Eigen::Ref<const VectorXd>& wheel_pos, const Eigen::Ref<const VectorXd>& wheel_vel)
        {
            assert(wheel_pos.size() == wheel_num_);
            assert(wheel_vel.size() == wheel_num_);
            assert(dt_ > 1e-6);

            /* 1. Compute base velocity (body frame) */
            Vector3d v_body = computeBaseVel(wheel_pos, wheel_vel);

            double vx = v_body(0);
            double vy = v_body(1);
            double w  = v_body(2);

            /* 2. Current pose */
            double x = base_pose_.translation()(0);
            double y = base_pose_.translation()(1);
            double theta = atan2(base_pose_.linear()(1,0), base_pose_.linear()(0,0));

            /* 3. Integrate */
            double dtheta = w * dt_;
            double dx_world, dy_world;

            /* Case 1: Near zero angular velocity */
            if (std::fabs(w) < 1e-6)
            {
                double theta_mid = theta + 0.5 * dtheta;

                dx_world = ( vx * cos(theta_mid) - vy * sin(theta_mid) ) * dt_;
                dy_world = ( vx * sin(theta_mid) + vy * cos(theta_mid) ) * dt_;
            }
            /* Case 2: Exact integration */
            else
            {
                double s = sin(dtheta);
                double c = cos(dtheta);

                double A = s / w;
                double B = (1.0 - c) / w;

                double dx_body =  A * vx - B * vy;
                double dy_body =  B * vx + A * vy;

                dx_world =  cos(theta) * dx_body - sin(theta) * dy_body;
                dy_world =  sin(theta) * dx_body + cos(theta) * dy_body;
            }

            /* 4. Update pose */
            x += dx_world;
            y += dy_world;
            theta += dtheta;

            /* 5. Write back to Affine */
            Eigen::Affine2d base_pose;
            base_pose.setIdentity();
            base_pose.translation() << x, y;

            Eigen::Rotation2Dd R(theta);
            base_pose.linear() = R.toRotationMatrix();

            return base_pose;
        }

        MatrixXd RobotData::DifferentialFKJacobian()
        {
            MatrixXd J(3, wheel_num_);
            J.setZero();
            J << param_.wheel_radius/2.,                  param_.wheel_radius/2.,
                 0.,                                      0.,
                -param_.wheel_radius/(param_.base_width), param_.wheel_radius/(param_.base_width);
            
            return J;
        }

        MatrixXd RobotData::MecanumFKJacobian()
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
            
            return DyrosMath::PinvCOD(J_inv);
        }

        // Holmberg, Robert, and Oussama Khatib. "A POWERED-CASTER HOLONOMIC ROBOTIC VEHICLE FOR MOBILE MANIPULATION TASKS."    
        MatrixXd RobotData::CasterFKJacobian(const Eigen::Ref<const VectorXd>& wheel_pos)
        {        
            VectorXd steer_angle(int(wheel_num_/2));
            for(size_t i=0; i<steer_angle.size(); i++) steer_angle(i) = wheel_pos(2*i);

            MatrixXd J_p_tilda(wheel_num_, 3), J_q_inv(wheel_num_, wheel_num_);
            J_p_tilda.setZero();
            J_q_inv.setZero();
            
            for(size_t i=0; i<steer_angle.size(); i++)
            {
                double r = param_.wheel_radius;
                double b = param_.wheel_offset;
                double px = param_.base2wheel_positions[i](0);
                double py = param_.base2wheel_positions[i](1);
                double phi = steer_angle(i);

                J_p_tilda.block(2*i,0,2,3) << 1, 0, -(py + b*sin(phi)),
                                              0, 1,  (px + b*cos(phi));
                J_q_inv.block(2*i,2*i,2,2) <<  b*sin(phi), r*cos(phi),
                                              -b*cos(phi), r*sin(phi);

            }

            return DyrosMath::PinvCOD(J_p_tilda.transpose() * J_p_tilda) * J_p_tilda.transpose() * J_q_inv;
        }
    } // namespace Mobile
} // namespace drc
