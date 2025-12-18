#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

class FR3XLSController 
{
    public:
        FR3XLSController(const double dt);
        ~FR3XLSController();

        void updateModel(const double current_time,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict);

        // Compute control for current mode.
        // Returns: actuator torque map (joint name -> tau [Nm]).
        std::unordered_map<std::string, double> compute();

        void setMode(const std::string& control_mode);

    private:
        const double dt_;
        int virtual_dof_;
        int mani_dof_;
        int mobile_dof_;
        int actuator_dof_;

        // --- Joint-space states (measured / desired / snapshots) ---
        // Mobile Base (computed base twist [vx, vy, wz] in base frame
        Eigen::Vector3d base_vel_;
        Eigen::Vector3d base_vel_desired_;
        Eigen::Vector3d base_vel_init_;
        
        // Virtual joint state (integrated planar base pose and twist: [x, y, yaw])
        Eigen::VectorXd q_virtual_;
        Eigen::VectorXd q_virtual_desired_;
        Eigen::VectorXd q_virtual_init_;
        Eigen::VectorXd qdot_virtual_;
        Eigen::VectorXd qdot_virtual_desired_;
        Eigen::VectorXd qdot_virtual_init_;
        
        // Manipulator joint state
        Eigen::VectorXd q_mani_;
        Eigen::VectorXd q_mani_desired_;
        Eigen::VectorXd q_mani_init_;
        Eigen::VectorXd qdot_mani_;
        Eigen::VectorXd qdot_mani_desired_;
        Eigen::VectorXd qdot_mani_init_;
        Eigen::VectorXd tau_mani_desired_;  // Manipulator torque command [Nm]

        // Mobile wheel joint state
        Eigen::VectorXd q_mobile_;
        Eigen::VectorXd q_mobile_desired_;
        Eigen::VectorXd q_mobile_init_;
        Eigen::VectorXd qdot_mobile_;
        Eigen::VectorXd qdot_mobile_desired_; // Wheel velocity command (mapped directly to output)
        Eigen::VectorXd qdot_mobile_init_;

        // --- Task-space (end-effector) states (measured / desired / snapshots) ---
        std::map<std::string, drc::TaskSpaceData> link_ee_task_;
        std::string ee_link_name_{"fr3_hand_tcp"}; // EE link name (FR3XLS URDF)

        // --- Mode bookkeeping (unified naming with Python example) ---
        std::string control_mode_{"Home"};
        bool   is_mode_changed_{true};
        double sim_time_{0.0};
        double control_start_time_{0.0};

        //  --- Gain
        Eigen::VectorXd                 mani_joint_kp_;
        Eigen::VectorXd                 mani_joint_kv_;
        Eigen::VectorXd                 qpik_damping_;
        Eigen::VectorXd                 qpid_vel_damping_;
        Eigen::VectorXd                 qpid_acc_damping_;
        std::map<std::string, Vector6d> link_task_kp_;
        std::map<std::string, Vector6d> link_task_kv_;
        std::map<std::string, Vector6d> link_qpik_tracking_;
        std::map<std::string, Vector6d> link_qpid_tracking_;

        // Dyros model/controller handles
        std::shared_ptr<drc::MobileManipulator::RobotData>       robot_data_;
        std::shared_ptr<drc::MobileManipulator::RobotController> robot_controller_;

        // --- Keyboard interface (non-blocking; background thread) ---
        void startKeyListener_();
        void stopKeyListener_();
        void keyLoop_();

        void setRawMode_();
        void restoreTerm_();

        std::atomic<bool> stop_key_{false};
        std::thread key_thread_;
        bool tty_ok_{false};
        struct termios orig_term_{};
};
