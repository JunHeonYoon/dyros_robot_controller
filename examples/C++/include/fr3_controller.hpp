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

#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"

class FR3Controller 
{
    public:
        FR3Controller(const double dt);
        ~FR3Controller();

        void updateModel(const double current_time,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict);

        // Compute control for current mode.
        // Returns: actuator torque map (joint name -> tau [Nm]).
        std::unordered_map<std::string, double> compute();

        void setMode(const std::string& control_mode);

    private:
        const double dt_;
        int dof_{0};

        // --- Joint-space states (measured / desired / snapshots) ---
        Eigen::VectorXd q_;            // measured joints
        Eigen::VectorXd qdot_;         // measured joint velocities
        Eigen::VectorXd q_desired_;    // desired joints
        Eigen::VectorXd qdot_desired_; // desired joint velocities
        Eigen::VectorXd q_init_;       // snapshot at mode entry
        Eigen::VectorXd qdot_init_;    // snapshot at mode entry
        Eigen::VectorXd tau_desired_;  // output torques

        // --- Task-space (end-effector) states (measured / desired / snapshots) ---
        std::map<std::string, drc::TaskSpaceData> link_ee_task_;
        std::string ee_link_name_{"fr3_link8"}; // EE link name (FR3 URDF)

        // --- Mode bookkeeping (unified naming with Python example) ---
        std::string control_mode_{"Home"};
        bool   is_mode_changed_{true};
        double sim_time_{0.0};
        double control_start_time_{0.0};

        //  --- Gain
        Eigen::VectorXd                 joint_kp_;
        Eigen::VectorXd                 joint_kv_;
        Eigen::VectorXd                 qpik_damping_;
        Eigen::VectorXd                 qpid_vel_damping_;
        Eigen::VectorXd                 qpid_acc_damping_;
        std::map<std::string, Vector6d> link_task_kp_;
        std::map<std::string, Vector6d> link_task_kv_;
        std::map<std::string, Vector6d> link_qpik_tracking_;
        std::map<std::string, Vector6d> link_qpid_tracking_;

        // Dyros model/controller handles
        std::shared_ptr<drc::Manipulator::RobotData>       robot_data_;
        std::shared_ptr<drc::Manipulator::RobotController> robot_controller_;

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
