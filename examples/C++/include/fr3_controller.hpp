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
        /**
         * @brief Construct FR3 example controller.
         * @param dt (double) Control loop time step in seconds.
         */
        FR3Controller(const double dt);
        /**
         * @brief Destructor. Stops keyboard listener and releases resources.
         */
        ~FR3Controller();
        /**
         * @brief Update internal model/state from simulator measurements.
         * @param current_time (double) Current simulation time in seconds.
         * @param qpos_dict (std::unordered_map<std::string, Eigen::VectorXd>) Joint position map keyed by model name.
         * @param qvel_dict (std::unordered_map<std::string, Eigen::VectorXd>) Joint velocity map keyed by model name.
         */
        void updateModel(const double current_time,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict);
        /**
         * @brief Compute actuator torques for the current control mode.
         * @return (std::unordered_map<std::string, double>) Actuator torque map (joint name -> tau [Nm]).
         */
        std::unordered_map<std::string, double> compute();
        /**
         * @brief Set high-level control mode for the next compute cycle.
         * @param control_mode (std::string) Mode name.
         */
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
        Eigen::VectorXd joint_kp_;
        Eigen::VectorXd joint_kv_;
        Vector6d        task_ik_kp_;
        Vector6d        task_id_kp_;
        Vector6d        task_id_kv_;
        Vector6d        qpik_tracking_;
        Eigen::VectorXd qpik_vel_damping_;
        Eigen::VectorXd qpik_acc_damping_;
        Vector6d        qpid_tracking_;
        Eigen::VectorXd qpid_vel_damping_;
        Eigen::VectorXd qpid_acc_damping_;

        // Dyros model/controller handles
        std::shared_ptr<drc::Manipulator::RobotData>       robot_data_;
        std::shared_ptr<drc::Manipulator::RobotController> robot_controller_;

        // --- Keyboard interface (non-blocking; background thread) ---
        /**
         * @brief Start background keyboard listener thread.
         */
        void startKeyListener_();
        /**
         * @brief Stop background keyboard listener thread.
         */
        void stopKeyListener_();
        /**
         * @brief Keyboard polling loop running in background thread.
         */
        void keyLoop_();
        /**
         * @brief Switch terminal to non-canonical raw input mode.
         */
        void setRawMode_();
        /**
         * @brief Restore terminal mode saved before setRawMode_().
         */
        void restoreTerm_();

        std::atomic<bool> stop_key_{false};
        std::thread key_thread_;
        bool tty_ok_{false};
        struct termios orig_term_{};
};
