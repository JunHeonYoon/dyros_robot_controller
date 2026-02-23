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
#include <mutex>
#include <chrono>

#include "dyros_robot_controller/mobile/robot_data.h"
#include "dyros_robot_controller/mobile/robot_controller.h"

class XLSController 
{
public:
  /**
   * @brief Construct XLS mobile-base example controller.
   * @param dt (double) Control loop time step in seconds.
   */
  XLSController(const double dt);
  /**
   * @brief Destructor. Stops keyboard listener and releases resources.
   */
  ~XLSController();
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
   * @brief Compute wheel commands for the current control mode.
   * @return (std::unordered_map<std::string, double>) Actuator velocity map (wheel joint name -> command).
   */
  std::unordered_map<std::string, double> compute();
  /**
   * @brief Set high-level control mode for the next compute cycle.
   * @param control_mode (std::string) Mode name.
   */
  void setMode(const std::string& control_mode);

private:
  // --- Core configuration/state ---
  const double dt_;
  int wheel_num_{0};

  // --- Wheel states/commands (measured / command buffers) ---
  Eigen::VectorXd wheel_pos_;          // measured wheel angles
  Eigen::VectorXd wheel_vel_;          // measured wheel velocities
  Eigen::VectorXd wheel_vel_desired_;  // desired wheel velocities (command)
  Eigen::Vector3d base_vel_desired_{Eigen::Vector3d::Zero()}; // desired base twist [vx, vy, w] (base frame)

  // --- Mode bookkeeping ---
  std::string control_mode_{"Stop"};
  bool   is_mode_changed_{true};
  double sim_time_{0.0};
  double control_start_time_{0.0};

  // Dyros model/controller handles
  std::shared_ptr<drc::Mobile::RobotData>       robot_data_;
  std::shared_ptr<drc::Mobile::RobotController> robot_controller_;

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

  /**
   * @brief Convert current key state into desired base velocity command.
   * @return (Eigen::Vector3d) Desired base twist [vx, vy, w] in base frame.
   */
  Eigen::Vector3d keysToxdot() const;

  // Internal key state (for continuous commands)
  struct Keys 
  {
    bool up{false}, down{false}, left{false}, right{false};
    bool b{false}, v{false};
    std::chrono::steady_clock::time_point last_event{std::chrono::steady_clock::now()};
  };

  mutable std::mutex keys_mtx_;
  Keys keys_;

  std::atomic<bool> stop_key_{false};
  std::thread key_thread_;
  bool tty_ok_{false};
  struct termios orig_term_{};
};
