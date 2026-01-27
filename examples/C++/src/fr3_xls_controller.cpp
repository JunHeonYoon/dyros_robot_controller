#include "fr3_xls_controller.hpp"

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined. Define in CMake: -DROBOTS_DIRECTORY=\"${CMAKE_SOURCE_DIR}/../robots\""
#endif

FR3XLSController::FR3XLSController(const double dt)
: dt_(dt)
{
    // Paths to URDF/SRDF (model files)
    const std::string urdf = std::string(ROBOTS_DIRECTORY) + "/fr3_xls/" + "fr3_xls.urdf";
    const std::string srdf = std::string(ROBOTS_DIRECTORY) + "/fr3_xls/" + "fr3_xls.srdf";

    // Define kinematic parameters (Mecanum drive) and constraints
    // All geometry is expressed in the base frame:
    //  - base2wheel_positions: 2D position of each wheel center w.r.t. base origin
    //  - base2wheel_angles   : steering angles (fixed here)
    //  - roller_angles       : Mecanum roller angles per wheel
    //  - max_*               : saturation limits used by the controller
    drc::Mobile::KinematicParam mobile_param;
    mobile_param.type = drc::Mobile::DriveType::Mecanum;
    mobile_param.wheel_radius = 0.120;
    mobile_param.base2wheel_positions = {Eigen::Vector2d( 0.2225,  0.2045),  // FL
                                         Eigen::Vector2d( 0.2225, -0.2045),  // FR
                                         Eigen::Vector2d(-0.2225,  0.2045),  // RL
                                         Eigen::Vector2d(-0.2225, -0.2045)}; // RR
    mobile_param.base2wheel_angles = {0, 0, 0, 0};
    mobile_param.roller_angles = {-M_PI/4,  // FL
                                   M_PI/4,  // FR
                                   M_PI/4,  // RL
                                  -M_PI/4}; // RR
    mobile_param.max_lin_speed = 2.0;
    mobile_param.max_ang_speed = 3.0;
    mobile_param.max_lin_acc = 3.0;
    mobile_param.max_ang_acc = 6.0;

    drc::MobileManipulator::JointIndex joint_idx;
    joint_idx.virtual_start = 0;
    joint_idx.mani_start = 3;
    joint_idx.mobi_start = 10;
    drc::MobileManipulator::ActuatorIndex actuator_idx;
    actuator_idx.mani_start = 0;
    actuator_idx.mobi_start = 7;

    // Instantiate dyros robot model/controller
    robot_data_ = std::make_shared<drc::MobileManipulator::RobotData>(dt_, mobile_param, joint_idx, actuator_idx, urdf, srdf);
    robot_controller_ = std::make_shared<drc::MobileManipulator::RobotController>(robot_data_);

    // Degree of freedom
    virtual_dof_ = 3;
    mani_dof_    = robot_data_->getManipulatorDof();
    mobile_dof_  = robot_data_->getMobileDof();
    actuator_dof_ = robot_data_->getActuatordDof();

    // --- Joint-space states (measured / desired / snapshots) ---
    // Mobile Base (computed base twist [vx, vy, wz] in base frame
    base_vel_.setZero();
    base_vel_desired_.setZero();
    base_vel_init_.setZero();
    
    // Virtual joint state (integrated planar base pose and twist: [x, y, yaw])
    q_virtual_.setZero(virtual_dof_);
    q_virtual_desired_.setZero(virtual_dof_);
    q_virtual_init_.setZero(virtual_dof_);
    qdot_virtual_.setZero(virtual_dof_);
    qdot_virtual_desired_.setZero(virtual_dof_);
    qdot_virtual_init_.setZero(virtual_dof_);
    
    // Manipulator joint state
    q_mani_.setZero(mani_dof_);
    q_mani_desired_.setZero(mani_dof_);
    q_mani_init_.setZero(mani_dof_);
    qdot_mani_.setZero(mani_dof_);
    qdot_mani_desired_.setZero(mani_dof_);
    qdot_mani_init_.setZero(mani_dof_);
    tau_mani_desired_.setZero(mani_dof_);  // Manipulator torque command [Nm]

    // Mobile wheel joint state
    q_mobile_.setZero(mobile_dof_);
    q_mobile_desired_.setZero(mobile_dof_);
    q_mobile_init_.setZero(mobile_dof_);
    qdot_mobile_.setZero(mobile_dof_);
    qdot_mobile_desired_.setZero(mobile_dof_); // Wheel velocity command (mapped directly to output)
    qdot_mobile_init_.setZero(mobile_dof_);

    // --- Task-space states (EE pose/twist and snapshots) ---
    link_ee_task_[ee_link_name_] = drc::TaskSpaceData::Zero();

    // --- Gain
    mani_joint_kp_.setZero(mani_dof_);
    mani_joint_kv_.setZero(mani_dof_);
    qpik_mani_damping_.setZero(mani_dof_);
    qpik_base_damping_.setZero();
    qpid_mani_vel_damping_.setZero(mani_dof_);
    qpid_mani_acc_damping_.setZero(mani_dof_);
    qpid_base_vel_damping_.setZero();
    qpid_base_acc_damping_.setZero();
    mani_joint_kp_ << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    mani_joint_kv_ << 30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0;
    qpik_mani_damping_ << 10, 10, 10, 10, 10, 10, 10;
    qpik_base_damping_ << 0.2, 0.2, 0.2; // [vx, vy, wz]
    qpid_mani_vel_damping_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    qpid_mani_acc_damping_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    qpid_base_vel_damping_ << 10, 10, 10;      // [vx, vy, wz]
    qpid_base_acc_damping_ << 0.0001, 0.0001, 0.0001;
    link_task_kp_[ee_link_name_] << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    link_task_kv_[ee_link_name_] << 20.0,  20.0,  20.0,  20.0,  20.0,  20.0;
    link_qpik_tracking_[ee_link_name_] << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    link_qpid_tracking_[ee_link_name_] << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    robot_controller_->setManipulatorJointGain(mani_joint_kp_, mani_joint_kv_);
    robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
    robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_mani_damping_, qpik_base_damping_);
    robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_mani_vel_damping_, qpid_mani_acc_damping_, qpid_base_vel_damping_, qpid_base_acc_damping_);


    // Print FR3 URDF info
    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl; 


    // Global keyboard listener (non-blocking)
    startKeyListener_();
}

FR3XLSController::~FR3XLSController() 
{
    stopKeyListener_();
}

void FR3XLSController::updateModel(const double current_time,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict)
{
    // Time update (shared convention)
    sim_time_ = current_time;

    // Read joint states (joint naming must match the simulator)
    for (size_t i = 0; i < mani_dof_; ++i) 
    {
        const std::string key = "fr3_joint" + std::to_string(i+1);
        q_mani_(i)  = qpos_dict.at(key)[0];
        qdot_mani_(i) = qvel_dict.at(key)[0];
    }
    q_mobile_(0) = qpos_dict.at("front_left_wheel")[0];   // FL
    q_mobile_(1) = qpos_dict.at("front_right_wheel")[0];  // FR
    q_mobile_(2) = qpos_dict.at("rear_left_wheel")[0];    // RL
    q_mobile_(3) = qpos_dict.at("rear_right_wheel")[0];   // RR
    qdot_mobile_(0) = qvel_dict.at("front_left_wheel")[0];
    qdot_mobile_(1) = qvel_dict.at("front_right_wheel")[0];
    qdot_mobile_(2) = qvel_dict.at("rear_left_wheel")[0];
    qdot_mobile_(3) = qvel_dict.at("rear_right_wheel")[0];

    // Compute base twist [vx, vy, wz] from mobile kinematics, then integrate planar base pose q_virtual=[x,y,yaw]
    base_vel_ = robot_data_->computeBaseVel(q_mobile_, qdot_mobile_);
    static double theta = 0;
    theta += base_vel_(2) * dt_;
    theta = atan2(sin(theta), cos(theta));

    Eigen::Matrix3d R_w_m;
    R_w_m << cos(theta), -sin(theta), 0,
             sin(theta),  cos(theta), 0,
             0,           0,          1;
    
    // Rotate base-frame twist into world frame for integrating x/y
    qdot_virtual_ = R_w_m * base_vel_;
    q_virtual_ += qdot_virtual_ * dt_;
    q_virtual_[2] = atan2(sin(q_virtual_[2]), cos(q_virtual_[2]));

    // Push to dyros robot model and cache EE pose/twist
    robot_data_->updateState(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_);
    link_ee_task_[ee_link_name_].x    = robot_data_->getPose(ee_link_name_);
    link_ee_task_[ee_link_name_].xdot = robot_data_->getVelocity(ee_link_name_);
}

std::unordered_map<std::string, double> FR3XLSController::compute() 
{
    // One-time init per mode entry (snapshot current measured states)
    if (is_mode_changed_) 
    {
        is_mode_changed_ = false;
        control_start_time_ = sim_time_;

        // Snapshot current measured states as new references
        q_virtual_init_ = q_virtual_;
        q_mobile_init_  = q_mobile_;
        q_mani_init_    = q_mani_;
        qdot_virtual_init_ = qdot_virtual_;
        qdot_mobile_init_  = qdot_mobile_;
        qdot_mani_init_    = qdot_mani_;
        link_ee_task_[ee_link_name_].setInit();

        // Reset desired trajectories to snapshots
        q_virtual_desired_ = q_virtual_;
        q_mobile_desired_  = q_mobile_;
        q_mani_desired_    = q_mani_;
        qdot_virtual_desired_ = qdot_virtual_;
        qdot_mobile_desired_  = qdot_mobile_;
        qdot_mani_desired_    = qdot_mani_;
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }

    // --- Mode: Home (joint-space cubic to a predefined posture) ---
    if (control_mode_ == "Home") 
    {
        Eigen::Vector7d q_home;
        q_home << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;

        tau_mani_desired_ = robot_controller_->moveManipulatorJointTorqueCubic(q_home,
                                                                               Eigen::VectorXd::Zero(mani_dof_),
                                                                               q_mani_init_,
                                                                               qdot_mani_init_,
                                                                               sim_time_,
                                                                               control_start_time_,
                                                                               3.0,
                                                                               false);
        qdot_mobile_desired_.setZero();
    }
    // --- Mode: QPIK (task-space, QP-based IK with cubic profiling) ---
    else if (control_mode_ == "QPIK") 
    {
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.1, 0.1); // +10 cm in Y and Z

        Eigen::VectorXd qdot_mobile_desired, qdot_mani_desired;
        robot_controller_->QPIKCubic(link_ee_task_,
                                     sim_time_, 
                                     control_start_time_, 
                                     3.0,
                                     qdot_mobile_desired,
                                     qdot_mani_desired);

        qdot_mobile_desired_ = qdot_mobile_desired;
        qdot_mani_desired_ = qdot_mani_desired;

        // Simple Euler integration for generating a joint-position target from qdot_desired (one-step lookahead)
        q_mani_desired_   = q_mani_ + qdot_mani_desired_ * dt_;

        // Map (q_target, qdot_target) -> manipulator torque command (joint PD + gravity compensation inside)
        tau_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_desired_, qdot_mani_desired_, false);
    }
    // --- Mode: Gravity Compensation W QPID (no tracking) ---
    else if (control_mode_ == "Gravity Compensation W QPID") 
    {
        link_ee_task_[ee_link_name_].xddot_desired.setZero();
        Eigen::VectorXd qddot_mobile_desired, tau_mani_desired;
        robot_controller_->QPID(link_ee_task_, qddot_mobile_desired, tau_mani_desired);
        // Integrate mobile acceleration output to wheel velocity command
        qdot_mobile_desired_ = qdot_mobile_ + qddot_mobile_desired * dt_;
        tau_mani_desired_ = tau_mani_desired;
    }

    // Format output for simulator actuators
    std::unordered_map<std::string, double> ctrl_dict;
    ctrl_dict.reserve(actuator_dof_);
    for (size_t i = 0; i < mani_dof_; ++i) 
    {
        ctrl_dict.emplace("fr3_joint" + std::to_string(i+1), tau_mani_desired_(i));
    }
    ctrl_dict["front_left_wheel"]  = qdot_mobile_desired_(0);
    ctrl_dict["front_right_wheel"] = qdot_mobile_desired_(1);
    ctrl_dict["rear_left_wheel"]   = qdot_mobile_desired_(2);
    ctrl_dict["rear_right_wheel"]  = qdot_mobile_desired_(3);
    return ctrl_dict;
}

void FR3XLSController::setMode(const std::string& control_mode) 
{
    // Switch control mode and trigger per-mode re-initialization
    is_mode_changed_ = true;
    control_mode_ = control_mode;
    std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

void FR3XLSController::startKeyListener_() 
{
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_) 
    {
        std::cout << "[FR3XLSController] stdin is not a TTY; keyboard control disabled.\n";
        return;
    }
    setRawMode_();
    stop_key_ = false;
    key_thread_ = std::thread(&FR3XLSController::keyLoop_, this);

    std::cout << "[FR3XLSController] Keyboard: [1]=Home, [2]=QPIK, [3]=Gravity Compensation W QPID\n";
}

void FR3XLSController::stopKeyListener_() 
{
    if (!tty_ok_) return;
    stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join();
    restoreTerm_();
}

void FR3XLSController::setRawMode_() 
{
    if (!tty_ok_) return;
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &orig_term_) == -1) 
    {
        perror("tcgetattr");
        tty_ok_ = false;
        return;
    }
    raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) 
    {
        perror("tcsetattr");
        tty_ok_ = false;
    }
}

void FR3XLSController::restoreTerm_() 
{
    if (!tty_ok_) return;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_) == -1) 
    {
        perror("tcsetattr restore");
    }
}

void FR3XLSController::keyLoop_() 
{
    while (!stop_key_) 
    {
        char c = 0;
        ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n == 1) 
        {
            if (c == '1') 
            {
                setMode("Home");
            } 
            else if (c == '2') 
            {
                setMode("QPIK");
            } 
            else if (c == '3') 
            {
                setMode("Gravity Compensation W QPID");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}
