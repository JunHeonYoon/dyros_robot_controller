cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(mujoco_ros_sim  REQUIRED)   # bindings + helper
find_package(rclcpp          REQUIRED)
find_package(Eigen3          REQUIRED)
find_package(pinocchio       REQUIRED)
find_package(OsqpEigen       REQUIRED)
find_package(std_msgs        REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(sensor_msgs     REQUIRED)

include(${mujoco_ros_sim_DIR}/MRSController.cmake)

mrs_add_controller(fr3_controller
                   src/fr3_controller/controller.cpp
                   src/fr3_controller/robot_data.cpp)

target_link_libraries(fr3_controller PUBLIC
                      pinocchio::pinocchio
                      OsqpEigen::OsqpEigen)

mrs_add_controller(husky_fr3_controller
                   src/husky_fr3_controller/controller.cpp
                   src/husky_fr3_controller/robot_data.cpp)

target_link_libraries(husky_fr3_controller PUBLIC
                      pinocchio::pinocchio
                      OsqpEigen::OsqpEigen)

install(DIRECTORY robot/  DESTINATION share/${PROJECT_NAME}/robot)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_target_dependencies(fr3_controller PUBLIC 
  std_msgs
  geometry_msgs
  sensor_msgs)

ament_target_dependencies(husky_fr3_controller PUBLIC 
  std_msgs
  geometry_msgs
  sensor_msgs)
  
  
  install(DIRECTORY   config/
          DESTINATION share/${PROJECT_NAME}/config)
  
  install(PROGRAMS    dyros_robot_controller/fr3_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      FR3ControllerQT)

  install(PROGRAMS    dyros_robot_controller/husky_fr3_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      HuskyFR3ControllerQT)
    
    
  install(PROGRAMS    dyros_robot_controller/husky_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      HuskyControllerQT)
    
  install(PROGRAMS    dyros_robot_controller/xls_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      XLSControllerQT)
    
  install(PROGRAMS    dyros_robot_controller/pcv_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      PCVControllerQT)
    
  install(PROGRAMS    dyros_robot_controller/ur5e_controller/ros2_qt_gui.py
          DESTINATION lib/${PROJECT_NAME}
          RENAME      UR5eControllerQT)
            
ament_python_install_package(${PROJECT_NAME})
ament_package()
