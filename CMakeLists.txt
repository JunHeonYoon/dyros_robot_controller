cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release )
endif()
add_compile_options(-O3)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigenpy            REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(pinocchio          REQUIRED)
find_package(OsqpEigen          REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED 
  src/mobile/robot_controller.cpp
  src/mobile/robot_data.cpp
  src/manipulator/robot_controller.cpp
  src/manipulator/robot_data.cpp
  src/mobile_manipulator/robot_controller.cpp
  src/mobile_manipulator/robot_data.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
  src/manipulator/QP_ID.cpp
  src/manipulator/QP_IK.cpp
  src/mobile_manipulator/QP_ID.cpp
  src/mobile_manipulator/QP_IK.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
  rclcpp::rclcpp
  OsqpEigen::OsqpEigen
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


add_library(${PROJECT_NAME}_cpp_wrapper MODULE
  src/bindings.cpp
)

target_link_libraries(${PROJECT_NAME}_cpp_wrapper PUBLIC
  ${PROJECT_NAME}
  Boost::python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}
  Python3::Module
  eigenpy::eigenpy
)

target_include_directories(${PROJECT_NAME}_cpp_wrapper PUBLIC
  ${Python3_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}_cpp_wrapper PROPERTIES
  PREFIX ""
  SUFFIX ".so"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib
)
          
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp pinocchio OsqpEigen eigenpy)
ament_python_install_package(drc)
ament_package()