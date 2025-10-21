cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(Boost REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
find_package(eigenpy            REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(pinocchio          REQUIRED)
find_package(OsqpEigen          REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen3 pinocchio OsqpEigen eigenpy
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
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
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
  Eigen3::Eigen
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
  LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/python${Python3_VERSION_MAJOR}/dist-packages"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}/dist-packages
)