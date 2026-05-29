# Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
#
# Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
# Seoul National University, under a research agreement with ETRI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3)

# Embed RPATH so installed .so files find libdyros_robot_controller without LD_LIBRARY_PATH
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost   REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
find_package(eigenpy REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)

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

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "12")
  # Workaround for sporadic GCC 11 ICE in manipulator/robot_controller.cpp at -O3.
  set_source_files_properties(
    src/manipulator/robot_controller.cpp
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-tree-vectorize"
  )
endif()

target_sources(${PROJECT_NAME} PRIVATE
  src/manipulator/QP_ID.cpp
  src/manipulator/QP_IK.cpp
  src/manipulator/HQP_IK.cpp
  src/manipulator/HQP_ID.cpp
  src/mobile_manipulator/QP_ID.cpp
  src/mobile_manipulator/QP_IK.cpp
  src/mobile_manipulator/HQP_IK.cpp
  src/mobile_manipulator/HQP_ID.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
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

# ── Install targets ──────────────────────────────────────────────────────────

install(
  TARGETS ${PROJECT_NAME}
  EXPORT dyros_robot_controllerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES LICENSE NOTICE
  DESTINATION share/${PROJECT_NAME}
)

# Python wrapper → site-packages (importable) AND lib (for ldconfig)
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib
)

# drc Python package
install(DIRECTORY drc/
  DESTINATION ${PYTHON_INSTALL_DIR}/drc
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc"       EXCLUDE
)

# ── CMake package config (enables find_package(dyros_robot_controller)) ──────

include(CMakePackageConfigHelpers)

configure_package_config_file(
  cmake/dyros_robot_controllerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dyros_robot_controllerConfig.cmake
  INSTALL_DESTINATION .
)

install(EXPORT dyros_robot_controllerTargets
  NAMESPACE ${PROJECT_NAME}::
  FILE dyros_robot_controllerTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dyros_robot_controllerConfig.cmake
  DESTINATION .
)

# ── Uninstall target ──────────────────────────────────────────────────────────
add_custom_target(uninstall
  COMMAND xargs rm -f < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
  COMMENT "Uninstalling..."
)
