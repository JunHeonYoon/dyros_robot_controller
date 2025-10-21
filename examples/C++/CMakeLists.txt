cmake_minimum_required(VERSION 3.16)
project(dyros_robot_controller_example)

# Build profile & OpenGL policy (kept as-is)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(CMAKE_CXX_STANDARD 17)

# Robot resource root (URDF/SRDF/meshes) for your controller code
add_definitions(-DROBOTS_DIRECTORY="${CMAKE_SOURCE_DIR}/../robots")

# Local public headers (for your controller/example code)
include_directories(include)

# =========================================================
# dyros_robot_controller — HOW TO USE IN CMAKE
# ---------------------------------------------------------
# =========================================================
find_package(dyros_robot_controller REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(pinocchio REQUIRED)

# The executable entry point is kept explicit; helpers are attached below.
add_executable(${PROJECT_NAME}
  src/main.cpp
)

# Your control/example sources that use dyros_robot_controller
target_sources(${PROJECT_NAME} PRIVATE
  src/fr3_controller.cpp
  src/xls_controller.cpp
)

# Include dirs
target_include_directories(${PROJECT_NAME} PRIVATE
  ${dyros_robot_controller_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

# Link the dyros controller (PRIMARY dependency)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${dyros_robot_controller_LIBRARIES}
  OsqpEigen::OsqpEigen
  pinocchio::pinocchio
)

# =========================================================
#  MuJoCo - checking for controller
# ---------------------------------------------------------
#  - This section wires a viewer to step and render the sim.
#  - If you replace MuJoCo with real hardware or another sim, keep the dyros part unchanged.
# =========================================================
include(FetchContent)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

# Wrap lodepng (non-CMake project) as a tiny static lib
if (NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG        ${MUJOCO_DEP_VERSION_lodepng}
  )
  FetchContent_GetProperties(lodepng)
  if (NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    add_library(lodepng STATIC
      ${lodepng_SOURCE_DIR}/lodepng.h
      ${lodepng_SOURCE_DIR}/lodepng.cpp
    )
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng    PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
  endif ()
endif ()

# MuJoCo stack
set(OpenGL_GL_PREFERENCE GLVND)
find_package(mujoco REQUIRED)
find_package(glfw3  REQUIRED)
find_package(Threads REQUIRED)

# Attach MuJoCo adapters (windowing/input/step/render)
target_sources(${PROJECT_NAME} PRIVATE
  src/mujoco/glfw_adapter.cc
  src/mujoco/glfw_dispatch.cc
  src/mujoco/platform_ui_adapter.cc
  src/mujoco/simulate.cc
)

# Link the viewer stack as PRIVATE (implementation detail)
target_link_libraries(${PROJECT_NAME} PRIVATE
  mujoco::mujoco
  glfw
  lodepng
  Threads::Threads
)