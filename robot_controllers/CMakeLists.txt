# Copyright information
#
# © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(robot_controllers)

# Find the Eigen3 library required for linear algebra operations
find_package(Eigen3 REQUIRED)

# Enable generation of compile commands database (used by some code editors)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
# Find and specify required catkin packages and components
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  pointfoot_sdk_lowlevel
  robot_common
  robot_description
  controller_interface
  angles
)

# Specify catkin-specific package configuration
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  pointfoot_sdk_lowlevel
  robot_common
  robot_description
  controller_interface
  angles
)

# Include directories for header files
include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

# Add a shared library target
add_library(${PROJECT_NAME} SHARED
  src/ControllerBase.cpp
  src/PointfootController.cpp
)

# Link libraries to the target
target_link_libraries(${PROJECT_NAME}
  pointfoot_sdk_lowlevel
  ${catkin_LIBRARIES}  # Link with catkin libraries
  onnxruntime          # Link with onnxruntime library
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/. DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Mark resource files for installation
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark other files for installation
install(FILES robot_controllers_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
