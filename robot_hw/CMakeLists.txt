# Copyright information
#
# © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

project(robot_hw)

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
  hardware_interface
  controller_manager
  urdf
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
  hardware_interface
  controller_manager
  urdf
)

find_package(urdfdom REQUIRED)

# Include directories for header files
include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

add_executable(pointfoot_node
        ${PROJECT_SOURCE_DIR}/src/RobotHW.cpp
        ${PROJECT_SOURCE_DIR}/src/RobotHWLoop.cpp
        ${PROJECT_SOURCE_DIR}/src/PointfootHW.cpp
        ${PROJECT_SOURCE_DIR}/src/pointfootNode.cpp
)

target_link_libraries(pointfoot_node
        ${catkin_LIBRARIES}  # Link with catkin libraries
        ${urdfdom_LIBRARIES}
        pointfoot_sdk_lowlevel
        pthread
)

# Mark executables and/or libraries for installation
install(TARGETS pointfoot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/. DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Mark resource files for installation
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
