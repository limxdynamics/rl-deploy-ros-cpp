# Copyright information
#
# © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Declare the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Declare the project
project(robot_common)

# Declare the include directories for the Catkin package
# The directories specified here contain header files that other packages need when building or linking to this package.
catkin_package(INCLUDE_DIRS include)

# Install the include directory to the global Catkin include destination
install(DIRECTORY include/. DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
