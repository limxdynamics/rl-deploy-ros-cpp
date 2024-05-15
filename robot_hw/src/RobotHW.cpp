// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_hw/RobotHW.h"

namespace hw {
// Initialize the RobotHW
bool RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle & /*robot_hw_nh*/) {
  // Load URDF for the robot
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up URDF");
    return false;
  }

  // Register hardware interfaces
  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  return true;
}
}  // namespace hw

