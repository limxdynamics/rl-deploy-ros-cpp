// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_ROBOT_HW_H_
#define _LIMX_ROBOT_HW_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <urdf/model.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robot_common/hardware_interface/ContactSensorInterface.h>
#include <robot_common/hardware_interface/HybridJointInterface.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace hw {

class RobotHW : public hardware_interface::RobotHW {
public:
  RobotHW() = default;

  /**
   * \brief Initializes the robot hardware.
   *
   * Initializes the robot hardware interface by setting up joint state, IMU sensor, hybrid joint, and contact sensor interfaces.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True if initialization is successful, false otherwise.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

  /**
   * \brief Retrieves the URDF model of the robot.
   *
   * Returns the URDF model of the robot.
   *
   * @return Shared pointer to the URDF model.
   */
  virtual std::shared_ptr<urdf::Model const> getUrdfModel() { return urdfModel_; }

  /**
   * \brief Loads the URDF model of the robot.
   *
   * Loads the URDF model of the robot from the ROS parameter server.
   *
   * @param nh Node-handle for loading URDF.
   * @return True if URDF loading is successful, false otherwise.
   */
  virtual bool loadUrdf(ros::NodeHandle& nh) { return false; }

protected:
  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;    // Interface for joint state data. 
  hardware_interface::ImuSensorInterface imuSensorInterface_;      // Interface for IMU sensor data. 
  robot_common::HybridJointInterface hybridJointInterface_;         // Interface for hybrid joint data.
  robot_common::ContactSensorInterface contactSensorInterface_;    // Interface for contact sensor data.
  std::shared_ptr<urdf::Model> urdfModel_;                        // Shared pointer to URDF model.
};

}  // namespace hw

#endif // _LIMX_ROBOT_HW_H_
