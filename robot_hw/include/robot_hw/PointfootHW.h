// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_POINTFOOT_HW_H_
#define _LIMX_POINTFOOT_HW_H_

#include <robot_hw/RobotHW.h>
#include <robot_hw/RobotData.h>
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "limxsdk/pointfoot.h"

namespace hw {

const std::vector<std::string> CONTACT_SENSOR_NAMES = {"L_FOOT", "R_FOOT"};

class PointfootHW : public hw::RobotHW {
public:
  PointfootHW() = default;

  /**
   * \brief Get necessary parameters from the parameter server and initialize hardware_interface.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when initialization is successful, False when failed.
   */
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  /**
   * \brief Communicate with hardware to get data and status of the robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /**
   * \brief Communicate with hardware to publish commands to the robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

  bool loadUrdf(ros::NodeHandle &nh) override;

  bool startBipedController();

  bool stopBipedController();

private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle &nh);

  MotorData jointData_[6]{}; // NOLINT(modernize-avoid-c-arrays)
  ImuData imuData_{};

  realtime_tools::RealtimeBuffer<limxsdk::RobotState> robotstate_buffer_;
  realtime_tools::RealtimeBuffer<limxsdk::ImuData> imudata_buffer_;
  limxsdk::RobotCmd robotCmd_;

  bool contactState_[2]{}; // NOLINT(modernize-avoid-c-arrays)
  int contactThreshold_{40};

  int calibration_state_{-1};
  ros::Publisher cmd_vel_pub_;
  std::map<std::string, int> joystick_btn_map_;
  std::map<std::string, int> joystick_axes_map_;
  ros::ServiceClient switch_controllers_client_;
  ros::ServiceClient list_controllers_client_;

  limxsdk::PointFoot *robot_;
};

} // namespace hw

#endif //_LIMX_POINTFOOT_HW_H_
