// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_ROBOT_HW_LOOP_H_
#define _LIMX_ROBOT_HW_LOOP_H_

#include "robot_hw/RobotHW.h"
#include <chrono>
#include <thread>
#include <controller_manager/controller_manager.h>

namespace hw {

class RobotHWLoop {  // NOLINT(cppcoreguidelines-special-member-functions)
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

 public:
  /** \brief Create controller manager.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A pointer which point to hardware_interface.
   */
  RobotHWLoop(ros::NodeHandle& nh, ros::NodeHandle& robot_hw_nh, std::shared_ptr<RobotHW> hardware_interface);

  ~RobotHWLoop();

  /** \brief Timed method that reads current hardware's state, runs the controller code once and sends the new commands
   * to the hardware.
   *
   * Timed method that reads current hardware's state, runs the controller code once and sends the new commands to the
   * hardware.
   *
   */
  void Update();

 protected:
  double cycleTimeErrorThreshold_, loopHz_;
  std::thread loopThread_;
  std::atomic_bool loopRunning_;
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;
  std::shared_ptr<RobotHW> hardwareInterface_;
};

}  // namespace hw

#endif // _LIMX_ROBOT_HW_LOOP_H_