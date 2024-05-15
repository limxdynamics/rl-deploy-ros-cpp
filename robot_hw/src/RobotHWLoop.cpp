// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include <ros/ros.h>
#include "robot_hw/RobotHWLoop.h"

namespace hw {
// Constructor for the RobotHWLoop class
RobotHWLoop::RobotHWLoop(ros::NodeHandle &nh, ros::NodeHandle &robot_hw_nh, std::shared_ptr<RobotHW> hardware_interface)
    : hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), robot_hw_nh));

  // Load ROS parameters
  int error = 0;

  loopHz_ = nh.param<double>("/robot_hw/loop_frequency", 500);
  cycleTimeErrorThreshold_ = nh.param<double>("/robot_hw/cycle_time_error_threshold", 0.002);

  ROS_INFO("Load param:\nloop_frequency: %f, \ncycleTimeErrorThreshold: %f",
            loopHz_, cycleTimeErrorThreshold_);

  // Get current time for use with first update
  lastTime_ = Clock::now();

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      Update();
    } 
  });
}

// Update function for the hardware loop
void RobotHWLoop::Update() {
  const auto currentTime = Clock::now();
  // Compute desired duration rounded to clock decimation
  const Duration desiredDuration(1.0 / loopHz_);

  // Get change in time
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_)
  {
    ROS_WARN("Cycle time exceeded error threshold by: %fs, cycle time: %fs, threshold: %fs",
              cycle_time_error - cycleTimeErrorThreshold_, elapsedTime_.toSec(), cycleTimeErrorThreshold_);
  }

  // Input
  // Get the hardware's state
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // Control
  // Let the controller compute the new command (via the controller manager)
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // Output
  // Send the new command to hardware
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // Sleep
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

// Destructor for the RobotHWLoop class
RobotHWLoop::~RobotHWLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}
} // namespace hw
