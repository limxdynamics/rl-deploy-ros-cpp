// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_ROBOT_DATA_H_
#define _LIMX_ROBOT_DATA_H_

#include <mutex>
#include <sstream>
#include <string>

namespace hw {

struct MotorData {
  MotorData() {
    // Initialize command variables
    pos_ = vel_ = tau_ = 0.0;
    posDes_ = velDes_ = kp_ = kd_ = tau_ff_ = 0.0;
    mode_ = 0;
  }
  // State variables
  double pos_, vel_, tau_;
  
  // Command variables
  double posDes_, velDes_, kp_, kd_, tau_ff_;
  uint8_t mode_;
};

struct ImuData {
  ImuData() {
    // Initialize orientation
    for (std::size_t i = 0; i < 4; i++) {
      ori_[i] = 0.0;
    }
    // Initialize orientation covariance
    for (std::size_t i = 0; i < 9; i++) {
      oriCov_[i] = 0.0;
    }
    // Initialize angular velocity
    for (std::size_t i = 0; i < 3; i++) {
      angularVel_[i] = 0.0;
    }
    // Initialize angular velocity covariance
    for (std::size_t i = 0; i < 9; i++) {
      angularVelCov_[i] = 0.0;
    }
    // Initialize linear acceleration
    for (std::size_t i = 0; i < 3; i++) {
      linearAcc_[i] = 0.0;
    }
    // Initialize linear acceleration covariance
    for (std::size_t i = 0; i < 9; i++) {
      linearAccCov_[i] = 0.0;
    }
  }

  // Orientation
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  
  // Angular velocity
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  
  // Linear acceleration
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

} // namespace hw

#endif // _LIMX_ROBOT_DATA_H_
