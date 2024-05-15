// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_HYBRID_JOINT_INTERFACE_H_
#define _LIMX_HYBRID_JOINT_INTERFACE_H_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace robot_common {

// Class defining a hybrid joint handle
class HybridJointHandle : public hardware_interface::JointStateHandle {
 public:
  // Default constructor
  HybridJointHandle() = default;

  // Constructor with initialization
  HybridJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff, uint8_t* mode)
      : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff), mode_(mode) {
    // Check for null pointers
    if (posDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Position desired data pointer is null.");
    }
    if (velDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Velocity desired data pointer is null.");
    }
    if (kp_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
    }
    if (kd_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
    }
    if (ff_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Feedforward data pointer is null.");
    }
  }

  // Setter methods

  // Set the desired position
  void setPositionDesired(double cmd) {
    assert(posDes_);
    *posDes_ = cmd;
  }

  // Set the desired velocity
  void setVelocityDesired(double cmd) {
    assert(velDes_);
    *velDes_ = cmd;
  }

  // Set the proportional gain
  void setKp(double cmd) {
    assert(kp_);
    *kp_ = cmd;
  }

  // Set the derivative gain
  void setKd(double cmd) {
    assert(kd_);
    *kd_ = cmd;
  }

  // Set the feedforward term
  void setFeedforward(double cmd) {
    assert(ff_);
    *ff_ = cmd;
  }

  // Set the control mode
  void setMode(uint8_t cmd) {
    assert(mode_);
    *mode_ = cmd;
  }

  // Method to set all control parameters at once
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff, uint8_t mode) {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
    setMode(mode);
  }

  // Getter methods

  // Get the desired position
  double getPositionDesired() {
    assert(posDes_);
    return *posDes_;
  }

  // Get the desired velocity
  double getVelocityDesired() {
    assert(velDes_);
    return *velDes_;
  }

  // Get the proportional gain
  double getKp() {
    assert(kp_);
    return *kp_;
  }

  // Get the derivative gain
  double getKd() {
    assert(kd_);
    return *kd_;
  }

  // Get the feedforward term
  double getFeedforward() {
    assert(ff_);
    return *ff_;
  }

 private:
  // Pointers to control parameters
  double* posDes_{nullptr}; // Pointer to desired position
  double* velDes_{nullptr}; // Pointer to desired velocity
  double* kp_{nullptr};     // Pointer to proportional gain
  double* kd_{nullptr};     // Pointer to derivative gain
  double* ff_{nullptr};     // Pointer to feedforward term
  uint8_t* mode_{nullptr};  // Pointer to control mode
};

// Class defining a hybrid joint interface
class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

}  // namespace robot_common

#endif // _LIMX_HYBRID_JOINT_INTERFACE_H_