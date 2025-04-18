// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_WHEELFOOT_CONTROLLER_H_
#define _LIMX_WHEELFOOT_CONTROLLER_H_

#include "ros/ros.h"
#include "robot_controllers/ControllerBase.h"
#include "limxsdk/pointfoot.h"

namespace robot_controller {
// Struct for holding configuration settings for a biped robot
struct WheelBipedRobotCfg : public RobotCfg {};

// Class for controlling a biped robot with point foot
class WheelfootController : public ControllerBase {
  using tensor_element_t = float; // Type alias for tensor elements
  using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices

public:
  WheelfootController() = default; // Default constructor

  ~WheelfootController() override = default; // Destructor

  // Enumeration for controller modes
  enum class Mode : uint8_t {
    STAND,  // Stand mode
    WALK,   // Walk mode
  };

  // Initialize the controller
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;

  // Perform actions when the controller starts
  void starting(const ros::Time &time) override;

  // Update the controller
  void update(const ros::Time &time, const ros::Duration &period) override;

protected:
  // Load the model for the controller
  bool loadModel() override;

  // Load RL configuration settings
  bool loadRLCfg() override;

  // Compute observations for the controller
  void computeObservation() override;

  // Compute encoder for the controller
  void computeEncoder() override;

  // Compute actions for the controller
  void computeActions() override;

  // Handle walk mode
  void handleWalkMode() override;

  // Handle stand mode
  void handleStandMode() override;

  // Callback function for command velocity
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) override;

  // Get the robot configuration
  RobotCfg &getRobotCfg() override { return robotCfg_; }

  WheelBipedRobotCfg robotCfg_; // Biped robot configuration

  Mode mode_; // Controller mode

private:
  // File path for policy model
  std::string policyFilePath_;

  std::shared_ptr<Ort::Env> onnxEnvPrt_; // Shared pointer to ONNX environment

  // ONNX session pointers
  std::unique_ptr<Ort::Session> policySessionPtr_;
  std::unique_ptr<Ort::Session> encoderSessionPtr_;

  // Names and shapes of inputs and outputs for ONNX sessions
  std::vector<std::vector<int64_t>> policyInputShapes_;
  std::vector<std::vector<int64_t>> policyOutputShapes_;
  std::vector<const char *> policyInputNames_;
  std::vector<const char *> policyOutputNames_;

  std::vector<std::vector<int64_t>> encoderInputShapes_;
  std::vector<std::vector<int64_t>> encoderOutputShapes_;
  std::vector<const char *> encoderInputNames_;
  std::vector<const char *> encoderOutputNames_;

  std::vector<tensor_element_t> proprioHistoryVector_;
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;

  vector3_t baseLinVel_; // Base linear velocity
  vector3_t basePosition_; // Base position
  vector_t lastActions_; // Last actions

  int actionsSize_; // Size of actions
  int observationSize_; // Size of observations
  int obsHistoryLength_; // Size of history observations
  int encoderInputSize_, encoderOutputSize_; // Input and output size of encoder
  float imu_orientation_offset[3]; // IMU orientation offset
  float wheelJointDamping_, wheelJointTorqueLimit_;
  std::vector<int> jointPosIdxs_;
  std::vector<tensor_element_t> actions_; // Actions
  std::vector<tensor_element_t> observations_; // Observations
  std::vector<tensor_element_t> encoderOut_;  // Encoder
  
  double gait_index_{0.0};
  bool isfirstRecObs_{true};
};

} // namespace robot_controller

#endif //_LIMX_WHEELFOOT_CONTROLLER_H_
