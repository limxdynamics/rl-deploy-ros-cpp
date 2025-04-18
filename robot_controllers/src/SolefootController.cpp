// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/SolefootController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

namespace robot_controller {

// Initialize the controller
bool SolefootController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  return ControllerBase::init(robot_hw, nh);
}

// Perform initialization when the controller starts
void SolefootController::starting(const ros::Time &time) {
  for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
    ROS_INFO_STREAM("starting hybridJointHandle: " << hybridJointHandles_[i].getPosition());
    defaultJointAngles_[i] = hybridJointHandles_[i].getPosition();
  }

  standPercent_ += 1 / (standDuration_ * loopFrequency_);

  loopCount_ = 0;

  mode_ = Mode::STAND;
}

// Update function called periodically
void SolefootController::update(const ros::Time &time, const ros::Duration &period) {
  switch (mode_) {
    case Mode::STAND:
      handleStandMode();
      break;
    case Mode::WALK:
      handleWalkMode();
      break;
  }

  loopCount_++;
}

// Handle walking mode
void SolefootController::handleWalkMode() {
  TicToc dida;
  // Compute observation & actions
  if (robotCfg_.controlCfg.decimation == 0) {
    ROS_ERROR("Error robotCfg_.controlCfg.decimation");
    return;
  }
  if (loopCount_ % robotCfg_.controlCfg.decimation == 0) {
    computeObservation();
    computeEncoder();
    computeActions();

    // Limit action range
    scalar_t actionMin = -robotCfg_.rlCfg.clipActions;
    scalar_t actionMax = robotCfg_.rlCfg.clipActions;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                   [actionMin, actionMax](scalar_t x) { return std::max(actionMin, std::min(actionMax, x)); });
  }

  // Set action
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  // in wheel mode, wheel joint actions differ from others
  for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
    if ((i + 1) % 4 != 0) {
      scalar_t actionMin =
          jointPos(i) - initJointAngles_(i, 0) +
          (robotCfg_.controlCfg.damping * jointVel(i) - robotCfg_.controlCfg.user_torque_limit) / robotCfg_.controlCfg.stiffness;
      scalar_t actionMax =
          jointPos(i) - initJointAngles_(i, 0) +
          (robotCfg_.controlCfg.damping * jointVel(i) + robotCfg_.controlCfg.user_torque_limit) / robotCfg_.controlCfg.stiffness;
      actions_[i] = std::max(actionMin / robotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / robotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
      scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.action_scale_pos + initJointAngles_(i, 0);
      hybridJointHandles_[i].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness, robotCfg_.controlCfg.damping,
                                        0, 2);
      lastActions_(i, 0) = actions_[i];
    } else {
      scalar_t actionMin = (jointVel(i) - ankleJointTorqueLimit_ / ankleJointDamping_);
      scalar_t actionMax = (jointVel(i) + ankleJointTorqueLimit_ / ankleJointDamping_);
      lastActions_(i, 0) = actions_[i];
      actions_[i] = std::max(actionMin / ankleJointDamping_,
                    std::min(actionMax / ankleJointDamping_, (scalar_t) actions_[i]));
      scalar_t velocity_des = actions_[i] * ankleJointDamping_;
      hybridJointHandles_[i].setCommand(0, velocity_des, 0, ankleJointDamping_, 0, 0);
    }
  }
}

// Handle standing mode
void SolefootController::handleStandMode() {
  if (standPercent_ < 1) {
    for (int j = 0; j < hybridJointHandles_.size(); j++) {
      scalar_t pos_des = defaultJointAngles_[j] * (1 - standPercent_) + initJointAngles_[j] * standPercent_;
      hybridJointHandles_[j].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness,
                                        robotCfg_.controlCfg.damping, 0, 2);
    }
    standPercent_ += 1 / (standDuration_ * loopFrequency_);
  } else {
    mode_ = Mode::WALK;
  }
}

bool SolefootController::loadModel() {
  // Load ONNX models for policy, encoder, and gait generator.

  std::string policyModelPath;
  if (!nh_.getParam("/policyFile", policyModelPath)) {
    ROS_ERROR("Failed to retrieve policy path from the parameter server!");
    return false;
  }

  std::string encoderModelPath;
  if (!nh_.getParam("/encoderFile", encoderModelPath)) {
    ROS_ERROR("Failed to retrieve encoder path from the parameter server!");
    return false;
  }

  // Create ONNX environment
  onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "PointFootOnnxController"));

  // Create session options
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);

  Ort::AllocatorWithDefaultOptions allocator;

  // Policy session
  ROS_INFO("Loading policy from: %s", policyModelPath.c_str());
  policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyModelPath.c_str(), sessionOptions);
  policyInputNames_.clear();
  policyOutputNames_.clear();
  policyInputShapes_.clear();
  policyOutputShapes_.clear();
  for (size_t i = 0; i < policySessionPtr_->GetInputCount(); i++) {
    policyInputNames_.push_back(policySessionPtr_->GetInputName(i, allocator));
    policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    ROS_INFO("GetInputName: %s", policySessionPtr_->GetInputName(i, allocator));
    std::vector<int64_t> shape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    ROS_INFO("Shape: [%s]", shapeString.c_str());
  }
  for (size_t i = 0; i < policySessionPtr_->GetOutputCount(); i++) {
    policyOutputNames_.push_back(policySessionPtr_->GetOutputName(i, allocator));
    ROS_INFO("GetOutputName: %s", policySessionPtr_->GetOutputName(i, allocator));
    policyOutputShapes_.push_back(policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    ROS_INFO("Shape: [%s]", shapeString.c_str());
  }

  // Encoder session
  ROS_INFO("Loading encoder from: %s", encoderModelPath.c_str());
  encoderSessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, encoderModelPath.c_str(), sessionOptions);
  encoderInputNames_.clear();
  encoderOutputNames_.clear();
  encoderInputShapes_.clear();
  encoderOutputShapes_.clear();
  for (size_t i = 0; i < encoderSessionPtr_->GetInputCount(); i++) {
    encoderInputNames_.push_back(encoderSessionPtr_->GetInputName(i, allocator));
    encoderInputShapes_.push_back(encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    ROS_INFO("GetInputName: %s", encoderSessionPtr_->GetInputName(i, allocator));
    std::vector<int64_t> shape = encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    ROS_INFO("Shape: [%s]", shapeString.c_str());
  }
  for (size_t i = 0; i < encoderSessionPtr_->GetOutputCount(); i++) {
    encoderOutputNames_.push_back(encoderSessionPtr_->GetOutputName(i, allocator));
    ROS_INFO("GetOutputName: %s", encoderSessionPtr_->GetOutputName(i, allocator));
    encoderOutputShapes_.push_back(encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    ROS_INFO("Shape: [%s]", shapeString.c_str());
  }

  ROS_INFO("Successfully loaded ONNX models!");
  return true;
}

// Loads the reinforcement learning configuration.
bool SolefootController::loadRLCfg() {
  auto &initState = robotCfg_.initState;
  SoleBipedRobotCfg::ControlCfg &controlCfg = robotCfg_.controlCfg;
  SoleBipedRobotCfg::RlCfg::ObsScales &obsScales = robotCfg_.rlCfg.obsScales;

  try {
    // Load parameters from ROS parameter server.
    int error = 0;
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/joint_names", jointNames_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/abad_L_Joint", initState["abad_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/hip_L_Joint", initState["hip_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/knee_L_Joint", initState["knee_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/abad_R_Joint", initState["abad_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/hip_R_Joint", initState["hip_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/knee_R_Joint", initState["knee_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/stand_mode/stand_duration", standDuration_));
    error += static_cast<int>(!nh_.getParam("/robot_hw/loop_frequency", loopFrequency_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/stiffness", controlCfg.stiffness));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/damping", controlCfg.damping));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/action_scale_pos", controlCfg.action_scale_pos));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/decimation", controlCfg.decimation));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/user_torque_limit", controlCfg.user_torque_limit));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/clip_scales/clip_observations", robotCfg_.rlCfg.clipObs));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/clip_scales/clip_actions", robotCfg_.rlCfg.clipActions));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/observations_size", observationSize_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/obs_history_length", obsHistoryLength_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/encoder_output_size", encoderOutputSize_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/commands_size", commandSizeSolefoot_));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imu_orientation_offset/yaw", imu_orientation_offset[0]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imu_orientation_offset/pitch", imu_orientation_offset[1]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imu_orientation_offset/roll", imu_orientation_offset[2]));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/lin_vel_x", robotCfg_.userCmdCfg.linVel_x));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/lin_vel_y", robotCfg_.userCmdCfg.linVel_y));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/ang_vel_yaw", robotCfg_.userCmdCfg.angVel_yaw));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/ankle_L_Joint", initState["ankle_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/ankle_R_Joint", initState["ankle_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/ankle_joint_damping", ankleJointDamping_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/ankle_joint_torque_limit", ankleJointTorqueLimit_));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/gait/frequencies", robotCfg_.gaitCfg.frequencies));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/gait/swing_height", robotCfg_.gaitCfg.swing_height));
    
    if (error) {
      ROS_ERROR("Load parameters from ROS parameter server error!!!");
    }
    if (standDuration_ <= 0.5) {
      ROS_ERROR("standDuration_ must be larger than 0.5!!!");
    }

    encoderInputSize_ = obsHistoryLength_ * observationSize_;
    robotCfg_.print();

    // Resize vectors.
    actions_.resize(actionsSize_);
    observations_.resize(observationSize_);
    proprioHistoryVector_.resize(observationSize_ * obsHistoryLength_);
    encoderOut_.resize(encoderOutputSize_);
    lastActions_.resize(actionsSize_);
    commandSolefoot_.resize(commandSizeSolefoot_);
    scaledCommandSolefoot_.resize(commandSizeSolefoot_);

    // Initialize vectors.
    lastActions_.setZero();
    commandSolefoot_.setZero();
    scaledCommandSolefoot_.setZero();
    baseLinVel_.setZero();
    basePosition_.setZero();
  } catch (const std::exception &e) {
    // Error handling.
    ROS_ERROR("Error in the PointfootCfg: %s", e.what());
    return false;
  }

  return true;
}

void SolefootController::computeObservation() {
  // Get IMU orientation
  Eigen::Quaterniond q_wi;
  for (size_t i = 0; i < 4; ++i) {
    q_wi.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
  }
  // Convert quaternion to ZYX Euler angles and calculate inverse rotation matrix
  vector3_t zyx = quatToZyx(q_wi);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

  // Define gravity vector and project it to the body frame
  vector3_t gravityVector(0, 0, -1);
  vector3_t projectedGravity(inverseRot * gravityVector);

  // Get base angular velocity and apply orientation offset
  vector3_t baseAngVel(imuSensorHandles_.getAngularVelocity()[0], imuSensorHandles_.getAngularVelocity()[1],
                       imuSensorHandles_.getAngularVelocity()[2]);
  vector3_t _zyx(imu_orientation_offset[0], imu_orientation_offset[1], imu_orientation_offset[2]);
  matrix_t rot = getRotationMatrixFromZyxEulerAngles(_zyx);
  baseAngVel = rot * baseAngVel;
  projectedGravity = rot * projectedGravity;

  // Get initial state of joints
  auto &initState = robotCfg_.initState;
  vector_t jointPos(initState.size());
  vector_t jointVel(initState.size());
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }

  vector_t gait(4);
  gait << robotCfg_.gaitCfg.frequencies, 0.5, 0.5, robotCfg_.gaitCfg.swing_height; // trot
  gait_index_ += 0.02 * gait(0);
  if (gait_index_ > 1.0)
  {
      gait_index_ = 0.0;
  }
  vector_t gait_clock(2);
  gait_clock << sin(gait_index_ * 2 * M_PI), cos(gait_index_ * 2 * M_PI);

  vector_t actions(lastActions_);

  // Define command scaler and observation vector
  vector_t commandScalerVal(commandSizeSolefoot_);
  commandScalerVal << robotCfg_.userCmdCfg.linVel_x, robotCfg_.userCmdCfg.linVel_y, robotCfg_.userCmdCfg.angVel_yaw; //, 1.0, 1.0;
  // matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(robotCfg_.userCmdCfg.linVel_x,
  //                                                             robotCfg_.userCmdCfg.linVel_y,
  //                                                             robotCfg_.userCmdCfg.angVel_yaw);
  vector_t scaled_commands(commandSizeSolefoot_);
  for (int i = 0; i < commandSizeSolefoot_; i++) {
    scaled_commands(i) = commandSolefoot_(i) * commandScalerVal(i);
  }
  vector_t obs(observationSize_);
  // vector_t scaled_commands = commandScaler * commandSolefoot_;
  // Populate observation vector
  vector_t jointPos_value = (jointPos - initJointAngles_) * robotCfg_.rlCfg.obsScales.dofPos;
  vector_t jointPos_input = jointPos_value;

  obs << baseAngVel * robotCfg_.rlCfg.obsScales.angVel,
      projectedGravity,
      jointPos_input,
      jointVel * robotCfg_.rlCfg.obsScales.dofVel,
      actions,
      gait_clock,
      gait;
  if (isfirstRecObs_)
  {
    int64_t inputSize = std::accumulate(encoderInputShapes_[0].begin(), encoderInputShapes_[0].end(),
                                        static_cast<int64_t>(1), std::multiplies<int64_t>());
    proprioHistoryBuffer_.resize(inputSize);
    for (size_t i = 0; i < obsHistoryLength_; i++)
    {
        proprioHistoryBuffer_.segment(i * observationSize_,
                                        observationSize_) = obs.cast<tensor_element_t>();
    }
    isfirstRecObs_ = false;
  }
  proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) = proprioHistoryBuffer_.tail(
      proprioHistoryBuffer_.size() - observationSize_);
  proprioHistoryBuffer_.tail(observationSize_) = obs.cast<tensor_element_t>();
  // Update observation, scaled commands, and proprioceptive history vector
  for (size_t i = 0; i < obs.size(); i++) {
    observations_[i] = static_cast<tensor_element_t>(obs(i));
  }
  for (size_t i = 0; i < scaledCommandSolefoot_.size(); i++) {
    scaledCommandSolefoot_[i] = static_cast<tensor_element_t>(scaled_commands(i));
  }
  for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++)
  {
      proprioHistoryVector_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_(i));
  }

  // Limit observation range
  scalar_t obsMin = -robotCfg_.rlCfg.clipObs;
  scalar_t obsMax = robotCfg_.rlCfg.clipObs;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                 [obsMin, obsMax](scalar_t x)
                 { return std::max(obsMin, std::min(obsMax, x)); });
}

void SolefootController::computeEncoder() {
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                          OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, proprioHistoryBuffer_.data(),
                                                                    proprioHistoryBuffer_.size(),
                                                                    encoderInputShapes_[0].data(),
                                                                    encoderInputShapes_[0].size()));

  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues =
      encoderSessionPtr_->Run(runOptions, encoderInputNames_.data(), inputValues.data(), 1,
                              encoderOutputNames_.data(), 1);
  for (int i = 0; i < encoderOutputSize_; i++)
  {
      encoderOut_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }

}

// Computes actions using the policy model.
void SolefootController::computeActions() {
  // Create input tensor object
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                          OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  std::vector<tensor_element_t> combined_obs;
  for (const auto &item : encoderOut_)
  {
    combined_obs.push_back(item);
  }
  for (const auto &item : observations_) {
    combined_obs.push_back(item);
  }
  for (int i = 0; i < scaledCommandSolefoot_.size(); i++)
  {
    combined_obs.push_back(scaledCommandSolefoot_[i]);
  }

  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, combined_obs.data(), combined_obs.size(),
                                                  policyInputShapes_[0].data(), policyInputShapes_[0].size()));
  // Run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = policySessionPtr_->Run(runOptions, policyInputNames_.data(),
                                                                inputValues.data(), 1, policyOutputNames_.data(),
                                                                1);

  for (size_t i = 0; i < actionsSize_; i++) {
    actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

// vector_t SolefootController::handleGait() {
//   vector_t gait(4);
//   gait << robotCfg_.gaitCfg.frequencies, 0.5, 0.5, robotCfg_.gaitCfg.swing_height;
//   return gait;
// }

// vector_t SolefootController::handleGaitClock(vector_t &gait) {
//   vector_t gait_clock(2);
//   gait_index_ += 0.02 * gait(0);
//   gait_index_ = gait_index_ > 1.0 ? 0.0 : gait_index_;
//   gait_clock << sin(gait_index_ * 2 * M_PI), cos(gait_index_ * 2 * M_PI);
//   return gait_clock;
// }

void SolefootController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
  // Update the commands with the linear and angular velocities from the Twist message.

  // Set linear x velocity.
  commandSolefoot_(0) = (msg->linear.x > 1.0 ? 1.0 : (msg->linear.x < -1.0 ? -1.0 : msg->linear.x));

  // Set linear y velocity.
  commandSolefoot_(1) = (msg->linear.y > 1.0 ? 1.0 : (msg->linear.y < -1.0 ? -1.0 : msg->linear.y));

  // Set angular z velocity.
  commandSolefoot_(2) = (msg->angular.z > 1.0 ? 1.0 : (msg->angular.z < -1.0 ? -1.0 : msg->angular.z));
  // commandSolefoot_(3) = 0.0;
  // commandSolefoot_(4) = 0.0;
}

} // namespace

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controller::SolefootController, controller_interface::ControllerBase)
