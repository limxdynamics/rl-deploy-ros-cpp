// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

/*
 * This file contains the implementation of the PointfootHW class, which is a hardware interface
 * for controlling a legged robot with point foot contacts. It utilizes ROS (Robot Operating System)
 * for communication and control.
 */

#include "robot_hw/PointfootHW.h"

namespace hw {
static const std::string CONTROLLER_NAME = "/controllers/pointfoot_controller";

// Method to start the biped controller
bool PointfootHW::startBipedController() {
  controller_manager_msgs::ListControllers list_controllers;
  if (!list_controllers_client_.call(list_controllers)) {
      ROS_ERROR("Failed to call list controllers service.");
      return false;
  }

  for (const auto& controller : list_controllers.response.controller) {
    if (controller.name == controller_name_ && controller.state == "running") {
      ROS_WARN("Controller %s is already running, skipping start.", controller.name.c_str());
      return false;
    }
  }

  // Creating a message to switch controllers
  controller_manager_msgs::SwitchController sw;
  sw.request.start_controllers.push_back(controller_name_);
  sw.request.start_asap = false;
  sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  sw.request.timeout = ros::Duration(3.0).toSec();

  // Calling the controller_manager service to switch controllers
  if (switch_controllers_client_.call(sw.request, sw.response)) {
    if (sw.response.ok) {
      ROS_INFO("Start controller %s successfully.", sw.request.start_controllers[0].c_str());
      return true;
    } else {
      ROS_WARN("Start controller %s failed.", sw.request.start_controllers[0].c_str());
    }
  } else {
    ROS_WARN("Failed to start controller %s.", sw.request.start_controllers[0].c_str());
  }
  return false;
}

// Method to stop the biped controller
bool PointfootHW::stopBipedController() {
  // Creating a message to switch controllers
  controller_manager_msgs::SwitchController sw;
  sw.request.stop_controllers.push_back(controller_name_);
  sw.request.start_asap = false;
  sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  sw.request.timeout = ros::Duration(3.0).toSec();

  // Calling the controller_manager service to switch controllers
  if (switch_controllers_client_.call(sw.request, sw.response)) {
    if (sw.response.ok) {
      ROS_INFO("Stop controller %s successfully.", sw.request.stop_controllers[0].c_str());
    } else {
      ROS_WARN("Stop controller %s failed.", sw.request.stop_controllers[0].c_str());
    }
  } else {
    ROS_WARN("Failed to stop controller %s.", sw.request.stop_controllers[0].c_str());
  }

  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    robotCmd_.q[i] = jointData_[i].posDes_ = 0.0;
    robotCmd_.dq[i] = jointData_[i].velDes_ = 0.0;
    robotCmd_.Kp[i] = jointData_[i].kp_ = 0.0;
    robotCmd_.tau[i] = jointData_[i].tau_ff_ = 0.0;
    robotCmd_.Kd[i] = jointData_[i].kd_ = 1.0;
  }
  robot_->publishRobotCmd(robotCmd_);

  std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));

  return true;
}

// Method to initialize the hardware interface
bool PointfootHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Initializing the legged robot instance
  robot_ = limxsdk::PointFoot::getInstance();
  
  const char* value = ::getenv("ROBOT_TYPE");
  if (value && strlen(value) > 0) {
    robot_type_ = std::string(value);
  } else {
    ROS_ERROR("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.");
    abort();
  }
  // Determine the specific robot configuration based on the robot type
  is_point_foot_ = (robot_type_.find("PF") != std::string::npos);
  is_wheel_foot_ = (robot_type_.find("WF") != std::string::npos);
  is_sole_foot_  = (robot_type_.find("SF") != std::string::npos);

  if (is_point_foot_)
  {
    controller_name_ = "/controllers/pointfoot_controller";
  }
  if (is_wheel_foot_)
  {
    controller_name_ = "/controllers/wheelfoot_controller";
  }
  if (is_sole_foot_)
  {
    controller_name_ = "/controllers/solefoot_controller";
  }

  // Initializing the RobotHW base class
  if (!RobotHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // Resize the jointData_ vector to match the number of motors in the robot
  jointData_.resize(robot_->getMotorNumber());

  // Initializing robot command instance, state buffer and imu buffer
  robotCmd_ = limxsdk::RobotCmd(robot_->getMotorNumber());
  robotstate_buffer_.writeFromNonRT(limxsdk::RobotState(robot_->getMotorNumber()));
  imudata_buffer_.writeFromNonRT(limxsdk::ImuData());

  // Subscribing to robot state
  robot_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& msg) {
    robotstate_buffer_.writeFromNonRT(*msg);
  });

  // Subscribing to robot imu
  robot_->subscribeImuData([this](const limxsdk::ImuDataConstPtr& msg) {
    imudata_buffer_.writeFromNonRT(*msg);
  });

  // Retrieving joystick configuration parameters
  root_nh.getParam("/joystick_buttons", joystick_btn_map_);
  for (auto button: joystick_btn_map_) {
    ROS_INFO_STREAM("Joystick button: [" << button.first << ", " << button.second << "]");
  }

  root_nh.getParam("/joystick_axes", joystick_axes_map_);
  for (auto axes: joystick_axes_map_) {
    ROS_INFO_STREAM("Joystick axes: [" << axes.first << ", " << axes.second << "]");
  }

  // When deploying on the real machine, this part receives data from the robot controller and processes it.
  // You can customize it according to your needs.
  robot_->subscribeSensorJoy([this](const limxsdk::SensorJoyConstPtr& msg) {
    // Logic for starting biped controller
    if (calibration_state_ == 0 && joystick_btn_map_.count("L1") > 0 && joystick_btn_map_.count("Y") > 0) {
      if (msg->buttons[joystick_btn_map_["L1"]] == 1 && msg->buttons[joystick_btn_map_["Y"]] == 1) {
        startBipedController();
      }
    }

    // Logic for stopping biped controller
    if (joystick_btn_map_.count("L1") > 0 && joystick_btn_map_.count("X") > 0) {
      if (msg->buttons[joystick_btn_map_["L1"]] == 1 && msg->buttons[joystick_btn_map_["X"]] == 1) {
        ROS_FATAL("L1 + X stopping controller!");
        stopBipedController();
        abort();
      }
    }

    // Publishing cmd_vel based on joystick input
    if (joystick_axes_map_.count("left_horizon") > 0 && joystick_axes_map_.count("left_vertical") > 0
      && joystick_axes_map_.count("right_horizon") > 0 && joystick_axes_map_.count("right_vertical") > 0) {
      static ros::Time lastpub;
      ros::Time now = ros::Time::now();
      if (fabs(now.toSec() - lastpub.toSec()) >= (1.0 / 30)) {
        geometry_msgs::Twist twist;
        twist.linear.x = msg->axes[joystick_axes_map_["left_vertical"]] * 0.5;
        twist.linear.y = msg->axes[joystick_axes_map_["left_horizon"]] * 0.5;
        twist.angular.z = msg->axes[joystick_axes_map_["right_horizon"]] * 0.5;
        cmd_vel_pub_.publish(twist);
        lastpub = now;
      }
    }
  });

  /*
   * Subscribing to diagnostic values for calibration state
   */
  robot_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {
    // Check if the diagnostic message pertains to calibration
    if (msg->name == "calibration") {
      ROS_WARN("Calibration state: %d, msg: %s", msg->code, msg->message.c_str());
      calibration_state_ = msg->code;
    }
    
    // Check if the diagnostic message pertains to EtherCAT
    if (msg->name == "ethercat" && msg->level == limxsdk::DiagnosticValue::ERROR) {
      ROS_FATAL("Ethercat code: %d, msg: %s", msg->code, msg->message.c_str());
      stopBipedController();
      abort();
    }

    // Check if the diagnostic message pertains to IMU
    if (msg->name == "imu" && msg->level == limxsdk::DiagnosticValue::ERROR) {
      ROS_FATAL("IMU code: %d, msg: %s", msg->code, msg->message.c_str());
      stopBipedController();
      abort();
    }
  });

  // Advertising cmd_vel topic for publishing twist commands
  cmd_vel_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Initializing ROS service clients for controller
  switch_controllers_client_ = robot_hw_nh.serviceClient<controller_manager_msgs::SwitchController>("/pointfoot_hw/controller_manager/switch_controller");
  list_controllers_client_ = robot_hw_nh.serviceClient<controller_manager_msgs::ListControllers>("/pointfoot_hw/controller_manager/list_controllers");

  // Setting up joints, IMU, and contact sensors
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  return true;
}

// Method to read data from hardware
void PointfootHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Reading robot state
  limxsdk::RobotState robotstate = *robotstate_buffer_.readFromRT();
  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    jointData_[i].pos_ = robotstate.q[i];
    jointData_[i].vel_ = robotstate.dq[i];
    jointData_[i].tau_ = robotstate.tau[i];
  }
  // Reading imu data
  limxsdk::ImuData imudata = *imudata_buffer_.readFromRT();
  imuData_.ori_[0] = imudata.quat[1];
  imuData_.ori_[1] = imudata.quat[2];
  imuData_.ori_[2] = imudata.quat[3];
  imuData_.ori_[3] = imudata.quat[0];
  imuData_.angularVel_[0] = imudata.gyro[0];
  imuData_.angularVel_[1] = imudata.gyro[1];
  imuData_.angularVel_[2] = imudata.gyro[2];
  imuData_.linearAcc_[0] = imudata.acc[0];
  imuData_.linearAcc_[1] = imudata.acc[1];
  imuData_.linearAcc_[2] = imudata.acc[2];
}

// Method to write commands to hardware
void PointfootHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Writing commands to robot
  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    robotCmd_.q[i] = static_cast<float>(jointData_[i].posDes_);
    robotCmd_.dq[i] = static_cast<float>(jointData_[i].velDes_);
    robotCmd_.Kp[i] = static_cast<float>(jointData_[i].kp_);
    robotCmd_.Kd[i] = static_cast<float>(jointData_[i].kd_);
    robotCmd_.tau[i] = static_cast<float>(jointData_[i].tau_ff_);
    robotCmd_.mode[i] = static_cast<float>(jointData_[i].mode_);
  }

  // Publishing robot commands
  if (calibration_state_ == 0) {
    robot_->publishRobotCmd(robotCmd_);
  }
}

// Method to setup joints based on URDF
bool PointfootHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index;
    int joint_index;
    if (joint.first.find("L_") != std::string::npos)
      leg_index = 0;
    else if (joint.first.find("R_") != std::string::npos)
      leg_index = 1;
    else
      continue;

    if (joint.first.find("abad") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("hip") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("knee") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("wheel") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("ankle") != std::string::npos)
      joint_index = 3;
    else
      continue;

    int index = leg_index * robot_->getMotorNumber() / 2 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(robot_common::HybridJointHandle(state_handle, &jointData_[index].posDes_,
                                                                         &jointData_[index].velDes_,
                                                                         &jointData_[index].kp_, &jointData_[index].kd_,
                                                                         &jointData_[index].tau_ff_, &jointData_[index].mode_));
  }

  return true;
}

// Method to setup IMU sensor
bool PointfootHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("limx_imu", "limx_imu", imuData_.ori_,
                                                                           imuData_.oriCov_, imuData_.angularVel_,
                                                                           imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                           imuData_.linearAccCov_));
  return true;
}

// Method to setup contact sensors
bool PointfootHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("/robot_hw/contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(robot_common::ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

// Method to load URDF model
bool PointfootHW::loadUrdf(ros::NodeHandle& nh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // Getting the URDF parameter from the parameter server
  nh.getParam("robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace hw
