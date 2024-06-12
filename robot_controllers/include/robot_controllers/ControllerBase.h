// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_CONTROLLER_BASE_H_
#define _LIMX_CONTROLLER_BASE_H_

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <robot_common/hardware_interface/ContactSensorInterface.h>
#include <robot_common/hardware_interface/HybridJointInterface.h>
#include <std_msgs/Float32MultiArray.h>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>

namespace robot_controller {

// Define scalar and vector types for Eigen
using scalar_t = double;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

// Utility class to measure time intervals
class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

// Structure to hold robot configuration settings
struct RobotCfg {
  // Control configuration settings
  struct ControlCfg {
    float stiffness{0.0};            // Stiffness parameter
    float damping{0.0};              // Damping parameter
    float action_scale_pos{0.0};     // Scaling factor for position action
    float action_scale_vel{0.0};     // Scaling factor for velocity action
    int decimation{0};             // Decimation factor
    float user_torque_limit{0.0};    // User-defined torque limit

    // Print control configuration settings
    void print() {
      ROS_INFO_STREAM("=======start ObsScales========");
      ROS_INFO_STREAM("stiffness: " << stiffness);
      ROS_INFO_STREAM("damping: " << damping);
      ROS_INFO_STREAM("action_scale_pos: " << action_scale_pos);
      ROS_INFO_STREAM("action_scale_vel: " << action_scale_vel);
      ROS_INFO_STREAM("decimation: " << decimation);
      ROS_INFO_STREAM("=======end ObsScales========\n");
    }
  };

  // Reinforcement learning configuration settings
  struct RlCfg {
    // Observation scaling parameters
    struct ObsScales {
      scalar_t linVel{0.0};            // Linear velocity scaling
      scalar_t angVel{0.0};            // Angular velocity scaling
      scalar_t dofPos{0.0};            // Degree of freedom position scaling
      scalar_t dofVel{0.0};            // Degree of freedom velocity scaling

      // Print observation scaling parameters
      void print() {
        ROS_INFO_STREAM("=======start ObsScales========");
        ROS_INFO_STREAM("linVel: " << linVel);
        ROS_INFO_STREAM("angVel: " << angVel);
        ROS_INFO_STREAM("dofPos: " << dofPos);
        ROS_INFO_STREAM("dofVel: " << dofVel);
        ROS_INFO_STREAM("=======end ObsScales========\n");
      }
    };

    scalar_t clipActions{0.0};       // Action clipping parameter
    scalar_t clipObs{0.0};           // Observation clipping parameter
    ObsScales obsScales;        // Observation scaling settings
  };

  // User command configuration settings
  struct UserCmdCfg {
    double linVel_x{0.0}; 
    double linVel_y{0.0}; 
    double angVel_yaw{0.0}; 

    // Print user command scaling parameters
    void print() {
      ROS_INFO_STREAM("=======Start User Cmd Scales========");
      ROS_INFO_STREAM("lin_vel_x: " << linVel_x);
      ROS_INFO_STREAM("lin_vel_y: " << linVel_y);
      ROS_INFO_STREAM("ang_vel_yaw: " << angVel_yaw);
      ROS_INFO_STREAM("=======End User Cmd Scales========\n");
    }
  };

  RlCfg rlCfg;                   // RL configuration settings
  UserCmdCfg userCmdCfg;         // User command configuration settings
  std::map<std::string, double> initState;  // Initial state settings
  ControlCfg controlCfg;         // Control configuration settings

  // Print robot configuration settings
  void print() {
    rlCfg.obsScales.print();
    controlCfg.print();
    userCmdCfg.print();
    ROS_INFO_STREAM("clipActions: " << rlCfg.clipActions);
    ROS_INFO_STREAM("clipObs: " << rlCfg.clipObs);
  }
};

// Base class for controllers
class ControllerBase
  : public controller_interface::MultiInterfaceController<robot_common::HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                            robot_common::ContactSensorInterface> {
public:
  enum class Mode : uint8_t; // Enumeration for controller modes

  ControllerBase() = default; // Default constructor

  virtual ~ControllerBase() = default; // Virtual destructor

  // Initialize the controller
  virtual bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh);

  // Perform actions when the controller starts
  virtual void starting(const ros::Time &time) {}

  // Update the controller
  virtual void update(const ros::Time &time, const ros::Duration &period) {}

  // Load the model for the controller
  virtual bool loadModel() { return false; }

  // Load RL configuration settings
  virtual bool loadRLCfg() { return false; }

  // Compute actions for the controller
  virtual void computeActions() {}

  // Compute observations for the controller
  virtual void computeObservation() {}

  // Display joint information
  virtual void showJointInfo() {}

  // Handle stand mode
  virtual void handleStandMode() {}

  // Handle walk mode
  virtual void handleWalkMode() {}

protected:
  // Callback function for velocity commands
  virtual void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg){};

  // Get the robot configuration
  virtual RobotCfg &getRobotCfg() { return robotCfg_; }

  scalar_t loopFrequency_; //Control Frequency

  Mode mode_;               // Controller mode
  int64_t loopCount_;       // Loop count
  vector3_t commands_;      // Command vector
  vector3_t scaled_commands_; // Scaled command vector
  RobotCfg robotCfg_{};     // Robot configuration

  std::vector<std::string> jointNames_;          // Joint names
  vector_t measuredRbdState_;                    // Measured RBD state
  // Hardware interface
  std::vector<robot_common::HybridJointHandle> hybridJointHandles_; // Hybrid joint handles
  hardware_interface::ImuSensorHandle imuSensorHandles_;            // IMU sensor handles
  std::vector<robot_common::ContactSensorHandle> contactHandles_;   // Contact sensor handles

  vector_t defaultJointAngles_; // Default joint angles
  vector_t initJointAngles_;    // Initial joint angles in standard standing pose

  scalar_t standPercent_;       // Standing percent
  scalar_t standDuration_;      // Standing duration

  ros::NodeHandle nh_;          // ROS node handle
  ros::Subscriber cmdVelSub_;   // Command velocity subscriber
};

// Function to compute square of a value
template <typename T>
T square(T a) {
  return a * a;
}

// Function to convert quaternion to ZYX Euler angles
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

// Function to compute rotation matrix from ZYX Euler angles
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // Construct rotation matrix
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                    -s2,          c2 * s3,                   c2 * c3;
  return rotationMatrix;
}
} // namespace

#endif //_LIMX_CONTROLLER_BASE_H_