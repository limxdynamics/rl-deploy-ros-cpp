# English | [中文](README.md)
# Deployment of Training Results



## 1. Deployment Environment Configuration

- Install ROS Noetic: Set up a ROS Noetic-based algorithm Development Environment on the Ubuntu 20.04 operating system. For installation, please refer to the documentation: https://docs.ros.org/en/noetic/Installation/Ubuntu-Install-Debians.html, and choose "ros-noetic-desktop" for installation. After the installation of ROS Noetic is completed, enter the following Shell commands in the Bash end point to install the libraries required by the Development Environment:

    ```bash
    sudo apt-get update
    sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-robot-state-* \
                 ros-noetic-joint-state-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- Install the onnxruntime dependency, download link:https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0. Please choose the appropriate version to download according to your operating system and platform. For example, on Ubuntu 20.04 x86_64, please follow the steps below for installation:
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. Create Workspace

You can create an RL deployment development workspace by following these steps:
- Open a Bash end point.
- Create a new directory to store the workspace. For example, you can create a directory named "limx_ws" under the user's home directory:
    ```Bash
    mkdir -p ~/limx_ws/src
    ```
    
- Download the Motion Control Development Interface:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
    ```
    
- Download Gazebo Simulator:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
    ```
    
- Download the robot model description file
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-description.git
    ```
    
- Download Visualization Tool
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-visualization.git
    ```
    
- Download RL deployment source code:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/rl-deploy-ros-cpp.git
    ```
    
- Compile Project:
    ```Bash
    cd ~/limx_ws
    catkin_make install
    ```

- Select robot type

  - List available robot types via the Shell command tree -L 1 src/robot-description/pointfoot : 
  
    ```
    src/robot-description/pointfoot
    ├── PF_P441A
    ├── PF_P441B
    ├── PF_P441C
    ├── PF_P441C2
    ├── PF_TRON1A
    ├── SF_TRON1A
    └── WF_TRON1A
    ```
  
  - TakingPF_P441C (please replace it according to the actual robot type) as an example, set the robot model type:
  
    ```
    echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
    ```
  
- Run Simulation

    Start the Gazebo simulator by running the Shell command, then press `Ctrl + Shift + R` in the simulator window to start the robot moving. You can also control the robot's movement by setting the publish topic of the `Robot Steering` plugin to `/cmd_vel`.
  
  
  ```
  source install/setup.bash
  roslaunch robot_hw pointfoot_hw_sim.launch
  ```
  ![](doc/simulator.gif)

