# 训练结果部署



## 1. 部署环境配置

- 安装ROS Noetic：我们推荐在Ubuntu 20.04操作系统上建立基于ROS Noetic的算法开发环境。ROS提供了一系列工具和库，如核心库、通信库和仿真工具（如Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash终端输入以下Shell命令，安装开发环境所依赖的库：

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

    

- 安装onnxruntime依赖，下载连接：https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0  。请您根据自己的操作系统和平台选择合适版本下载。如在Ubuntu 20.04 x86_64，请按下面步骤进行安装：
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. 创建工作空间

可以按照以下步骤，创建一个RL部署开发工作空间：

- 打开一个Bash终端。

- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：
    ```Bash
    mkdir -p ~/limx_ws/src
    ```
    
- 下载运动控制开发接口：
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
    ```
    
- 下载Gazebo仿真器：
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
    ```
    
- 下载机器人模型描述文件
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-description.git
    ```
    
- 下载可视化工具
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-visualization.git
    ```
    
- 下载RL部署源码：
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/rl-deploy-ros-cpp.git
    ```
    
- 编译工程：
    ```Bash
    cd ~/limx_ws
    catkin_make install
    ```

- 选择机器人类型

  - 通过 Shell 命令 `tree -L 1 src/robot-description/pointfoot ` 列出可用的机器人类型：
  
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
  
  - 以`PF_P441C`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：
  
    ```
    echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
    ```
  
- 运行仿真

  通过运行Shell命令启动Gazebo仿真器，然后在仿真器窗口中按 `Ctrl + Shift + R`，机器人将开始移动。您还可以通过将 `Robot Steering` 插件的发布主题设置为 `/cmd_vel` 来控制机器人的行走。
  
  
  ```
  source install/setup.bash
  roslaunch robot_hw pointfoot_hw_sim.launch
  ```
  ![](doc/simulator.gif)

