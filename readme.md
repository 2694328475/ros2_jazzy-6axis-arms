# AR3 ROS2 智能遥操作机械臂控制系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)

本项目实现了一套基于 **ROS 2 Jazzy** 和 **Teensy 4.1** 的六轴协作机械臂智能控制系统，集成 **MoveIt 2** 运动规划、**ros2_control** 硬件抽象层，以及 **CAN 总线力控夹爪**。采用 **MIT 许可证** 开源，旨在为低成本、高灵活性的智能机械臂控制提供可复用的解决方案。

---

## 📌 主要特性

- ✅ **ROS 2 + MoveIt 2 完整控制链路**：从轨迹规划到电机脉冲生成的闭环控制
- ✅ **Teensy 4.1 高性能固件**：非阻塞梯形加减速、多轴任务队列、自定义串口协议（支持 `RJ`/`RP`/`HOME` 等命令）
- ✅ **ros2_control 硬件接口插件**：严格遵循生命周期管理，提供精确的关节状态反馈与指令下发（50 Hz）
- ✅ **CAN 总线力控夹爪**：抗积分饱和 PID 力矩控制，支持实时调节夹持力
- ✅ **Gazebo 仿真支持**：完整的仿真环境（含视觉插件与夹爪模型），可用于算法验证

---

## 🧱 系统架构

| 层级 | 组件 | 说明 |
|------|------|------|
| 规划层 | MoveIt 2 | 运动规划、避障、逆运动学求解 |
| 控制层 | `joint_trajectory_controller` | 轨迹跟踪、插值、指令下发 |
| 驱动层 | `AR3HardwareInterface` (ros2_control 插件) | 弧度/步数转换、限位裁剪、串口通信 |
| 执行层 | Teensy 4.1 固件 | 脉冲生成、限位检测、安全中断、CAN 夹爪管理 |

---

## 📦 软件包说明

| 包名 | 功能 |
|------|------|
| `ar3_description` | URDF/Xacro 模型描述 |
| `ar3_hardware_interface` | ros2_control 硬件插件，实现 `SystemInterface` |
| `teensy4.1` | Teensy 4.1 固件（Arduino 平台）|
| `ar3_moveit_config` | MoveIt 2 配置文件 |
| `ar3_gazebo` | Gazebo 仿真 |
| `teensy4.1` | 下位机代码 |
---

## 🚀 快速开始

### 1. 环境要求

- **操作系统**：Ubuntu 24.04
- **ROS 2 发行版**：Jazzy
- **硬件**：
  - AR3 六轴机械臂（开环步进电机版）
  - Teensy 4.1 开发板
  - ASTRA PRO 深度相机（或任何 ROS 2 支持的 RGB-D 相机）
  - M2006 电机 + C610 电调（力控夹爪）
- **依赖**：
  ```bash
  sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit
  sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-tools

### 2. 构建工作空间

mkdir -p ~/ar3_ws/src
cd ~/ar3_ws/src
git clone https://github.com/yourname/ar3_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

### 3. 烧录固件

使用 Arduino IDE 或 PlatformIO 打开 teensy4.1/teensy4.1.ino
选择开发板 Teensy 4.1，编译并上传。
确保串口设备为 /dev/ttyACM0

### 4. 启动真实机械臂控制
ros2 launch ar3_hd_moveit_config demo.launch.py

### 5. gazebo仿真模式
ros2 launch ar3_gazebo ar3_gazebo_bringup.launch.py
ros2 launch ar3_moveit_config demo.launch.py use_sim_time:=true

### 6. 直接通过ros2_control发送指令
先修改ar3_description/urdf/ar3.urdf.xacro文件，将ros2_control configuration部分取消注释
再在终端使用 ros2 launch ar3_bringup ar3_trajectory.launch.py
