# AR3 ROS2 Intelligent Teleoperation Robotic Arm Control System / AR3 ROS2 智能遥操作机械臂控制系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)

[English](#english) | [中文](#中文)

---

## English

This project implements an intelligent control system for a 6-axis collaborative robotic arm based on **ROS 2 Jazzy** and **Teensy 4.1**, integrating **MoveIt 2** motion planning, the **ros2_control** hardware abstraction layer, and a **CAN bus force-controlled gripper**. Released under the MIT License, it aims to provide a reusable solution for low-cost, highly flexible robotic arm control.

> **Hardware Platform Note**: The mechanical structure of this project is based on the AR3 open-source 6-axis robotic arm. The ROS 2 control stack, Teensy 4.1 firmware, ros2_control hardware interface, CAN force-controlled gripper, and Gazebo simulation are all independently implemented.

### 📌 Key Features

- ✅ **Complete ROS 2 + MoveIt 2 Control Pipeline**: Closed-loop control from trajectory planning to motor pulse generation
- ✅ **High-Performance Teensy 4.1 Firmware**: Non-blocking trapezoidal acceleration/deceleration, multi-axis task queue, custom serial protocol (supporting `RJ`/`RP`/`HOME` commands)
- ✅ **ros2_control Hardware Interface Plugin**: Strict lifecycle management, providing precise joint state feedback and command dispatch (50 Hz)
- ✅ **CAN Bus Force-Controlled Gripper**: Teensy 4.1 + C610 + M2006, 1 kHz control loop, current ramp + speed PI (anti-windup) + stall-detection force control, with travel protection and CAN watchdog
- ✅ **Gazebo Simulation Support**: Complete simulation environment (with vision plugin and gripper model) for algorithm validation

### 🧱 System Architecture

| Layer | Component | Description |
|:---:|:---|:---|
| Planning | MoveIt 2 | Motion planning, obstacle avoidance, inverse kinematics |
| Control | `joint_trajectory_controller` | Trajectory tracking, interpolation, command dispatch |
| Driver | `AR3HardwareInterface` (ros2_control plugin) | Radian/step conversion, limit clipping, serial communication |
| Execution | Teensy 4.1 Firmware | Pulse generation, limit detection, safety interrupt, CAN gripper management |
| Simulation | Gazebo | Vision plugin, gripper model, algorithm validation |

### 🔧 Main Work

#### 1. Complete ROS 2 + MoveIt 2 Control Pipeline

- Established the closed-loop control from **MoveIt 2 trajectory planning → ros2_control → hardware interface → motor pulse generation**
- Decoupled the planning layer from the execution layer, enabling real-time trajectory dispatch to the low-level firmware

#### 2. High-Performance Teensy 4.1 Firmware

- Implemented a **non-blocking trapezoidal acceleration/deceleration** algorithm to ensure smooth multi-axis motion without blocking the main loop
- Designed a **multi-axis task queue** to support parallel motion scheduling of multiple joints
- Custom serial communication protocol supporting `RJ` (joint motion), `RP` (Cartesian pose), `HOME` (homing), and other commands

#### 3. ros2_control Hardware Interface Plugin

- Strictly follows the ros2_control **lifecycle management** (configure → activate → deactivate → cleanup)
- Provides precise **joint state feedback** and **command dispatch**, with a stable control frequency of **50 Hz**

#### 4. CAN Bus Force-Controlled Gripper Firmware

- Based on **Teensy 4.1 + C610 ESC + M2006 motor**, with a 1 kHz control loop
- Implemented **current ramp + speed PI (anti-windup)** to avoid current surges and integral windup
- **Stall-detection force control**: torque reaching 90% of threshold and output shaft speed < 5 rpm, sustained for 150 ms, determines grip completion
- Designed **output shaft travel accumulation protection** (including single-turn angle overflow handling) and a **CAN communication watchdog**
- Custom serial command protocol supporting online adjustment of force threshold, speed, travel, and other parameters

#### 5. Gazebo Simulation Support

- Built a complete Gazebo simulation environment, including a **vision plugin** and **gripper model**
- Enables algorithm validation and testing of trajectory planning and control logic without physical hardware

### 📦 Package Description

| Package | Function |
|:---|:---|
| `ar3_description` | URDF/Xacro model description |
| `ar3_hardware_interface` | ros2_control hardware plugin, implementing `SystemInterface` |
| `teensy4.1` | Teensy 4.1 firmware (Arduino/PlatformIO platform) |
| `ar3_moveit_config` | MoveIt 2 configuration files |
| `ar3_gazebo` | Gazebo simulation |
| `ar3_bringup` | Launch files and trajectory examples |

### 🚀 Quick Start

#### 1. Requirements

- **OS**: Ubuntu 24.04
- **ROS 2 Distribution**: Jazzy
- **Hardware**:
  - AR3 6-axis robotic arm (open-loop stepper motor version)
  - Teensy 4.1 development board
  - ASTRA PRO depth camera (or any ROS 2-supported RGB-D camera)
  - M2006 motor + C610 ESC (force-controlled gripper)

**Dependency Installation:**

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-tools
```

#### 2. Build Workspace

```bash
mkdir -p ~/ar3_ws/src
cd ~/ar3_ws/src
git clone https://github.com/2694328475/ar3_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

#### 3. Flash Firmware

Open `teensy4.1/teensy4.1.ino` with Arduino IDE, select the **Teensy 4.1** board, compile and upload. Ensure the serial device is `/dev/ttyACM0`.
Or open `Teensy_4.1_FreeRTOS` with PlatformIO.

#### 4. Launch Real Robot Control

```bash
ros2 launch ar3_hd_moveit_config demo.launch.py
```

#### 5. Gazebo Simulation Mode

```bash
ros2 launch ar3_gazebo ar3_gazebo_bringup.launch.py
ros2 launch ar3_moveit_config demo.launch.py use_sim_time:=true
```

#### 6. Send Commands Directly via ros2_control

1. Edit `ar3_description/urdf/ar3.urdf.xacro` and uncomment the `ros2_control` configuration section
2. Launch trajectory control:

```bash
ros2 launch ar3_bringup ar3_trajectory.launch.py
```

### 📸 Demo

#### Gazebo Simulation

<img width="480" height="270" alt="Gazebo Simulation" src="docs/gazebo_demo.gif" />


#### ros2_control and Limit Homing

<img width="480" height="270" alt="ros2_control and Limit Homing" src="docs/ros2_control_demo.gif" />


#### Real Robot Motion

<img width="480" height="270" alt="Real Robot Motion" src="docs/real_robot_demo.gif" />


### 📄 License

This project is licensed under the [MIT License](LICENSE).

---

## 中文

本项目实现了一套基于 **ROS 2 Jazzy** 和 **Teensy 4.1** 的六轴协作机械臂智能控制系统，集成 **MoveIt 2** 运动规划、**ros2_control** 硬件抽象层，以及 **CAN 总线力控夹爪**。采用 MIT 许可证开源，旨在为低成本、高灵活性的智能机械臂控制提供可复用的解决方案。

> **硬件平台说明**：本项目的机械结构采用 AR3 开源六轴机械臂。ROS 2 控制栈、Teensy 4.1 固件、ros2_control 硬件接口、CAN 力控夹爪与 Gazebo 仿真均为自主实现。

### 📌 主要特性

- ✅ **ROS 2 + MoveIt 2 完整控制链路**：从轨迹规划到电机脉冲生成的闭环控制
- ✅ **Teensy 4.1 高性能固件**：非阻塞梯形加减速、多轴任务队列、自定义串口协议（支持 `RJ`/`RP`/`HOME` 等命令）
- ✅ **ros2_control 硬件接口插件**：严格遵循生命周期管理，提供精确的关节状态反馈与指令下发（50 Hz）
- ✅ **CAN 总线力控夹爪**：Teensy 4.1 + C610 + M2006，1 kHz 控制循环，电流斜坡 + 速度 PI（抗积分饱和）+ 失速检测力控判定，含行程保护与 CAN 看门狗
- ✅ **Gazebo 仿真支持**：完整的仿真环境（含视觉插件与夹爪模型），可用于算法验证

### 🧱 系统架构

| 层级 | 组件 | 说明 |
|:---:|:---|:---|
| 规划层 | MoveIt 2 | 运动规划、避障、逆运动学求解 |
| 控制层 | `joint_trajectory_controller` | 轨迹跟踪、插值、指令下发 |
| 驱动层 | `AR3HardwareInterface`（ros2_control 插件） | 弧度/步数转换、限位裁剪、串口通信 |
| 执行层 | Teensy 4.1 固件 | 脉冲生成、限位检测、安全中断、CAN 夹爪管理 |
| 仿真层 | Gazebo | 视觉插件、夹爪模型、算法验证 |

### 🔧 主要工作

#### 1. ROS 2 + MoveIt 2 完整控制链路

- 打通 **MoveIt 2 轨迹规划 → ros2_control → 硬件接口 → 电机脉冲生成** 的闭环控制
- 实现规划层与执行层解耦，轨迹可实时下发至底层固件

#### 2. Teensy 4.1 高性能固件

- 实现**非阻塞梯形加减速**算法，保证多轴运动平滑且不阻塞主循环
- 设计**多轴任务队列**，支持多关节并行运动调度
- 自定义串口通信协议，支持 `RJ`（关节运动）、`RP`（笛卡尔位姿）、`HOME`（回零）等命令

#### 3. ros2_control 硬件接口插件

- 严格遵循 ros2_control **生命周期管理**（configure → activate → deactivate → cleanup）
- 提供精确的**关节状态反馈**与**指令下发**，控制频率稳定在 **50 Hz**

#### 4. CAN 总线力控夹爪固件

- 基于 **Teensy 4.1 + C610 电调 + M2006 电机**，1 kHz 控制循环
- 实现**电流斜坡 + 速度 PI（抗积分饱和）**，避免电流突变与积分饱和
- **失速检测力控判定**：力矩达阈值 90% 且输出轴转速 < 5 rpm，维持 150 ms 判定夹取完成
- 设计**输出轴行程累积保护**（含单圈角度溢出处理）与 **CAN 通信看门狗**
- 自定义串口命令协议，支持力阈值、转速、行程等参数在线调节

#### 5. Gazebo 仿真支持

- 搭建完整 Gazebo 仿真环境，包含**视觉插件**与**夹爪模型**
- 可用于算法验证，无需实机即可测试轨迹规划与控制逻辑

### 📦 软件包说明

| 包名 | 功能 |
|:---|:---|
| `ar3_description` | URDF/Xacro 模型描述 |
| `ar3_hardware_interface` | ros2_control 硬件插件，实现 `SystemInterface` |
| `teensy4.1` | Teensy 4.1 固件（Arduino/PlatformIO 平台） |
| `ar3_moveit_config` | MoveIt 2 配置文件 |
| `ar3_gazebo` | Gazebo 仿真 |
| `ar3_bringup` | 启动文件与轨迹示例 |

### 🚀 快速开始

#### 1. 环境要求

- **操作系统**：Ubuntu 24.04
- **ROS 2 发行版**：Jazzy
- **硬件**：
  - AR3 六轴机械臂（开环步进电机版）
  - Teensy 4.1 开发板
  - ASTRA PRO 深度相机（或任何 ROS 2 支持的 RGB-D 相机）
  - M2006 电机 + C610 电调（力控夹爪）

**依赖安装：**

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-tools
```

#### 2. 构建工作空间

```bash
mkdir -p ~/ar3_ws/src
cd ~/ar3_ws/src
git clone https://github.com/2694328475/ar3_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

#### 3. 烧录固件

使用 Arduino IDE 打开 `teensy4.1/teensy4.1.ino`，选择开发板 **Teensy 4.1**，编译并上传。确保串口设备为 `/dev/ttyACM0`。
或使用 PlatformIO 打开 `Teensy_4.1_FreeRTOS`。

#### 4. 启动真实机械臂控制

```bash
ros2 launch ar3_hd_moveit_config demo.launch.py
```

#### 5. Gazebo 仿真模式

```bash
ros2 launch ar3_gazebo ar3_gazebo_bringup.launch.py
ros2 launch ar3_moveit_config demo.launch.py use_sim_time:=true
```

#### 6. 通过 ros2_control 直接发送指令

1. 编辑 `ar3_description/urdf/ar3.urdf.xacro`，取消 `ros2_control` 配置部分的注释
2. 启动轨迹控制：

```bash
ros2 launch ar3_bringup ar3_trajectory.launch.py
```

### 📸 演示

#### Gazebo 仿真

<img width="480" height="270" alt="Gazebo 仿真" src="docs/gazebo_demo.gif" />


#### ros2_control 与限位归零

<img width="480" height="270" alt="ros2_control 与限位归零" src="docs/ros2_control_demo.gif" />


#### 真机运动

<img width="480" height="270" alt="真机运动" src="docs/real_robot_demo.gif" />


### 📄 许可证

本项目采用 [MIT 许可证](LICENSE)。
---
