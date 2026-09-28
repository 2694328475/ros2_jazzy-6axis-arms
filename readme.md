# AR3 ROS2 智能遥操作机械臂控制系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)

本项目实现了一套基于 ROS 2 Jazzy 和 Teensy 4.1 的六轴协作机械臂智能控制系统，集成 MoveIt 2 运动规划、ros2_control 硬件抽象层，以及 CAN 总线力控夹爪。采用 MIT 许可证 开源，旨在为低成本、高灵活性的智能机械臂控制提供可复用的解决方案。

硬件平台说明：本项目的机械结构采用 AR3 开源六轴机械臂。ROS 2 控制栈、Teensy 4.1 固件、ros2_control 硬件接口、CAN 力控夹爪与 Gazebo 仿真均为自主实现。

📌 主要特性
✅ ROS 2 + MoveIt 2 完整控制链路：从轨迹规划到电机脉冲生成的闭环控制

✅ Teensy 4.1 高性能固件：非阻塞梯形加减速、多轴任务队列、自定义串口协议（支持 RJ/RP/HOME 等命令）

✅ ros2_control 硬件接口插件：严格遵循生命周期管理，提供精确的关节状态反馈与指令下发（50 Hz）

✅ CAN 总线力控夹爪：Teensy 4.1 + C610 + M2006，1 kHz 控制循环，电流斜坡 + 速度 PI（抗积分饱和）+ 失速检测力控判定，含行程保护与 CAN 看门狗

✅ Gazebo 仿真支持：完整的仿真环境（含视觉插件与夹爪模型），可用于算法验证

🧱 系统架构
层级	组件	说明
规划层	MoveIt 2	运动规划、避障、逆运动学求解
控制层	joint_trajectory_controller	轨迹跟踪、插值、指令下发
驱动层	AR3HardwareInterface (ros2_control 插件)	弧度/步数转换、限位裁剪、串口通信
执行层	Teensy 4.1 固件	脉冲生成、限位检测、安全中断、CAN 夹爪管理
仿真层	Gazebo	视觉插件、夹爪模型、算法验证
🔧 主要工作
1. ROS 2 + MoveIt 2 完整控制链路
打通 MoveIt 2 轨迹规划 → ros2_control → 硬件接口 → 电机脉冲生成 的闭环控制。

实现规划层与执行层解耦，轨迹可实时下发至底层固件。

2. Teensy 4.1 高性能固件
实现非阻塞梯形加减速算法，保证多轴运动平滑且不阻塞主循环。

设计多轴任务队列，支持多关节并行运动调度。

自定义串口通信协议，支持 RJ（关节运动）、RP（笛卡尔位姿）、HOME（回零）等命令。

3. ros2_control 硬件接口插件
严格遵循 ros2_control 生命周期管理（configure → activate → deactivate → cleanup）。

提供精确的关节状态反馈与指令下发，控制频率稳定在 50 Hz。

4. CAN 总线力控夹爪固件
基于 Teensy 4.1 + C610 电调 + M2006 电机，1 kHz 控制循环。

实现电流斜坡 + 速度 PI（抗积分饱和），避免电流突变与积分饱和。

失速检测力控判定：力矩达阈值 90% 且输出轴转速 < 5 rpm，维持 150 ms 判定夹取完成。

设计输出轴行程累积保护（含单圈角度溢出处理）与 CAN 通信看门狗。

自定义串口命令协议，支持力阈值、转速、行程等参数在线调节。

5. Gazebo 仿真支持
搭建完整 Gazebo 仿真环境，包含视觉插件与夹爪模型。

可用于算法验证，无需实机即可测试轨迹规划与控制逻辑。

📦 软件包说明
包名	功能
ar3_description	URDF/Xacro 模型描述
ar3_hardware_interface	ros2_control 硬件插件，实现 SystemInterface
teensy4.1	Teensy 4.1 固件（Arduino/PlatformIO 平台）
ar3_moveit_config	MoveIt 2 配置文件
ar3_gazebo	Gazebo 仿真
ar3_bringup	启动文件与轨迹示例

🚀 快速开始
1. 环境要求
操作系统：Ubuntu 24.04
ROS 2 发行版：Jazzy
硬件：
AR3 六轴机械臂（开环步进电机版）
Teensy 4.1 开发板
ASTRA PRO 深度相机（或任何 ROS 2 支持的 RGB-D 相机）
M2006 电机 + C610 电调（力控夹爪）

依赖：
bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-tools

2. 构建工作空间
bash
mkdir -p ~/ar3_ws/src
cd ~/ar3_ws/src
git clone https://github.com/2694328475/ar3_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

4. 烧录固件
使用 Arduino IDE 或 PlatformIO 打开 teensy4.1/teensy4.1.ino，选择开发板 Teensy 4.1，编译并上传。确保串口设备为 /dev/ttyACM0。

5. 启动真实机械臂控制
bash
ros2 launch ar3_hd_moveit_config demo.launch.py

7. Gazebo 仿真模式
bash
ros2 launch ar3_gazebo ar3_gazebo_bringup.launch.py
ros2 launch ar3_moveit_config demo.launch.py use_sim_time:=true

9. 通过 ros2_control 直接发送指令
编辑 ar3_description/urdf/ar3.urdf.xacro，取消 ros2_control 配置部分的注释。

启动轨迹控制：

bash
ros2 launch ar3_bringup ar3_trajectory.launch.py
📸 演示
Gazebo仿真演示
<img width="480" height="270" alt="animation_edited" src="https://github.com/user-attachments/assets/72e2e0ac-0798-4331-b809-e90396640faf" />

使用ros2_control和限位归零演示
<img width="480" height="270" alt="animation_edited (1)" src="https://github.com/user-attachments/assets/1f91e74e-7481-4735-b4cc-5169fd489639" />

真机运动演示
<img width="480" height="270" alt="animation_edited (2)" src="https://github.com/user-attachments/assets/17acd07d-3f86-4251-9a96-401c3b8dbc41" />

📄 许可证
本项目采用 MIT 许可证。
