# 调整相机位置使用
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明 launch 参数：模型文件的路径（支持 .urdf 或 .xacro）
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('ar3_description'),  # 替换成你的包名
            'urdf', 'camera_urdf.xacro'
        ]),
        description='URDF/Xacro model file path'
    )

    # 使用 Command 预处理 xacro 文件（如果文件以 .xacro 结尾）
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False  # 根据实际情况设置
        }]
    )

    # joint_state_publisher_gui 节点（可选，用于手动调节关节）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            FindPackageShare('ar3_description').find('ar3_description'),
            'rviz', 'display.rviz'
        )],
        output='screen'
    )

    # 可选：如果希望 joint_state_publisher_gui 在 robot_state_publisher 之后启动，可以添加事件处理
    # 但这里简单并行启动即可

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])