import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from xacro import process_file

def generate_launch_description():
    # 获取各个包的共享目录
    model_pkg_share = get_package_share_directory('ar3_description')
    gazebo_pkg_share = get_package_share_directory('ar3_gazebo')

    # 文件路径
    world_file = os.path.join(gazebo_pkg_share, 'world', 'empty.sdf')
    urdf_file = os.path.join(model_pkg_share, 'urdf', 'ar3_gazebo_urdf.xacro')

    # 处理 xacro 文件，得到机器人描述
    doc = process_file(urdf_file)
    robot_desc = doc.toxml()

    # 启动 Gazebo（使用 ros_gz_sim 的 launch 文件）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
    )

    # 启动 robot_state_publisher，发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 在 Gazebo 中生成机器人实体
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ar3',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'          # 确保底座接触地面
        ],
        output='screen'
    )

    # 使用 spawner 加载 joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 使用 spawner 加载 joint_trajectory_controller
    spawn_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    # 发送初始位置命令（假设零位是安全姿态，若需要调整请修改 positions 数组）
    send_initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal', '/joint_trajectory_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            '{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 1}}]}}'
        ],
        output='screen'
    )

    # 定义事件执行顺序
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_trajectory_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_trajectory_controller,
                on_exit=[send_initial_pose]
            )
        )
    ])