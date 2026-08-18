from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_package',
            default_value='ar3_description',
            description='package with URDF'
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ar3.urdf.xacro',
            # default_value='camera_urdf.xacro',
            description='xacro file'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Teensy serial port'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='serial baudrate'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='launch RViz'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='ar3_trajectory_controllers.yaml',
            description='controller configuration'
        ),
    ]

    description_package = LaunchConfiguration('description_package')
    description_file    = LaunchConfiguration('description_file')
    serial_port         = LaunchConfiguration('serial_port')
    baudrate            = LaunchConfiguration('baudrate')
    start_rviz          = LaunchConfiguration('start_rviz')
    controllers_file    = LaunchConfiguration('controllers_file')

    # 生成机器人描述
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        ' serial_port:=', serial_port,
        ' baudrate:=', baudrate,
    ])
    robot_description = {'robot_description': robot_description_content}

    # 控制器配置文件路径
    controllers_config_path = PathJoinSubstitution(
        [FindPackageShare('ar3_bringup'), 'config', controllers_file]
    )

    # robot_state_publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )

    # ros2_control节点
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config_path],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # 加载joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 延迟加载joint_trajectory_controller
    joint_trajectory_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'ar3.rviz'])],
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rviz_node
    ])