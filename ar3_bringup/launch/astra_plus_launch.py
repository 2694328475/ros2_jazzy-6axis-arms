from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,           # 显式启用彩色流
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'color_format': 'RGB',          # 小写的rgb不能正常使用彩色图
                'enable_depth': True,            # 显式启用深度流
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
                'depth_format': 'Y11',
                'enable_point_cloud': True,
                'camera_info_url': '/home/wyf/ros2_ws/src/ar3_bringup/config/color_Astra_Pro_Plus.yaml'
            }],
            output='screen'
        )
    ])
