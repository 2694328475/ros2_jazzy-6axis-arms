#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer
import tf2_ros
import math
import numpy as np

class CameraPoseTuner(Node):
    def __init__(self):
        super().__init__('camera_pose_tuner')

        # 适配你的机器人：父连杆和相机根坐标
        self.ee_frame = "link_4"
        self.camera_frame = "astra_camera_mount"

        # 初始偏移（位置 + 四元数，请用你原URDF中的origin值）
        self.offset_x = 0.0   # 改成你原来的 xyz 值
        self.offset_y = 0.0
        self.offset_z = 0.0
        # 初始姿态（单位四元数 = 无旋转）
        self.orient = {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 创建交互标记服务器
        self.im_server = InteractiveMarkerServer(self, "camera_drag_marker")
        self.create_interactive_marker()

        # 高频定时器确保持续广播
        self.timer = self.create_timer(0.02, self.publish_tf)

        self.get_logger().info("Camera tuner with rotation ready. Drag AND rotate in RViz!")

    def create_interactive_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = self.ee_frame
        im.name = "camera_marker"
        im.description = "6-DOF camera adjust"
        im.pose.position.x = self.offset_x
        im.pose.position.y = self.offset_y
        im.pose.position.z = self.offset_z
        im.pose.orientation.w = 1.0
        im.scale = 0.2

        controls = []

        # 三个移动轴
        axes = [
            (1.0, 0.0, 0.0, "move_x"),
            (0.0, 1.0, 0.0, "move_y"),
            (0.0, 0.0, 1.0, "move_z"),
        ]
        for ax_x, ax_y, ax_z, name in axes:
            ctrl = InteractiveMarkerControl()
            ctrl.name = name
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = ax_x
            ctrl.orientation.y = ax_y
            ctrl.orientation.z = ax_z
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            controls.append(ctrl)

        # 三个旋转轴
        rot_axes = [
            (1.0, 0.0, 0.0, "rotate_x"),
            (0.0, 1.0, 0.0, "rotate_y"),
            (0.0, 0.0, 1.0, "rotate_z"),
        ]
        for rx, ry, rz, name in rot_axes:
            ctrl = InteractiveMarkerControl()
            ctrl.name = name
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = rx
            ctrl.orientation.y = ry
            ctrl.orientation.z = rz
            ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            controls.append(ctrl)

        im.controls = controls

        # 插入标记
        self.im_server.insert(im)

        # 注册回调
        def feedback_cb(fb: InteractiveMarkerFeedback):
            if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                # 更新位置
                self.offset_x = fb.pose.position.x
                self.offset_y = fb.pose.position.y
                self.offset_z = fb.pose.position.z
                # 更新姿态
                self.orient['qx'] = fb.pose.orientation.x
                self.orient['qy'] = fb.pose.orientation.y
                self.orient['qz'] = fb.pose.orientation.z
                self.orient['qw'] = fb.pose.orientation.w
                # 归一化（保险）
                norm = math.sqrt(self.orient['qx']**2 + self.orient['qy']**2 +
                                 self.orient['qz']**2 + self.orient['qw']**2)
                if norm > 0:
                    self.orient['qx'] /= norm
                    self.orient['qy'] /= norm
                    self.orient['qz'] /= norm
                    self.orient['qw'] /= norm
                self.get_logger().info(
                    f"Pos: x={self.offset_x:.6f}, y={self.offset_y:.6f}, z={self.offset_z:.6f} | "
                    f"Ori: qx={self.orient['qx']:.6f}, qy={self.orient['qy']:.6f}, qz={self.orient['qz']:.6f}, qw={self.orient['qw']:.6f}"
                )

        self.im_server.setCallback("camera_marker", feedback_cb)
        self.im_server.applyChanges()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ee_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = self.offset_x
        t.transform.translation.y = self.offset_y
        t.transform.translation.z = self.offset_z
        t.transform.rotation.x = self.orient['qx']
        t.transform.rotation.y = self.orient['qy']
        t.transform.rotation.z = self.orient['qz']
        t.transform.rotation.w = self.orient['qw']
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = CameraPoseTuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()