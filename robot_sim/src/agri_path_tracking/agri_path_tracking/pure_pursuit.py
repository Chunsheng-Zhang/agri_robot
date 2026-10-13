#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    """四元数转换为偏航角。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # 可通过 ROS 参数调整
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('closed_path', True)

        self.lookahead = float(
            self.get_parameter('lookahead_distance').value
        )
        self.linear_speed = float(
            self.get_parameter('linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        control_rate = float(
            self.get_parameter('control_rate').value
        )
        self.closed_path = bool(
            self.get_parameter('closed_path').value
        )

        if self.lookahead <= 0.0:
            raise ValueError('lookahead_distance must be positive')
        if self.linear_speed < 0.0:
            raise ValueError('linear_speed cannot be negative')
        if self.max_angular_speed <= 0.0:
            raise ValueError('max_angular_speed must be positive')
        if control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        # 参考路径与里程计
        self.reference_path = None
        self.path_points = []
        self.path_frame = 'map'
        self.have_odom = False

        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 底盘速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 将实际轨迹发布给 RViz
        self.tracked_path = Path()
        self.tracked_path.header.frame_id = 'map'
        self.last_track_xy = None
        self.tracked_pub = self.create_publisher(
            Path,
            '/tracked_path',
            10
        )

        # 通过 TF 获取 map 坐标系下的机器人位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )
        self.tf_warning_shown = False

        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )

        self.get_logger().info(
            'Pure Pursuit started: '
            f'lookahead={self.lookahead:.2f} m, '
            f'speed={self.linear_speed:.2f} m/s, '
            f'rate={control_rate:.1f} Hz'
        )

    def path_callback(self, msg):
        if not msg.poses:
            self.path_points = []
            self.get_logger().warning('Received an empty reference path')
            return

        new_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y
            )
            for pose in msg.poses
        ]

        if len(new_points) < 2:
            self.path_points = []
            self.get_logger().warning(
                'Reference path must contain at least 2 points'
            )
            return

        changed = (
            len(new_points) != len(self.path_points)
            or not self.path_points
        )

        self.path_points = new_points
        self.reference_path = msg
        self.path_frame = msg.header.frame_id or 'map'

        if changed:
            self.get_logger().info(
                f'Reference path received: '
                f'{len(self.path_points)} points, '
                f'frame={self.path_frame}'
            )

    def odom_callback(self, msg):
        # 里程计到达后，允许控制器工作。
        # 实际的 map 位姿通过 map -> odom -> base_footprint TF 获取。
        self.have_odom = True

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def get_robot_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.path_frame,
            'base_footprint',
            Time()
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        x = translation.x
        y = translation.y
        yaw = quaternion_to_yaw(rotation)

        return x, y, yaw

    def find_nearest_index(self, x, y):
        """寻找距离机器人最近的参考路径点。"""
        best_index = 0
        best_distance = float('inf')

        for i, (px, py) in enumerate(self.path_points):
            distance_sq = (px - x) ** 2 + (py - y) ** 2

            if distance_sq < best_distance:
                best_distance = distance_sq
                best_index = i

        return best_index

    def find_lookahead_target(self, nearest_index):
        """从最近路径点开始，沿路径向前累计前视距离。"""
        count = len(self.path_points)
        distance_sum = 0.0

        for step in range(1, count + 1):
            next_index = nearest_index + step

            if self.closed_path:
                next_index %= count
            elif next_index >= count:
                return self.path_points[-1]

            previous_index = (
                next_index - 1
                if self.closed_path or next_index > 0
                else 0
            )

            x1, y1 = self.path_points[previous_index]
            x2, y2 = self.path_points[next_index]

            distance_sum += math.hypot(x2 - x1, y2 - y1)

            if distance_sum >= self.lookahead:
                return self.path_points[next_index]

        return self.path_points[nearest_index]

    def record_actual_path(self, x, y, yaw):
        """每行驶约 0.1 m，记录一个实际轨迹点。"""
        if self.last_track_xy is not None:
            moved = math.hypot(
                x - self.last_track_xy[0],
                y - self.last_track_xy[1]
            )
            if moved < 0.1:
                return

        pose = PoseStamped()
        pose.header.frame_id = self.path_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.tracked_path.header.frame_id = self.path_frame
        self.tracked_path.header.stamp = pose.header.stamp
        self.tracked_path.poses.append(pose)

        self.last_track_xy = (x, y)

        self.tracked_pub.publish(self.tracked_path)

    def control_callback(self):
        # 数据尚未准备好时，明确发送零速度
        if not self.path_points or not self.have_odom:
            self.publish_stop()
            return

        try:
            x, y, yaw = self.get_robot_pose()
            self.tf_warning_shown = False

        except TransformException as exc:
            if not self.tf_warning_shown:
                self.get_logger().warning(
                    f'Waiting for TF '
                    f'{self.path_frame} -> base_footprint: {exc}'
                )
                self.tf_warning_shown = True

            self.publish_stop()
            return

        # 记录机器人实际轨迹
        self.record_actual_path(x, y, yaw)

        # 1. 找到最近的路径点
        nearest_index = self.find_nearest_index(x, y)

        # 2. 找到前视目标点
        target_x, target_y = self.find_lookahead_target(
            nearest_index
        )

        # 3. 计算目标点相对机器人航向的夹角
        dx = target_x - x
        dy = target_y - y
        target_distance = math.hypot(dx, dy)

        if target_distance < 0.05:
            self.publish_stop()
            return

        target_heading = math.atan2(dy, dx)
        alpha = normalize_angle(target_heading - yaw)

        # 4. Pure Pursuit 曲率
        curvature = 2.0 * math.sin(alpha) / target_distance

        # 5. 转换为差速底盘速度指令
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.linear_speed * curvature

        # 限制最大角速度
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, cmd.angular.z)
        )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
