#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
真实履带式农业机器人 Pure Pursuit 路径跟踪节点。

订阅：
    /reference_path
        nav_msgs/msg/Path

    /agri_localization/position
        geometry_msgs/msg/PointStamped

    /mgx23/attitude
        geometry_msgs/msg/Vector3

发布：
    /cmd_vel
        geometry_msgs/msg/Twist

    /real_tracked_path
        nav_msgs/msg/Path

说明：
    1. 不使用编码器。
    2. 不依赖 /odom 或 TF。
    3. 定位数据来自 MGX23 坐标转换节点。
    4. 航向数据来自 MGX23。
    5. 定位坐标与参考路径必须使用相同的坐标系。
    6. 本节点不负责底盘电机的底层闭环控制。
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist, Vector3, PoseStamped
from nav_msgs.msg import Path


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    """从四元数提取 ROS yaw，单位为弧度。"""
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(sin_yaw, cos_yaw)


class RealPurePursuit(Node):

    def __init__(self):
        super().__init__('real_pure_pursuit')

        # ==================================================
        # 1. 参数
        # ==================================================

        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('heading_mode', 'ros')
        self.declare_parameter('yaw_sign', 1.0)

        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )

        self.linear_speed = float(
            self.get_parameter('linear_speed').value
        )

        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        self.control_rate = float(
            self.get_parameter('control_rate').value
        )

        self.heading_mode = str(
            self.get_parameter('heading_mode').value
        )

        self.yaw_sign = float(
            self.get_parameter('yaw_sign').value
        )

        if self.lookahead_distance <= 0.0:
            raise ValueError(
                'lookahead_distance 必须大于 0'
            )

        if self.control_rate <= 0.0:
            raise ValueError(
                'control_rate 必须大于 0'
            )

        if self.linear_speed < 0.0:
            raise ValueError(
                'linear_speed 不能小于 0'
            )

        if self.max_angular_speed <= 0.0:
            raise ValueError(
                'max_angular_speed 必须大于 0'
            )

        if self.heading_mode not in ('ros', 'north_cw'):
            raise ValueError(
                "heading_mode 必须是 'ros' 或 'north_cw'"
            )

        # ==================================================
        # 2. 输入数据状态
        # ==================================================

        self.position_received = False
        self.attitude_received = False
        self.path_received = False

        # 车辆在 local_map 中的位置
        self.position_x = 0.0
        self.position_y = 0.0

        # 转换到 ROS yaw 约定后的车辆航向
        self.vehicle_yaw = 0.0

        # 参考路径点
        self.path_points = []
        self.path_yaws = []
        self.path_frame = 'map'

        # 最近路径点索引
        self.nearest_index = 0

        # 轨迹记录
        self.tracked_path = Path()
        self.tracked_path.header.frame_id = 'local_map'

        self.log_counter = 0

        # ==================================================
        # 3. 订阅
        # ==================================================

        self.path_sub = self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10
        )

        self.position_sub = self.create_subscription(
            PointStamped,
            '/agri_localization/position',
            self.position_callback,
            10
        )

        self.attitude_sub = self.create_subscription(
            Vector3,
            '/mgx23/attitude',
            self.attitude_callback,
            10
        )

        # ==================================================
        # 4. 发布
        # ==================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tracked_path_pub = self.create_publisher(
            Path,
            '/real_tracked_path',
            10
        )

        # ==================================================
        # 5. 控制定时器
        # ==================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info(
            '真实车辆 Pure Pursuit 节点已启动'
        )

        self.get_logger().info(
            f'前视距离：{self.lookahead_distance:.2f} m'
        )

        self.get_logger().info(
            f'线速度：{self.linear_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'控制频率：{self.control_rate:.1f} Hz'
        )

        self.get_logger().info(
            f'航向模式：{self.heading_mode}'
        )

        self.get_logger().warn(
            '本节点会发布 /cmd_vel；测试前请确认没有其他控制节点'
            '同时向该话题发布指令。'
        )

    # ======================================================
    # 6. 参考路径回调
    # ======================================================

    def path_callback(self, msg):
        """接收参考路径。"""

        if len(msg.poses) < 2:
            self.get_logger().warn(
                '参考路径点不足，至少需要两个点'
            )
            return

        points = []
        yaws = []

        for pose_stamped in msg.poses:
            pose = pose_stamped.pose

            x = float(pose.position.x)
            y = float(pose.position.y)

            if not (
                math.isfinite(x)
                and math.isfinite(y)
            ):
                self.get_logger().warn(
                    '参考路径包含无效坐标，忽略本次路径'
                )
                return

            points.append((x, y))
            yaws.append(
                quaternion_to_yaw(pose.orientation)
            )

        self.path_points = points
        self.path_yaws = yaws
        self.path_frame = msg.header.frame_id or 'map'
        self.path_received = True

        self.nearest_index = 0

        self.tracked_path = Path()
        self.tracked_path.header.frame_id = self.path_frame

        start_x, start_y = self.path_points[0]

        self.get_logger().info(
            f'收到参考路径：{len(points)} 个点，'
            f'坐标系={self.path_frame}，'
            f'起点=({start_x:.3f}, {start_y:.3f})'
        )

    # ======================================================
    # 7. 定位回调
    # ======================================================

    def position_callback(self, msg):
        """接收 GNSS 转换后的局部位置。"""

        x = float(msg.point.x)
        y = float(msg.point.y)

        if not (
            math.isfinite(x)
            and math.isfinite(y)
        ):
            self.position_received = False
            return

        self.position_x = x
        self.position_y = y
        self.position_received = True

    # ======================================================
    # 8. 航向回调
    # ======================================================

    def attitude_callback(self, msg):
        """
        接收 MGX23 姿态。

        默认假定：
            msg.z = ROS yaw，单位 rad。

        如果设备输出为从正北开始、顺时针增加的航向角，
        可将 heading_mode 设置为 north_cw。

        具体定义应以 MGX23 手册为准。
        """

        raw_yaw = float(msg.z)

        if not math.isfinite(raw_yaw):
            self.attitude_received = False
            return

        if self.heading_mode == 'ros':
            yaw = raw_yaw

        else:
            # 北向为 0、顺时针为正，转换成 ROS yaw。
            yaw = math.pi / 2.0 - raw_yaw

        yaw *= self.yaw_sign

        self.vehicle_yaw = normalize_angle(yaw)
        self.attitude_received = True

    # ======================================================
    # 9. 查找最近路径点
    # ======================================================

    def find_nearest_index(self, x, y):
        """寻找离车辆当前位置最近的参考路径点。"""

        min_distance_sq = float('inf')
        nearest_index = 0

        # 从上一次最近点附近开始搜索，减少不必要的计算。
        # 为兼容车辆重新定位和较大位置变化，保留全路径搜索。
        for i, (px, py) in enumerate(self.path_points):
            distance_sq = (
                (px - x) ** 2
                + (py - y) ** 2
            )

            if distance_sq < min_distance_sq:
                min_distance_sq = distance_sq
                nearest_index = i

        return nearest_index

    # ======================================================
    # 10. 寻找前视目标点
    # ======================================================

    def find_lookahead_point(self, x, y, nearest_index):
        """
        从最近路径点开始向前寻找目标点。

        当前版本按路径点顺序搜索。
        闭合路径在 CSV 中包含首尾重复点，
        因此最后一个点与第一个点重合。
        """

        point_count = len(self.path_points)

        for index in range(nearest_index, point_count):
            px, py = self.path_points[index]

            distance = math.hypot(
                px - x,
                py - y
            )

            if distance >= self.lookahead_distance:
                return index, px, py

        # 剩余路径不足前视距离时，选择最后一个点。
        index = point_count - 1
        px, py = self.path_points[index]

        return index, px, py

    # ======================================================
    # 11. Pure Pursuit 控制计算
    # ======================================================

    def calculate_control(self, x, y, yaw):
        """
        Pure Pursuit 算法。

        alpha = 目标点方向 - 当前航向

        curvature = 2 * sin(alpha) / Ld

        angular_speed = linear_speed * curvature
        """

        nearest_index = self.find_nearest_index(x, y)

        (
            target_index,
            target_x,
            target_y
        ) = self.find_lookahead_point(
            x,
            y,
            nearest_index
        )

        dx = target_x - x
        dy = target_y - y

        target_angle = math.atan2(dy, dx)

        alpha = normalize_angle(
            target_angle - yaw
        )

        target_distance = math.hypot(dx, dy)

        if target_distance < 1e-6:
            curvature = 0.0

        else:
            curvature = (
                2.0 * math.sin(alpha)
                / target_distance
            )

        angular_speed = (
            self.linear_speed * curvature
        )

        angular_speed = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_speed
            )
        )

        return (
            angular_speed,
            target_index,
            target_x,
            target_y,
            alpha
        )

    # ======================================================
    # 12. 发布车辆跟踪轨迹
    # ======================================================

    def publish_tracked_position(self, x, y, yaw):
        """将当前位置写入跟踪轨迹并发布。"""

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.path_frame

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.tracked_path.header.stamp = now
        self.tracked_path.header.frame_id = self.path_frame

        self.tracked_path.poses.append(pose)

        self.tracked_path_pub.publish(
            self.tracked_path
        )

    # ======================================================
    # 13. 控制循环
    # ======================================================

    def control_loop(self):
        """定时执行路径跟踪计算。"""

        if not self.path_received:
            return

        if not self.position_received:
            return

        if not self.attitude_received:
            return

        # 直接使用定位坐标。
        # 不做初始位置平移或旋转。
        x = self.position_x
        y = self.position_y
        yaw = self.vehicle_yaw

        (
            angular_speed,
            target_index,
            target_x,
            target_y,
            alpha
        ) = self.calculate_control(
            x,
            y,
            yaw
        )

        # 生成 Twist 指令。
        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_speed

        self.cmd_pub.publish(cmd)

        # 记录跟踪轨迹。
        self.publish_tracked_position(
            x,
            y,
            yaw
        )

        # 每秒左右输出一次状态信息。
        self.log_counter += 1

        log_interval = max(
            1,
            round(self.control_rate)
        )

        if self.log_counter % log_interval == 0:
            self.get_logger().info(
                f'位置=({x:.2f}, {y:.2f}) m，'
                f'航向={yaw:.3f} rad，'
                f'最近点={self.nearest_index}，'
                f'目标点={target_index}，'
                f'目标=({target_x:.2f}, {target_y:.2f})，'
                f'航向误差={alpha:.3f} rad，'
                f'角速度={angular_speed:.3f} rad/s'
            )


def main(args=None):
    rclpy.init(args=args)

    node = RealPurePursuit()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()